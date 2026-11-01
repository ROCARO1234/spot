"""Mux/ADS-only acquisition. Never imports or addresses motor hardware.

Models come from explicit wiring configuration, not from I2C autodetection.
PGA range is a conversion scale, not an allowed analog pin voltage.
"""
from contextlib import contextmanager
import math
import time

MODELS={'ADS1015':(12,0), 'ADS1115':(16,4)}  # resolution, DR bits for 128 SPS
PGA={6.144:0,4.096:1,2.048:2,1.024:3,.512:4,.256:5}
EXPECTED_PCA={'FL_sh':15,'FL_up':14,'FL_lo':13,'FR_sh':3,'FR_up':2,'FR_lo':1,
              'RL_sh':11,'RL_up':10,'RL_lo':9,'RR_sh':7,'RR_up':6,'RR_lo':5}


class AcquisitionError(RuntimeError):
    pass


def finite(value,label):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(label+': expected finite number')
    return float(value)


def single_shot_config(model,input_channel,full_scale_v=6.144):
    if model not in MODELS or type(input_channel) is not int or input_channel not in range(4):
        raise ValueError('Unsupported ADC model or input')
    if full_scale_v not in PGA:
        raise ValueError('Unsupported PGA scale')
    return (0x8000 | ((4+input_channel)<<12) | (PGA[full_scale_v]<<9)
            | 0x0100 | (MODELS[model][1]<<5) | 3)


def decode_conversion(model,data,full_scale_v=6.144):
    if model not in MODELS or len(data)!=2 or any(type(x) is not int or not 0<=x<=255 for x in data):
        raise ValueError('Invalid conversion register bytes')
    if full_scale_v not in PGA: raise ValueError('Unsupported PGA scale')
    raw=(data[0]<<8)|data[1]
    signed=raw-65536 if raw&0x8000 else raw
    bits=MODELS[model][0]
    if bits==12:
        if raw&15:
            raise AcquisitionError(f'ADS1015 reserved low bits are nonzero (register 0x{raw:04x}); model/configuration may be wrong')
        signed >>= 4
    return {'register_u16':raw,'code':signed,'volts':signed*full_scale_v/(1<<(bits-1)),
            'lsb_v':full_scale_v/(1<<(bits-1)),
            'saturated':signed in (-(1<<(bits-1)),(1<<(bits-1))-1)}


def validate_layout(config):
    if (config.get('schema_version')!=1 or type(config.get('schema_version')) is not int
            or type(config.get('bus')) is not int or config['bus']!=1
            or type(config.get('mux_address')) is not int or config['mux_address']!=0x71
            or config.get('adc_address')!=0x48 or type(config.get('adc_address')) is not int
            or config.get('mux_protocol')!='one-hot-control-byte'
            or config.get('sample_rate_sps')!=128
            or config.get('full_scale_v') not in PGA):
        raise ValueError('This layout requires bus1, mux0x71, ADC0x48, one-hot selection and 128 SPS')
    devices=config.get('devices')
    if not isinstance(devices,list) or len(devices)!=4:
        raise ValueError('Exactly four configured ADCs required')
    expected={0:('ADS1015',None),1:('ADS1015',None),2:('ADS1015',None),3:('ADS1115','RR')}
    actual={}
    for d in devices:
        if not isinstance(d,dict) or set(d)!={'mux_channel','model','leg'}:
            raise ValueError('Invalid device declaration')
        ch=d['mux_channel']
        if type(ch) is not int or ch not in range(4) or ch in actual:
            raise ValueError('Duplicate or invalid mux channel')
        actual[ch]=(d['model'],d['leg'])
    if actual!=expected: raise ValueError('ADC layout differs from the supplied mixed-ADC wiring')
    if config.get('pca9685_channels')!=EXPECTED_PCA:
        raise ValueError('PCA channel labels differ from the supplied command mapping; no auto-correction')
    return config


class MuxADSReader:
    def __init__(self,config,*,bus_factory=None,clock=time.monotonic,sleep=time.sleep,timeout_s=.15):
        self.config=validate_layout(config)
        self.bus_factory=bus_factory
        self.clock,self.sleep=clock,sleep
        self.timeout_s=finite(timeout_s,'conversion timeout')
        if not .02<=self.timeout_s<=1.: raise ValueError('Conversion timeout outside .02..1 s')
        self.bus=None
        self.selected=None
        self.cleanup_errors=[]
        self.lock=None
        self.devices={d['mux_channel']:d['model'] for d in config['devices']}

    def open(self):
        if self.bus is not None: raise AcquisitionError('Reader already open')
        if self.bus_factory is None:
            # Advisory lock complements stopping all other I2C users. It cannot
            # exclude an unrelated program that ignores this lock.
            import fcntl
            self.lock=open('/tmp/spot-feedback-i2c1.lock','a')
            try:
                fcntl.flock(self.lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                from smbus2 import SMBus
                self.bus_factory=SMBus
            except BaseException:
                self.lock.close(); self.lock=None
                raise
        try:
            self.bus=self.bus_factory(self.config['bus'])
            self.bus.write_byte(0x71,0)
            if self.bus.read_byte(0x71)!=0: raise AcquisitionError('Mux could not deselect all channels')
        except BaseException:
            self.close()
            raise
        return self

    def _assert_selected(self):
        if self.bus is None or self.selected is None:
            raise AcquisitionError('No mux channel selected')
        if self.bus.read_byte(0x71)!=(1<<self.selected):
            raise AcquisitionError('Mux changed or multiple channels enabled; stop competing I2C processes')

    @contextmanager
    def select(self,channel):
        if self.bus is None or self.selected is not None or channel not in self.devices or type(channel) is not int:
            raise AcquisitionError('Invalid or nested mux selection')
        try:
            self.bus.write_byte(0x71,0)
            self.bus.write_byte(0x71,1<<channel)
            self.selected=channel
            self._assert_selected()
            yield
        finally:
            try:
                self.bus.write_byte(0x71,0)
                if self.bus.read_byte(0x71)!=0:
                    raise AcquisitionError('Mux did not confirm deselection')
            except Exception as exc:
                self.cleanup_errors.append('Mux deselection failed: '+str(exc))
                raise
            finally:
                self.selected=None

    def _read_word(self,register):
        self._assert_selected()
        data=self.bus.read_i2c_block_data(0x48,register,2)
        if len(data)!=2 or any(type(v) is not int or not 0<=v<=255 for v in data):
            raise AcquisitionError('Malformed ADC register read')
        return (data[0]<<8)|data[1]

    def _write_word(self,register,value):
        self._assert_selected()
        self.bus.write_i2c_block_data(0x48,register,[value>>8,value&255])

    def _wait_ready(self,expected=None):
        deadline=self.clock()+self.timeout_s
        while True:
            value=self._read_word(1)
            if expected is not None and (value&0x7fff)!=(expected&0x7fff):
                raise AcquisitionError('ADC configuration mismatch; reject possibly stale/wrong-input data')
            if value&0x8000: return
            if self.clock()>=deadline: raise AcquisitionError('ADC conversion timed out')
            self.sleep(.001)

    def read_input(self,input_channel,*,raw_only=False):
        if self.selected not in self.devices: raise AcquisitionError('No configured ADC selected')
        model=self.devices[self.selected]
        wanted=single_shot_config(model,input_channel,self.config['full_scale_v'])
        old=self._read_word(1)
        if not old&0x0100:
            self._write_word(1,(old|0x0100)&0x7fff)  # stop continuous mode after its current conversion
        self._wait_ready()
        self._write_word(1,wanted)
        self._wait_ready(wanted)
        raw=self._read_word(0)
        if raw_only:
            value={'register_u16':raw,'register_signed16':raw-65536 if raw&0x8000 else raw,
                   'code':None,'volts':None,'lsb_v':None,'saturated':None,
                   'format_not_contradicted':model=='ADS1115' or raw&15==0,
                   'model_verified':False}
        else:
            try:
                value=decode_conversion(model,[raw>>8,raw&255],self.config['full_scale_v'])
            except AcquisitionError as exc:
                raise AcquisitionError(f'mux{self.selected}/AIN{input_channel}: {exc}') from exc
        return value | {'mux_channel':self.selected,'input':input_channel,'model':model,
                        'timestamp_monotonic_s':self.clock(),'provenance':'ADC conversion; not servo angle'}

    def scan(self,*,raw_only=False,after_input=None):
        result={}
        for channel in sorted(self.devices):
            with self.select(channel):
                for input_channel in range(4):
                    result[f'{channel}:{input_channel}']=self.read_input(input_channel,raw_only=raw_only)
                    if after_input is not None: after_input()
        return result

    def close(self):
        if self.bus is not None:
            try:
                self.bus.write_byte(0x71,0)
                if self.bus.read_byte(0x71)!=0:
                    raise AcquisitionError('Mux did not confirm final deselection')
            except Exception as exc: self.cleanup_errors.append('Final mux deselection failed: '+str(exc))
            try: self.bus.close()
            except Exception as exc: self.cleanup_errors.append('I2C close failed: '+str(exc))
        self.bus=None; self.selected=None
        if self.lock is not None:
            try: self.lock.close()
            except Exception as exc: self.cleanup_errors.append('Lock close failed: '+str(exc))
            self.lock=None
