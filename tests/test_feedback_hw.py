import copy
import json
from pathlib import Path
import unittest

from feedback_hw import AcquisitionError,MuxADSReader,decode_conversion,single_shot_config,validate_layout
from feedback_readout import acquisition,unresolved_proposal

ROOT=Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT/'config'/'feedback-identification.json').read_text())


class Clock:
    def __init__(self): self.t=0.
    def now(self): return self.t
    def sleep(self,seconds): self.t+=seconds


class Bus:
    def __init__(self,number):
        self.number=number; self.mask=0; self.calls=[]; self.closed=False
        self.cfg=0x8583; self.busy=0; self.stuck=False; self.corrupt=False; self.fail_conversion=False
    def write_byte(self,address,value):
        self.calls.append(('write_byte',address,value)); self.mask=value
    def read_byte(self,address):
        self.calls.append(('read_byte',address)); return self.mask
    def write_i2c_block_data(self,address,register,data):
        self.calls.append(('write_adc',address,register,list(data)))
        self.cfg=(data[0]<<8)|data[1]
        self.busy=2 if self.cfg&0x8000 else 0
    def read_i2c_block_data(self,address,register,length):
        self.calls.append(('read_adc',address,register,length))
        if register==1:
            if self.stuck: value=self.cfg&0x7fff
            elif self.busy:
                self.busy-=1; value=self.cfg&0x7fff
            else: value=self.cfg|0x8000
            if self.corrupt: value ^= 0x1000
        else:
            if self.fail_conversion: raise OSError('injected conversion failure')
            value=0x1000
        return [value>>8,value&255]
    def close(self): self.calls.append(('close',)); self.closed=True


class FeedbackHardwareTests(unittest.TestCase):
    def reader(self,bus=None):
        clock=Clock(); bus=bus or Bus(1)
        return MuxADSReader(config(),bus_factory=lambda _:bus,clock=clock.now,sleep=clock.sleep),bus

    def test_model_specific_signed_decoding_and_voltage(self):
        for model,raw,code,volts in [('ADS1015',[0x10,0],256,.768),
             ('ADS1015',[0xff,0xf0],-1,-.003),('ADS1115',[0x10,0],4096,.768),
             ('ADS1115',[0xff,0xff],-1,-.0001875)]:
            r=decode_conversion(model,raw)
            self.assertEqual(r['code'],code); self.assertAlmostEqual(r['volts'],volts)
        self.assertTrue(decode_conversion('ADS1015',[0x7f,0xf0])['saturated'])
        self.assertTrue(decode_conversion('ADS1115',[0x80,0])['saturated'])
        with self.assertRaises(AcquisitionError): decode_conversion('ADS1015',[0,1])

    def test_single_shot_configuration_bitfields_and_data_rates(self):
        self.assertEqual(single_shot_config('ADS1015',0),0xc103)
        self.assertEqual(single_shot_config('ADS1115',0),0xc183)
        for model in ('ADS1015','ADS1115'):
            for ain in range(4):
                word=single_shot_config(model,ain,4.096)
                self.assertEqual((word>>12)&7,4+ain)
                self.assertEqual((word>>9)&7,1)
                self.assertEqual(word&3,3)

    def test_complete_scan_touches_only_mux71_and_adc48_and_one_channel_at_a_time(self):
        r,b=self.reader(); r.open(); scan=r.scan(); r.close()
        self.assertEqual(len(scan),16)
        self.assertEqual(scan['0:0']['model'],'ADS1015')
        self.assertEqual(scan['3:0']['model'],'ADS1115')
        self.assertAlmostEqual(scan['0:0']['volts'],scan['3:0']['volts'])
        for call in b.calls:
            if call[0]=='close': continue
            self.assertIn(call[1],(0x71,0x48))
            if call[0]=='write_byte': self.assertIn(call[2],(0,1,2,4,8))
            if call[0]=='write_adc':
                self.assertEqual(call[2],1)
                self.assertEqual(len(call[3]),2)
        self.assertTrue(b.closed); self.assertEqual(b.mask,0)

    def test_timeout_wrong_config_and_conversion_error_deselect_before_close(self):
        for problem in ('stuck','corrupt','fail_conversion'):
            r,b=self.reader(); r.open(); setattr(b,problem,True)
            with self.assertRaises((AcquisitionError,OSError)):
                r.scan()
            self.assertEqual(b.mask,0)
            r.close(); self.assertTrue(b.closed)
            self.assertEqual(b.calls[-3:], [('write_byte',0x71,0),('read_byte',0x71),('close',)])

    def test_competing_mux_change_is_rejected(self):
        r,b=self.reader(); r.open()
        with r.select(0):
            b.mask=3
            with self.assertRaises(AcquisitionError): r.read_input(0)
        r.close(); self.assertEqual(b.mask,0)

    def test_inflight_previous_conversion_is_waited_out(self):
        r,b=self.reader(); r.open(); b.busy=3
        with r.select(2): r.read_input(1)
        writes=[i for i,c in enumerate(b.calls) if c[0]=='write_adc']
        self.assertGreaterEqual(sum(c[0]=='read_adc' for c in b.calls[:writes[0]]),4)
        r.close()

    def test_continuous_mode_is_stopped_before_new_single_shot(self):
        r,b=self.reader(); r.open(); b.cfg=0x0083
        with r.select(0): r.read_input(0)
        values=[(c[3][0]<<8)|c[3][1] for c in b.calls if c[0]=='write_adc']
        self.assertFalse(values[0]&0x8000); self.assertTrue(values[0]&0x0100)
        self.assertEqual(values[-1],0xc103)
        r.close()

    def test_layout_rejects_70_wrong_adc_and_mapping_changes(self):
        for key,value in [('mux_address',0x70),('adc_address',0x49),('bus',2)]:
            c=config(); c[key]=value
            with self.assertRaises(ValueError): validate_layout(c)
        c=config(); c['devices'][3]['model']='ADS1015'
        with self.assertRaises(ValueError): validate_layout(c)
        c=config(); c['pca9685_channels']['RL_up']=5
        with self.assertRaises(ValueError): validate_layout(c)

    def test_baseline_preserves_samples_noise_and_unknown_mapping(self):
        r,b=self.reader()
        report=acquisition(config(),5,reader_factory=lambda _:r)
        self.assertTrue(report['passed']); self.assertFalse(report['motion_performed'])
        self.assertFalse(report['electrical_safety_confirmed']); self.assertTrue(b.closed)
        self.assertEqual(len(report['statistics']),16)
        self.assertEqual(report['statistics']['0:0']['stdev_v'],0.)
        proposal=unresolved_proposal(config())
        self.assertEqual(len(proposal['rows']),12)
        self.assertTrue(all(row['input'] is None for row in proposal['rows']))
        self.assertTrue(all(row['mux_channel']==3 for row in proposal['rows'] if row['joint'].startswith('RR_')))

    def test_error_report_is_created_only_after_cleanup(self):
        r,b=self.reader(); b.fail_conversion=True
        report=acquisition(config(),5,reader_factory=lambda _:r)
        self.assertFalse(report['passed']); self.assertIn('injected',report['error'])
        self.assertTrue(b.closed); self.assertEqual(b.mask,0)

    def test_cleanup_errors_are_reported_even_when_conversion_failed(self):
        class FailingCleanupBus(Bus):
            def write_byte(self,address,value):
                if self.fail_conversion and value==0:
                    raise OSError('injected deselection failure')
                super().write_byte(address,value)
        b=FailingCleanupBus(1)
        r,b=self.reader(b)
        original=r.open
        def opened():
            original(); b.fail_conversion=True; return r
        r.open=opened
        result=acquisition(config(),5,reader_factory=lambda _:r)
        self.assertFalse(result['passed'])
        self.assertTrue(result['cleanup_errors']); self.assertTrue(b.closed)


if __name__=='__main__': unittest.main()
