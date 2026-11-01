"""Synthetic hardware only; never power a real relay during tests."""
import copy
import json
from pathlib import Path
import unittest

from feedback_hw import AcquisitionError
from feedback_identify import identify
from robot_config import JOINTS,RobotConfig,load_config
from robot_io import DryRunOutput


class Clock:
    def __init__(self): self.t=0.
    def now(self): return self.t
    def sleep(self,s): self.t+=s


class IdentificationExecutorTests(unittest.TestCase):
    def setUp(self):
        self.layout=json.loads((Path(__file__).resolve().parents[1]/'config'/'feedback-identification.json').read_text())
        self.layout['electrical'].update(adc_supply_v_measured={str(i):3.3 for i in range(4)},
            feedback_min_v_measured={str(i):0. for i in range(4)},
            feedback_max_v_measured={str(i):3. for i in range(4)},
            adaptation_circuit_verified=True,motion_electrically_verified=True,
            measurement_evidence='SYNTHETIC TEST FIXTURE ONLY')
        old=load_config(); data=copy.deepcopy(old.data)
        data['geometry']['verified_on_robot']=True
        data['hardware']['calibration_verified_on_robot']=True
        self.robot=RobotConfig(data,copy.deepcopy(old.poses))
        self.clock=Clock(); self.events=[]; self.output=None; self.scans=0
        self.reader_error=None; self.interrupt_motion=False; self.overvoltage=False; self.fail_off=False

    def output_factory(self,config):
        owner=self
        class Output(DryRunOutput):
            def open(self,pose):
                owner.events.append('power_on'); super().open(pose)
            def write(self,pose,dt):
                if owner.interrupt_motion and abs(pose['FL_sh']-owner.robot.pose('STAND')['FL_sh'])>.3:
                    raise KeyboardInterrupt
                owner.events.append(('write',pose.copy()))
                super().write(pose,dt)
            def close(self):
                owner.events.append('power_off')
                super().close()
                if owner.fail_off: self.cleanup_errors.append('injected relay power-off failure')
        self.output=Output(config)
        return self.output

    def reader_factory(self,layout):
        owner=self
        class Reader:
            cleanup_errors=[]
            def open(self): owner.events.append('adc_open')
            def scan(self):
                owner.scans+=1; owner.clock.sleep(.03)
                if owner.reader_error: raise owner.reader_error
                powered=owner.output is not None and owner.output.armed
                rows={}
                for mux,leg in enumerate(('FL','FR','RL','RR')):
                    for ain in range(4):
                        offset=0.
                        if powered and ain<3:
                            joint=f'{leg}_{("sh","up","lo")[ain]}'
                            offset=owner.output.last_pose[joint]-owner.robot.pose('STAND')[joint]
                        volts=(1.5+.12*offset+((owner.scans%3)-1)*.001) if powered else .001
                        if owner.overvoltage and powered: volts=3.5
                        rows[f'{mux}:{ain}']={'volts':volts,'lsb_v':.003 if mux<3 else .0001875,
                                              'saturated':False}
                return rows
            def close(self): owner.events.append('adc_close')
        return Reader()

    def run_trial(self,**kwargs):
        return identify(self.layout,self.robot,confirmed_supported=True,repeats=2,samples_per_window=3,
                        output_factory=self.output_factory,reader_factory=self.reader_factory,
                        clock=self.clock.now,sleep=self.clock.sleep,**kwargs)

    def test_full_synthetic_identification_collects_powered_baseline_and_proposal(self):
        result=self.run_trial()
        self.assertTrue(result['completed_without_error'],result.get('error'))
        self.assertTrue(result['powered_baseline_collected'])
        self.assertLess(result['prepower_format_scan']['0:0']['volts'],.01)
        self.assertGreater(result['baseline_v']['0:0'][0],1.)
        self.assertEqual(len(result['trials']),12*2*5)
        self.assertTrue(result['proposal']['all_joints_proposed'])
        self.assertFalse(result['absolute_angle_calibration'])
        self.assertEqual(self.events[-2:],['power_off','adc_close'])
        stand=self.robot.pose('STAND')
        for event in self.events:
            if isinstance(event,tuple):
                delta=[j for j in JOINTS if abs(event[1][j]-stand[j])>1e-8]
                self.assertLessEqual(len(delta),1)
                self.assertTrue(all(abs(event[1][j]-stand[j])<=1.+1e-8 for j in JOINTS))

    def test_unmeasured_electrical_data_cannot_touch_hardware(self):
        self.layout['electrical']['adc_supply_v_measured']=None
        result=self.run_trial()
        self.assertFalse(result['power_on_attempted']); self.assertEqual(self.events,[])

    def test_missing_readiness_and_excessive_amplitude_cannot_touch_hardware(self):
        result=identify(self.layout,self.robot,confirmed_supported=False,
                        output_factory=self.output_factory,reader_factory=self.reader_factory)
        self.assertFalse(result['power_on_attempted']); self.assertEqual(self.events,[])
        result=self.run_trial(amplitude_deg=3.1)
        self.assertFalse(result['power_on_attempted']); self.assertEqual(self.events,[])

    def test_adc_format_error_before_power_has_no_output_instance(self):
        self.reader_error=AcquisitionError('injected ADS1015 incompatible format')
        result=self.run_trial()
        self.assertFalse(result['power_on_attempted']); self.assertIsNone(self.output)
        self.assertEqual(self.events,['adc_open','adc_close'])

    def test_interrupt_cuts_power_before_adc_cleanup_without_return_motion(self):
        self.interrupt_motion=True
        result=self.run_trial()
        self.assertIn('KeyboardInterrupt',result['error'])
        self.assertEqual(self.events[-2:],['power_off','adc_close'])
        last=[event[1] for event in self.events if isinstance(event,tuple)][-1]
        self.assertNotEqual(last,self.robot.pose('STAND'))
        self.assertIsNone(result['proposal'])

    def test_powered_out_of_range_and_poweroff_failure_are_reported(self):
        self.overvoltage=True; self.fail_off=True
        result=self.run_trial()
        self.assertIn('electrical range',result['error'])
        self.assertIn('injected relay',result['cleanup_errors'][0])
        self.assertFalse(result['completed_without_error'])
        self.assertEqual(self.events[-2:],['power_off','adc_close'])


if __name__=='__main__': unittest.main()
