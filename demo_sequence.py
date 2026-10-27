"""Low -> rise -> active -> land/center -> lower -> complete. No I/O."""
from controller import MotionController
from robot_config import SafetyError


class DemoSequence:
    def __init__(self, config, low_pose='CROUCH_READY', dynamicity=1.):
        if low_pose in ('STAND', 'LIE', 'UP_READY'):
            raise SafetyError('A separate safe low pose is required')
        config.pose(low_pose)
        self.motion = MotionController(config, low_pose, dynamicity=dynamicity)
        self.low_pose = low_pose
        self.phase = 'RISING'
        self.stopping = False
        self.motion.request_pose('STAND')

    def __getattr__(self, name):
        return getattr(self.motion, name)

    @property
    def state(self):
        return 'DEMO_' + self.phase + '_' + self.motion.state

    @property
    def settled(self):
        return self.phase == 'COMPLETE'

    def set_command(self, vx=0., vy=0., wz=0.):
        # Never queue a movement key pressed during rise/lower.
        if self.phase == 'ACTIVE' and not self.stopping:
            self.motion.set_command(vx, vy, wz)
        else:
            self.motion.set_command()

    def request_pose(self, name):
        raise SafetyError('Demo owns posture transitions; use Space to stop or Escape to finish')

    def request_stop(self):
        self.stopping = True
        self.motion.set_command()
        if self.phase == 'ACTIVE':
            self.phase = 'STOPPING'

    def update(self, dt):
        pose = self.motion.update(dt)
        if self.phase == 'RISING' and self.motion.settled and self.motion.posture == 'STAND':
            self.phase = 'STOPPING' if self.stopping else 'ACTIVE'
        if self.phase == 'STOPPING' and self.motion.settled:
            self.motion.request_pose(self.low_pose)
            self.phase = 'LOWERING'
        elif self.phase == 'LOWERING' and self.motion.settled and self.motion.posture == self.low_pose:
            self.phase = 'COMPLETE'
        return pose
