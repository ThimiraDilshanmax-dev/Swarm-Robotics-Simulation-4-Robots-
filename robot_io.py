"""Local gyro/encoder odometry, motors, and radio helpers. No global pose access."""
import json
import math
import random
from robot_config import WHEEL_RADIUS, TRACK, NAMES


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class RobotIO:
    def __init__(self, robot):
        self.robot = robot
        self.index = NAMES.index(robot.getName())
        self.step_ms = int(robot.getBasicTimeStep())
        self.dt = self.step_ms / 1000.0
        self.random = random.Random(420 + self.index)
        self.motors = [robot.getDevice(n) for n in ('left_motor', 'right_motor')]
        self.encoders = [robot.getDevice(n) for n in ('left_encoder', 'right_encoder')]
        for motor, encoder in zip(self.motors, self.encoders):
            motor.setPosition(float('inf'))
            motor.setVelocity(0)
            motor.setAcceleration(7)
            encoder.enable(self.step_ms)
        self.gyro = robot.getDevice('gyro')
        self.gyro.enable(self.step_ms)
        self.state_rx = robot.getDevice('state_receiver')
        self.feedback_rx = robot.getDevice('feedback_receiver')
        self.state_tx = robot.getDevice('state_sender')
        self.feedback_tx = robot.getDevice('feedback_sender')
        self.peer_rx = robot.getDevice('peer_receiver')
        self.peer_tx = robot.getDevice('peer_sender')
        self.x = self.y = self.heading = self.distance = 0.0
        self.v = self.w = self.command_v = self.command_w = 0.0
        self.last_encoders = None
        self.bias_sum = self.bias_count = self.bias = 0.0
        self.tick = 0

    def sense(self):
        self.tick += 1
        # Simulation assumptions: 720 counts/revolution and biased, noisy gyro.
        # This encoder resolution is not a claim about the selected hardware.
        quantum = 2 * math.pi / 720
        enc = [round(e.getValue()/quantum)*quantum for e in self.encoders]
        raw_w = self.gyro.getValues()[2]+0.003+self.index*0.0003+self.random.gauss(0,0.0008)
        if self.robot.getTime() <= 1.0:
            self.bias_sum += raw_w
            self.bias_count += 1
            self.bias = self.bias_sum/self.bias_count
        self.w = raw_w-self.bias
        ds = 0.0 if self.last_encoders is None else WHEEL_RADIUS*sum(a-b for a,b in zip(enc,self.last_encoders))/2
        self.last_encoders = enc
        old_heading = self.heading
        if self.robot.getTime() > 1.0:
            self.heading += self.w*self.dt
        self.x += ds*math.cos((old_heading+self.heading)/2)
        self.y += ds*math.sin((old_heading+self.heading)/2)
        self.distance += abs(ds)
        self.v += 0.15*(ds/self.dt-self.v)

    def drive(self,v,w):
        self.command_v += clip(v-self.command_v,-0.30*self.dt,0.30*self.dt)
        self.command_w += clip(w-self.command_w,-0.90*self.dt,0.90*self.dt)
        left = (self.command_v-self.command_w*TRACK/2)/WHEEL_RADIUS
        right = (self.command_v+self.command_w*TRACK/2)/WHEEL_RADIUS
        limit = max(1,abs(left)/8,abs(right)/8)
        self.motors[0].setVelocity(left/limit)
        self.motors[1].setVelocity(right/limit)

    def pose(self):
        return dict(x=self.x,y=self.y,heading=self.heading,v=self.v,w=self.w)

    @staticmethod
    def send(device,message):
        device.send(json.dumps(message).encode('utf-8'))

    @staticmethod
    def receive(device):
        while device.getQueueLength():
            try:
                message = json.loads(device.getString())
                strength = device.getSignalStrength()
                yield message, 1/math.sqrt(strength) if strength > 0 else None
            except (ValueError,TypeError):
                pass
            finally:
                device.nextPacket()

