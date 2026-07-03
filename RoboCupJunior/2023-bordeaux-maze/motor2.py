# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 George Zhang — https://github.com/TheYellowDuck

"""Drive-motor interface for the maze robot.

Wraps the four STS3215 serial bus servos (driven through ``pyb2.SServo``) behind a
small ``Motor`` class — straight-line driving, in-place turns, odometry from servo
position, and battery voltage — plus a few status-LED helpers. Run directly
(``python motor2.py`` on the OpenMV device) for a quick motor self-test.
"""

from pyb2 import *
import time
from pyb import millis, LED


# Uses STS3215 serial bus servos; this is the motor interface used by the robot.
class Motor:
    def __init__(self):
        print("create new motor")
        self.motor = SServo('LP1', [11, 12, 21, 22])

    def set_speeds(self, speeds):
        self.motor.set_speeds(speeds)

    def run(self, left, right):
        self.motor.set_speeds([-left, -left, right, right])

    def stop(self):
        self.motor.set_speeds([0, 0, 0, 0])

    def is_stopped(self):
        # Servo reports 0 while still moving.
        r = self.motor.is_stopped()
        return r == 0

    def forward_ms(self, speed, ms):
        """Drive forward for ``ms`` milliseconds; return distance travelled in steps."""
        t0 = millis()
        p0 = self.motor.get_position(2)
        c = 0
        last_p2 = 0
        p2 = last_p2
        self.run(speed, speed)
        while True:
            t1 = millis()
            if t1 - t0 > ms:
                break
            p1 = self.motor.get_position(2)
            p2 = (p1 - p0) % 4096
            if last_p2 == 0 and p2 > 4090:
                p2 = 0
            if p2 < 2048 and last_p2 > 2048:
                c += 1
            last_p2 = p2

        self.stop()
        s = p2 + 4096 * c
        return s

    def forward_cm(self, speed, cm):
        pass  # not implemented

    def forward_step(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([-s, -s, s, s])
        time.sleep_ms(t)
        print('finished turn')

    def turn_right(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([-s, -s, -s, -s])
        time.sleep_ms(t)
        print('finished turn')

    def turn_left(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')

    def turn_back(self, s=None, t=5000):
        if s is None:
            s = 7000 - 50 - 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')

    def backward_ms(self, speed, ms):
        pass  # not implemented

    def backward_cm(self, speed, cm):
        pass  # not implemented

    def voltage(self):
        return self.motor.voltage()


def drop_cube():
    print("Drop a cube")


def flash_blue(count=10000):
    led = LED(3)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)


def flash_red(count=5):
    led = LED(1)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)


def flash_green(count=5):
    led = LED(2)
    for i in range(count):
        led.on()
        time.sleep_ms(500)
        led.off()
        time.sleep_ms(500)


if __name__ == '__main__':
    print("Motor test..")
    m = Motor()
    print("\n Voltage: ", m.voltage())
    flash_red(3)
