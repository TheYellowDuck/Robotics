from pyb2 import *
import time, math
from pyb import millis, LED


# use sts3215
# used by robot
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
        return
        while True:
            if self.is_stopped():
                break
            time.sleep_ms(100)

    def is_stopped(self):
        # check servo for True or false
        r = self.motor.is_stopped()
        return r == 0

    # return distance in steps
    def forward_ms(self, speed, ms):
        t0 = millis()
        p0 = self.motor.get_position(2)
        c = 0
        last_p2 = 0
        p2 = last_p2
        flag = 0
        self.run(speed, speed)
        while True:
            t1 = millis()
            if t1 - t0 > ms: break
            p1 = self.motor.get_position(2)
            p2 = (p1 - p0) % 4096
            if last_p2 == 0 and p2 > 4090:
                p2 = 0
                flag = 1
            if p2 < 2048 and last_p2 > 2048: c += 1
            last_p2 = p2
            # print('p1 p2 c', p1, p2, c)

        self.stop()
        s = p2 + 4096 * c
        # print('p0 p1 c s flag,', p0, p1, c, s,flag)
        return s

    def forward_cm(self, speed, cm):
        pass

    def forward_step(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([-s, -s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return

    def turn_right(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([-s, -s, -s, -s])
        time.sleep_ms(t)
        print('finished turn')
        return
        while True:
            time.sleep_ms(100)
            t = self.is_stopped()
            if t: break

    def turn_left(self, s=None, t=3000):
        if s is None:
            s = 3350 - 50 - 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return
        while True:
            time.sleep_ms(100)
            t = self.is_stopped()
            if t: break

    def turn_back(self, s=None, t=5000):
        if s is None:
            s = 7000 - 50 - 20
        self.motor.set_positions([s, s, s, s])
        time.sleep_ms(t)
        print('finished turn')
        return

    def backward_ms(self, speed, ms):
        pass

    def backward_cm(self, speed, cm):
        pass

    def voltage(self):
        v = self.motor.voltage()
        return v


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
    #flash_green(5)
    #flash_blue()
    # s = m.forward_ms(2000, 3200)  # 299.444cm
    # print('steps : ', s, s / 4096.0 * 62 * math.pi)

    # m.turn_right()
    # m.turn_left()
    # m.stop()
    # print(m.is_stopped())

    # print("Voltage: ", v)
