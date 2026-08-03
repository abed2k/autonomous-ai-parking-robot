from machine import I2C, Pin, PWM
import time
import sys
import uselect



# ================= IMU =================
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
IMU_ADDR = 104
i2c.writeto_mem(IMU_ADDR, 0x6B, b'\x00')

def read_raw_data(addr):
    high = i2c.readfrom_mem(IMU_ADDR, addr, 1)[0]
    low = i2c.readfrom_mem(IMU_ADDR, addr + 1, 1)[0]
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

# ================= ENCODERS =================
class Encoder:
    def __init__(self, pin_a, pin_b, direction=1):
        self.count = 0
        self.dir = direction
        self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.a.irq(trigger=Pin.IRQ_RISING, handler=self.cb)
    def cb(self, pin):
        if self.a.value() == self.b.value():
            self.count += self.dir
        else:
            self.count -= self.dir

enc1 = Encoder(2, 3, -1)
enc2 = Encoder(6, 7, 1)
enc3 = Encoder(14, 15, 1)
enc4 = Encoder(18, 19, -1)

# ================= MOTORS =================
class Motor:
    def __init__(self, rpwm, lpwm):
        self.r = PWM(Pin(rpwm))
        self.l = PWM(Pin(lpwm))
        self.r.freq(1000)
        self.l.freq(1000)
        self.stop()
    def drive(self, speed):
        speed = max(min(speed, 100), -100)
        duty = int(abs(speed) * 655.35)
        if speed > 0:
            self.r.duty_u16(duty)
            self.l.duty_u16(0)
        elif speed < 0:
            self.r.duty_u16(0)
            self.l.duty_u16(duty)
        else:
            self.stop()
    def stop(self):
        self.r.duty_u16(0)
        self.l.duty_u16(0)

motors = [
    Motor(4, 5),
    Motor(8, 9),
    Motor(16, 17),
    Motor(20, 21)
]

# ================= SERIAL =================
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

# ================= MAIN LOOP =================
while True:
    try:
        events = poll.poll(1)
        if events:
            cmd = sys.stdin.readline().strip()
            if cmd:
                left, right = map(int, cmd.split(','))
                motors[1].drive(left)
                motors[2].drive(left)
                motors[0].drive(right)
                motors[3].drive(right)
    except:
        pass
    acc_x = read_raw_data(0x3B)
    acc_y = read_raw_data(0x3D)
    acc_z = read_raw_data(0x3F)
    gx = read_raw_data(0x43)
    gy = read_raw_data(0x45)
    gz = read_raw_data(0x47)
    print("{},{},{},{},{},{},{},{},{},{}".format(
        acc_x, acc_y, acc_z,
        gx, gy, gz,
        enc1.count, enc2.count, enc3.count, enc4.count
    ))
    time.sleep(0.05)
