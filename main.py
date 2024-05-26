from machine import TouchPad, Pin,PWM,I2C,
from MX1508 import *
from tcs34725 import *
from time import sleep, sleep_ms
from neopixel import NeoPixel
from bleuart import *
from micropython import const
import uasyncio as asio

i2c_bus = I2C(0, sda=Pin(23), scl=Pin(22))
tcs = TCS34725(i2c_bus)
tcs.gain(4)#gain must be 1, 4, 16 or 60
tcs.integration_time(80)
debug=0
NUM_OF_LED = 1
np = NeoPixel(Pin(27), NUM_OF_LED)
col_id=0
color=['Cyan','Black','Yellow','Navy','Orange','Green','Red']

motor = [MX1508(26, 25),MX1508(33,32),MX1508(19,21),MX1508(16,17)]

sp=300
sp1=300

an=90
on=0
an2=10
on2=0

col_id=0
comand=''
pwm = PWM(Pin(12,Pin.OUT))
pwm.freq(50)
pwm.duty(0)

pwm2 = PWM(Pin(14,Pin.OUT))
pwm2.freq(50)
pwm2.duty(0)
        
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def servo(pin, angle):
    pin.duty(map(angle, 0, 180, 20, 120))#на 360
def servo2(pin, angle):
    pin.duty(map(angle, 0, 180, 20, 120))#на 180
    
def on_rx():
    global comand,on,on2
    on = 1
    on2 = 1
    comand=uart.read().decode().strip()
    comand=comand[2:]


ble = bluetooth.BLE()
uart = BLEUART(ble,"LapaLaser")
uart.irq(handler=on_rx)

async def color_det():
    global col_id
    await asio.sleep_ms(0)
    rgb=tcs.read(1)
    r,g,b=rgb[0],rgb[1],rgb[2]
    h,s,v=rgb_to_hsv(r,g,b)
    if (h>340)or(h<25):
        col_id=6
        np[0] = (255,0,0)
    if 25<h<60:
        col_id=4
        np[0] = (255,128,0)
    if 60<h<120:
        col_id=2
        np[0] = (255,255,0)
    if 120<h<140:
        col_id=5
        np[0] = (0,255,0)
    if 140<h<240:
        if v>130:
            col_id=0
            np[0] = (51,255,255)
        if 30<v<40:
            col_id=3
            np[0] = (0,0,153)
        if v<30:
            col_id=1
            np[0] = (0,0,0)
    np.write()
    #print('Color is {}. R:{} G:{} B:{} H:{:.0f} S:{:.0f} V:{:.0f}'.format(color[col_id],r,g,b,h,s,v))
                
async def send_color(int_ms):     
    while(1):
        try:
            while True:
                uart.write(color[col_id]+"\n")
                await asio.sleep_ms(int_ms)
        except KeyboardInterrupt:
            pass


async def do_it(int_ms):
    global an,on, an2, on2, sp
    while 1:
        await asio.sleep_ms(int_ms)
        #print(comand)
        sp=300
        while comand=='516':
            for i in range(4):
                motor[i].forward(sp)
            time.sleep(0.1)
            sp+=100
            if sp>1023:
                sp=1023
            #print(sp)
        sp=500
        if comand=='615':
            for i in range(4):
                motor[i].reverse(1023)
        if comand=='714':
            motor[0].forward(sp1)
            motor[1].forward(sp1)
            motor[2].reverse(sp1)
            motor[3].reverse(sp1)
        if comand=='813':
            motor[2].forward(sp1)
            motor[3].forward(sp1)
            motor[0].reverse(sp1)
            motor[1].reverse(sp1)
        if comand=='11:':
            an=150
            servo(pwm, 150)
        if comand=='10;':
            an=150
            servo(pwm, 90)
        if comand=='219':
            servo(pwm, 30)
        if comand=='20:':
            servo(pwm,90)
        while comand=='318':
            an2 += 10
            if an2>35:
                an2=35
            servo(pwm2, an2)
            time.sleep(0.08)
        while comand=='417':
            an2 -= 10
            if an2<10:
                an2=10
            servo(pwm2, an2)
            time.sleep(0.08)
        if (comand=='507')or(comand=='606') or(comand=='705')or(comand=='804'):
            for i in range(4):
                motor[i].stop()
        await color_det()

# define loop
loop = asio.get_event_loop()

#create looped tasks
loop.create_task(do_it(5))
loop.create_task(send_color(100))
# loop run forever
loop.run_forever()

#uart.close()