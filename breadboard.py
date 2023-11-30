import os
import time
import busio
import digitalio
import board
import pwmio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


ledLanding = pwmio.PWMOut(board.D19, frequency=5000, duty_cycle=0)

ledBeacon = digitalio.DigitalInOut(board.D13)
ledBeacon.direction = digitalio.Direction.OUTPUT

ledWingRed = digitalio.DigitalInOut(board.D6)
ledWingRed.direction = digitalio.Direction.OUTPUT

ledWingGreen = digitalio.DigitalInOut(board.D5)
ledWingGreen.direction = digitalio.Direction.OUTPUT

btnBeacon = digitalio.DigitalInOut(board.D21)
btnBeacon.direction = digitalio.Direction.INPUT
btnBeacon.pull = digitalio.Pull.UP

btnWings = digitalio.DigitalInOut(board.D20)
btnWings.direction = digitalio.Direction.INPUT
btnWings.pull = digitalio.Pull.UP


# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

while True:


    # convert 16bit adc0 (0-65535) analog pin read into 0-100 volume level
    chan0Weight = remap_range(chan0.value, 0, 65535, 0, 1023)

    chan0Percent = remap_range(chan0.value, 0, 65535, 0, 100)

    print("="*40)
    print("Raw value: ", chan0Weight)
    print("Binary: ", bin(chan0Weight))
    print("Percent: ", chan0Percent, "%")
    print("="*40)

    
    ledLanding.duty_cycle = int(65535 * chan0Percent / 100)
    
    if btnBeacon.value:
        ledBeacon.value = True
    else:
        ledBeacon.value = False

    if btnWings.value:
        ledWingGreen.value = True
    else:
        ledWingGreen.value = False

    if btnBeacon.value and btnWings.value:
        ledWingRed.value = True
    else:
        ledWingRed.value = False
        

    #print(btnBeacon.value, " ", btnWings.value)
    

    time.sleep(0.1)
