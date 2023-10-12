import time
import neopixel
import board
import numpy

strip = neopixel.NeoPixel(board.D18, 300, brightness=1, auto_write=True)

strip.fill((0,0,0))

for i in range(0,300):
    strip[i] = (255,255,255)
    time.sleep(.05)