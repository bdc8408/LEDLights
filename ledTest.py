import time
import neopixel
import board
import numpy

pixels = neopixel.NeoPixel(board.D18, 300, brightness=1, auto_write=True)

b = 1
dt = 1/60
t = 0

endtime = 300

for i in range(0,round(endtime/dt)):
    r = round(255/2*(numpy.sin(2*3.1415*.05*t)+1))
    g = round(255/2*(numpy.sin(2*(3.1415*.05*t+1/3))+1))
    b = round(255/2*(numpy.sin(2*(3.1415*.05*t+2/3))+1))
    
    pixels.fill((r,g,b))
    t = t + dt
    print((r,g,b))
