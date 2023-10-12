import time
import board
import neopixel
from transitions import Machine
import numpy
import cv2
import os
from rpi_ws281x import *
import random as rand
import RPi.GPIO as GPIO
import threading


class LEDSync:
    states = ['standby', 'running', 'brightnessAdjust']

    def __init__(self):
        self.machine = Machine(model=self, states=self.states, initial='standby')
        #self.machine.add_transition('enable', 'standby', 'enabled', conditions=['is_switch_on'], after='start_animation')
        self.machine.add_transition('run', 'standby', 'running')
        self.machine.add_transition('stop', 'running', 'standby',after='stopAnimation')
        self.machine.add_transition('run', 'brightnessAdjust', 'running')
        self.machine.add_transition('run', 'brightnessAdjust', 'running')

        # Initialize your Neopixel LED strip here
        self.led_strip = neopixel.NeoPixel(board.D18, 300)  # Replace with your pin and number of LEDs
        self.led_strip.fill((0, 0, 0))  # Turn off all LEDs initially


    def stopAnimation(self):
        self.led_strip.fill((0,0,0))


def getTVimage():
    
    # Read a frame from the webcam
    ret, frame = cam.read()
    orig = frame
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the OpenCV frame to a PIL Image
    frame = cv2.resize(frame, (imgSize[0], imgSize[1]))
        
    return frame, orig

def computeColors(strip):
    
    def gamma_correction(value, gamma):
        # Apply gamma correction formula
        return int(pow(value / 255.0, gamma) * 255)
    
    def rgb_to_hsv(r, g, b):
      r /= 255
      g /= 255
      b /= 255
      maxc = max(r, g, b)
      minc = min(r, g, b)
      v = maxc
      if minc == maxc:
          return 0.0, 0.0, v
      s = (maxc-minc) / maxc
      rc = (maxc-r) / (maxc-minc)
      gc = (maxc-g) / (maxc-minc)
      bc = (maxc-b) / (maxc-minc)
      if r == maxc:
          h = 0.0+bc-gc
      elif g == maxc:
          h = 2.0+rc-bc
      else:
          h = 4.0+gc-rc
      h = (h/6.0) % 1.0
      return h, s, v
    
    def hsv_to_rgb(h, s, v):
        """
        Convert HSV (Hue, Saturation, Value) values to RGB (Red, Green, Blue) values.

        Parameters:
        - h (float): Hue value in the range [0, 1].
        - s (float): Saturation value in the range [0, 1].
        - v (float): Value (brightness) value in the range [0, 1].

        Returns:
        - tuple: RGB values as integers in the range [0, 255].
        """
        if s == 0:
            # Achromatic (gray) color, hue is undefined
            return round(v * 255), round(v * 255), round(v * 255)
        
        h *= 6  # Sector 0 to 5
        i = int(h)
        f = h - i  # Fractional part of h
        p = v * (1 - s)
        q = v * (1 - s * f)
        t = v * (1 - s * (1 - f))

        if i == 0:
            return round(v * 255), round(t * 255), round(p * 255)
        elif i == 1:
            return round(q * 255), round(v * 255), round(p * 255)
        elif i == 2:
            return round(p * 255), round(v * 255), round(t * 255)
        elif i == 3:
            return round(p * 255), round(q * 255), round(v * 255)
        elif i == 4:
            return round(t * 255), round(p * 255), round(v * 255)
        else:  # i == 5
            return round(v * 255), round(p * 255), round(q * 255)
    
    c = []
    npts = 4
    L = [-1+(2/(npts-1))*i for i in range(0,npts)]
    
    
    for i in range(0,len(ox)):
        
        
        p = image[oy[i],ox[i]]
        
        rSum=0
        gSum=0
        bSum=0
        
        for j in L:
            for k in L:
        
                samp = image[round(oy[i]+j*res),round(ox[i]+k*res)]
                rSum = rSum + samp[0]
                gSum = gSum + samp[1]
                bSum = bSum + samp[2]
        
        p = [rSum/npts,gSum/npts,bSum/npts]
        
        h,s,v = rgb_to_hsv(p[0],p[1],p[2])
        
       
        
        r,g,b = hsv_to_rgb(h,s,v)
        
        p = [round(r),round(g),round(b)]
        
        p = [min(gamma_correction(p[i], gamma),255) for i in range(0,3)]
        
        
        strip[i] = p
    return strip

def computeLEDLoc():

    #we must compute or measure pixel xy coords as measured in pixels from the TV origin (top left I'm assuming)
    #assume leds start at bottom right
    led_spacing = .65617 #in

    TV_wd = 60 #in
    TV_ht = 33 #in
    
    xi = []
    yi = []

    s = 0
    
    img_ht = imgSize[1]
    img_wd = imgSize[0]
    
    pSpacingx = TV_wd/img_wd #in/px
    pSpacingy = TV_ht/img_ht #in/px
    
    for i in range(0,n):
        if s < TV_ht:
            xi.append(0)
            yi.append(s)
        elif s < TV_ht+TV_wd:
            xi.append(xi[i-1]+led_spacing)
            yi.append(yi[i-1])
        elif s < TV_ht*2+TV_wd:
            xi.append(xi[i-1])
            yi.append(yi[i-1]-led_spacing)
        elif s < TV_ht*2+TV_wd*2:
            xi.append(xi[i-1]-led_spacing)
            yi.append(yi[i-1])
            
        
        
        s = s+led_spacing
    
    for i in range(0, len(xi)):
        xi[i] = TV_wd - xi[i]
        yi[i] = TV_ht - yi[i]
    
    img_ht = imgSize[1]
    img_wd = imgSize[0]
    
    pSpacingx = TV_wd/img_wd #in/px
    pSpacingy = TV_ht/img_ht #in/px
   
    
    px = []
    py = []
    
    for i in range(0, len(xi)):
        px.append(min(round(xi[i]/pSpacingx),img_wd-1))
        py.append(min(round(yi[i]/pSpacingy),img_ht-1))
       
    write_lists_to_file(px,py,'pxloc_infunc.txt')
    return px, py

def write_lists_to_file(list1, list2, filename):
    # Make sure both lists have the same length
    if len(list1) != len(list2):
        raise ValueError("Both lists must have the same length")

    # Open the file in write mode
    with open(filename, 'w') as file:
        # Iterate over the pairs of elements from both lists
        for item1, item2 in zip(list1, list2):
            # Write the pair to the file
            file.write(f"{item1}\t{item2}\n")


def computeOffsetPx():
    # dist = 50  # determines size of sub-image
    # res = 1  # % of sub-image to scan
    # imgSize = [w, h]

    cpx = round(imgSize[0] / 2)
    cpy = round(imgSize[1] / 2)

    mpx = [x-cpx for x in px]
    mpy = [y-cpy for y in py]
    
    ox = []
    oy = []

    cx = imgSize[0] - 2*dist
    cy = imgSize[1] - 2*dist

    for i in range(len(mpx)):
        x = mpx[i]
        y = mpy[i]

        u = [x,y]
        ulen = numpy.sqrt(u[0] ** 2 + u[1] ** 2)
        u = [-u[0] / ulen, -u[1] / ulen]
        
        h = max(abs(u[0]),abs(u[1]))
        a =  dist/h
        
        ox.append(round(x+a*u[0]))
        oy.append(round(y+a*u[1]))

    #ox = ox + cpx
    #oy = oy + cpy

    ox = [x+cpx for x in ox]
    oy = [y+cpy for y in oy]

    return ox, oy

def isblank(image1):
    return image1.shape == blankimg.shape and not(numpy.bitwise_xor(image1,blankimg).any())

def checkBlackLines(img):
    thresh = .4
    nBlack = 0
    
    ctResult = False
    
    nCrit = .4*len(colorAssignments)
    
    for p in colorAssignments:
        if p[0] == 0 and p[1] == 0 and p[2] == 0:
            nBlack += 1
            if nBlack > nCrit:
                ctResult = True
                break
            
    if not ctResult:
        return False
    
    nColumns = 10
    
    ht, wd, ch = img.shape
    
    columnResults = []
    
    for i in range(0,nColumns):
        evaluatingColumn = True
        
        C = rand.randint(0,wd-1)
        j = 0
        while evaluatingColumn:
            p = img[j,C]
            if p[0] != 0 and p[1] != 0 and p[2] != 0 or j > ht/2:
                columnResults.append(j)
                evaluatingColumn = False
            j+=1
    
    res = all(ele == columnResults[0] for ele in columnResults)
    
    return res

def getWideScreenInfo(img):
    
    thinking = True
    
    j = 1
    
    h,w,ch = img.shape
    
    while thinking:
        
        for i in range(0, w-1,10):
            p = img[j,i]
            if p[0] != 0 and p[1] != 0 and p[2] != 0:
                ofst = j+1
                thinking = False
            if j>w/2:
                ofst = j+1
                thinking = False
        
        j += 1
    
    aspx = w/(h - 2*ofst)
    
    return aspx, ofst

def computeWideScreenOx(img):
    #wideScreenImgAspect, wideScreenOffset
    h,w,ch = img.shape
    
    newoy = oy
    
    for i in range(len(ox)):
        p = newoy[i]
        if p < wideScreenOffset:
            newoy[i] = wideScreenOffset
        if p > h-wideScreenOffset:
            newoy[i] = h-wideScreenOffset
            
    return ox, newoy

def checkStillWideScreen(img):
    # sample n rows at the top of the screen, if all are black, return true. You have the original image.
    n = 4
    
    h,w,ch = img.shape
    
    for i in range(0,n):
        for j in range(0,w-1,5):
            p = img[i,j]
            if p[0] != 0 and p [1] != 0 and p[2] != 0:
                return False
        
    return True

def readGammaEncoder(Clk_last, clkPin, dtPin):
    
    DT_state = GPIO.input(dtPin)
    Clk_state = GPIO.input(clkPin)
    
    count = 0
    
    if Clk_last != Clk_state:
        if DT_state != Clk_state:
            print('gamma up')
            count = 1
            
        else:
            print('gamma down')
            count  = -1
        
    return Clk_state, count

'''
def get_tv_image_thread():
    global tv_image_data
    while True:
        tv_image_data = getTVImage()
        time.sleep(1)  # Adjust the interval as needed

# Function to run computeLEDcolors in the main thread
def compute_led_colors_main():
    global led_colors
    while True:
        led_colors = computeLEDcolors(strip)
        time.sleep(1)  # Adjust the interval as needed
        '''
        
def tv_image_thread():
    global image
    global orig
    
    while True:
        image, orig = getTVimage()

def led_data_thread():
    global strip
    while True:
        strip = computeColors(strip)


# Initialize the LED sync system
led_sync_system = LEDSync()
led_sync_system.run()
n = 300

GPIO.setmode(GPIO.BCM)


res = 15
dist = 50
gamma = 1.5

gammaEncoderClk_pin = 17
gammaEncoderDt_pin = 27

GPIO.setup(gammaEncoderClk_pin, GPIO.IN)
GPIO.setup(gammaEncoderDt_pin, GPIO.IN)

gammaEncoderClk_last = GPIO.input(gammaEncoderClk_pin)

cam = cv2.VideoCapture(0)

global image

# Read a frame from the webcam
ret, image = cam.read()

# Convert the OpenCV frame to a PIL Image
h,w,ch = image.shape

ambientMult = 1

cwd = os.getcwd()

blankimg = cv2.imread(cwd + '/blankimg.png')

#imgSize = [w,h]
aspct = 16/9
imgSize = [round(h*aspct),h]

px, py = computeLEDLoc()
ox, oy = computeOffsetPx()

tick = 0
tock = 0

lastBlackLineCheck = time.time()

displayMode = 'fullscreen'

image, orig = getTVimage()

n =len(ox)

global strip
strip = neopixel.NeoPixel(board.D18, n,auto_write=False)
colorAssignments = computeColors(strip)
wideScreenOffset = 0

currentlyblank = False

#get_tv_image_thread = threading.Thread(target=get_tv_image_thread)
image_thread = threading.Thread(target=tv_image_thread)
image_thread.start()

color_thread = threading.Thread(target=led_data_thread)
color_thread.start()

while True:
    
    gammaEncoderClk_last, gammachange = readGammaEncoder(gammaEncoderClk_last, gammaEncoderClk_pin, gammaEncoderDt_pin)
    
    if gammachange == 1 and gamma < 10:
        gamma+= .1
    if gammachange == -1 and gamma >= .3:
        gamma-= .1
    
    #image, orig = getTVimage()
    
    if led_sync_system.state == 'running':

        if(not currentlyblank):
            
            #strip = computeColors(strip)
            #for i in range(0,n):
            #    strip[i] = colorAssignments[i]
            
            strip.show()
            
        else:
            led_sync_system.stop()
            color_thread.stop()
            image_thread.stop()
    
    
    tock = time.time()
    print(f"Tick to tock: {tock-tick:0.4f}s",end='\r', flush=True)
    tick = time.time()
   
    #currentlyblank = isblank(orig)
    
    currentTime = time.time()
    
    if displayMode == 'widescreen' and currentTime - lastBlackLineCheck >= 5 and led_sync_system.state == 'running':
        if(not checkStillWideScreen(orig)):
            displayMode = 'fullscreen'
            ox, oy = computeOffsetPx()
            lastBlackLineCheck = time.time()
            print('going fullscreen')
            wideScreenOffset = 0
    
    if displayMode == 'fullscreen' and currentTime - lastBlackLineCheck >= 5 and led_sync_system.state == 'running':
        if(checkBlackLines(orig)):
            displayMode = 'widescreen'
            wideScreenImgAspect, wideScreenOffset = getWideScreenInfo(orig)
            
            ox_orig = ox
            oy_orig = oy
            
            ox, oy = computeWideScreenOx(image)
            print('going widescreen')
            lastBlackLineCheck = time.time()
    
    if led_sync_system.state == 'standby':
       
        timeSinceStart = time.time()
        
        while (time.time()-timeSinceStart<.5):
            image, orig = getTVimage()
            if not isblank(orig):
                led_sync_system.run()
                break
        
        time.sleep(1)
            
    
        
    #
    # Add more conditions here for chaning to/from different states
    #
