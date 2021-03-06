# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from math import pi

lcd_columns = 16
lcd_rows = 2

bus = smbus.SMBus(1)
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcd_columns,lcd_rows)

address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

if __name__ == '__main__':
 
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    
    # automatic white balance
    camera.iso = 100
    time.sleep(2)
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    
    rawCapture = PiRGBArray(camera)
 
    # allow the camera to warmup
    time.sleep(0.1)
    
    lower = np.array([105, 50, 50])
    upper = np.array([135, 255, 255]) # assign upper and lower bounds for blue in HSV
    
    kernel = np.ones((20, 20), np.uint8) # generate kernel of 1's for morphology
    
    while(1): # loop until the porgram is manually stopped
    
        # grab an image from the camera
        try:
            camera.capture(rawCapture, format="bgr")
            img = rawCapture.array
        except:
            print("Failed to capture")

    # display the image
        try:
            img = img[:,:,::-1] # flips image colors
            img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC) # decreases image size by half
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # generates HSV version of image
            mask = cv2.inRange(hsv, lower, upper) # generates mask for blue based on HSV image
            
            output = cv2.bitwise_and(img, img, mask = mask) # applies mask to image
            output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) # closes holes in masked image
            output = cv2.blur(output, (6, 6)) # blurs image
            
            gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
            ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            gray[500:544, 0:960] = 0 # sets bottom ~10% of image to black to account for problematic pixels
            
            gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours in image
            
            if(len(contours) == 1): # if there is only one contour, it is the tape
                contours = contours[0]
            else: # else, find the contour with the largest area, it is likely the tape
                maxArea = cv2.contourArea(contours[0])
                i = 0
                index = 0
                for cont in contours:
                    area = cv2.contourArea(cont)
                    if(area > maxArea):
                        maxArea = area
                        index = i
                    i = i + 1
                contours = contours[index]
                
            m = cv2.moments(contours) # find the moments of the contour
            pos = int(m["m10"] / m["m00"]) # calculate the x position of the center of the contour
            
            if(contours != None): # checks if the tape is in the image
                relX = (pos - 480) / 960 # calculates mean horizontal phi postion
                angle = relX * -53.5 # calculates phi in degrees
                
                print(angle)
                lcd.clear()
                writeNumber(int(angle))
                time.sleep(1)
                
                lcd.message = "Angle: %d" % angle
            
        except:
            print("Couldn't load")
            
        rawCapture.truncate(0) # empties camera buffer
