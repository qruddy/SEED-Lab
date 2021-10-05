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
    
    # autmatic white balance
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
    
    lower = np.array([20, 50, 50])
    upper = np.array([40, 255, 255]) # assign upper and lower bounds for yellow in HSV
    
    kernel = np.ones((20, 20), np.uint8) # generate kernel of 1's for morphology
    
    while(1): # loop until the porgram is manually stopped
    
        # grab an image from the camera
        #print("Capturing Image...")
        try:
            camera.capture(rawCapture, format="bgr")
            image = rawCapture.array
        except:
            print("Failed to capture")

    # save the image to the disk
        #print("Saving image")
        try:
            cv2.imwrite('test.jpg', image)
        except:
            print("Couldn't save")

    # display the image
        #print("Loading image")
        try:
            img = cv2.imread('test.jpg')
            img = img[:,:,::-1] # flips image colors
            img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC) # decreases image size by half
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # generates HSV version of image
            mask = cv2.inRange(hsv, lower, upper) # generates mask for yellow based on HSV image
            
            output = cv2.bitwise_and(img, img, mask = mask) # applies mask to image
            output[500:544, 0:960] = 0 # sets bottom ~10% of image to black to account for problematic pixels
            output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) # closes holes in masked image
            output = cv2.blur(output, (6, 6)) # blurs image
            
            gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
            ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            pos = np.nonzero(gray) # creates array of all the pixels that correspond to the hexagon
            
            #cv2.imshow('Test', np.hstack([img, output]))
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            
            if(pos[0].size != 0): # checks if the hexagon is in the image
                #print("x: ", np.mean(pos[1]))
                #print("y: ", np.mean(pos[0])) # prints mean x and y positions
                
                relX = (np.mean(pos[1]) - 480) / 960
                relY = (np.mean(pos[0]) - 270) / 540 # calculates mean vertical and horizontal phi postions
                
                phiX = relX * 30
                phiY = relY * -25
                
                if(phiX >= 0):
                    if(phiY >= 6.25):
                        angle = 90
                    elif (phiY <= -6.25):
                        angle = 270
                    else:
                        angle = 0
                else:
                    if(phiY >= 6.25):
                        angle = 90
                    elif (phiY <= -6.25):
                        angle = 270
                    else:
                        angle = 180
                        
                #print("Phi X: ", phiX)
                #print("Phi Y: ", phiY)
                print(angle)
                lcd.clear()
                writeNumber(int(angle/2))
                time.sleep(1)
                
                lcd.message = "Setpoint: %d" % angle
                #print("Phi X: ", relX * 30)
                #print("Phi Y: ", relY * 25) # prints mean vertical and horizontal phi positions
        except:
            print("Couldn't load")
            
        rawCapture.truncate(0) # empties camera buffer