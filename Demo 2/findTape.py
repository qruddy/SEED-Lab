# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

try:
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
    
except:
    print("I2C Not Initialized") # initializes I2C communication with the LCD
    
found = False # flag for signaling if tape has been found
close = False # flag for signaling if tape is close
sentStop = False # flag to determine if stop flag has been sent

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
    
    lower = np.array([100, 100, 100])
    upper = np.array([130, 200, 200]) # assign upper and lower bounds for blue in HSV
    
    kernel = np.ones((20, 20), np.uint8) # generate kernel of 1's for morphology
    
    while(1): # loop until the porgram is manually stopped
    
        # grab an image from the camera
        #print("Capturing Image...")
        try:
            camera.capture(rawCapture, format="bgr")
            img = rawCapture.array
        except:
            print("Failed to capture")

    # save the image to the disk
        #print("Saving image")
        #try:
            #cv2.imwrite('test.jpg', img)
        #except:
            #print("Couldn't save")

    # display the image
        #print("Loading image")
        try:
            #img = cv2.imread('test.jpg')
            img = img[:,:,::-1] # flips image colors
            img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC) # decreases image size by half
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # generates HSV version of image
            mask = cv2.inRange(hsv, lower, upper) # generates mask for blue based on HSV image
            
        except:
            print("Mask Failed")
            
        try:
            output = cv2.bitwise_and(img, img, mask = mask) # applies mask to image
            output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) # closes holes in masked image
            output = cv2.blur(output, (6, 6)) # blurs image
            
        except:
            print("Morphology Failed")
            
        #cv2.imshow('Test', np.hstack([img, output]))
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
            
        try:
            gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
            ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            gray[500:544, 0:960] = 0 # sets bottom ~10% of image to black to account for problematic pixels
            gray[0:45, 0:960] = 0 # sets top 45 pixels to black to account for windows in the room
            
            gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # finds groups of blue
            if(len(contours) == 1):
                contours = contours[0]
                maxArea = cv2.contourArea(contours[0])
            elif(len(contours) != 0):
                maxArea = cv2.contourArea(contours[0])
                i = 0
                index = 0
                for cont in contours:
                    area = cv2.contourArea(cont)
                    if(area > maxArea):
                        maxArea = area
                        index = i
                    i = i + 1
                contours = contours[index] # finds largest group of blue, this is likely the tape
            
        except:
            print("Contours Failed")
            
        try:
            if(found == True and len(contours) != 0): # determines if tape is close by detecting where the edges in the masked image are
                edges = cv2.Canny(gray, 100, 255)
                indicies = np.where(edges != [0])
                coords = zip(indicies[0], indicies[1])
                for coord in coords:
                    if(coord[0] >= 490):
                        close = True
                        break
        except:
            print("Close Calculation Failed")
            
        try:        
            if(len(contours) != 0): # calculates the angle phi
                m = cv2.moments(contours)
                pos = int(m["m10"] / m["m00"])
                relX = (pos - 480) / 960
                angle = relX * -53.5 # calculates phi in degrees
            
        except:
            print("Angle Calculation Failed")
            
        if(found == False and len(contours) != 0 and maxArea >= 25): # if there is a large enough group of blue, send a flag
            print("Tape found")
            found = True
                
            try:
                lcd.clear()
                writeNumber(50)
                time.sleep(1)
                
                lcd.message = "Tape Found"
                    
            except:
                print("Found I2C Failed")
                
        if(found != False and close != True and len(contours) != 0 and maxArea >= 25): # if there is a large enough group of blue, send the angle
            print(angle)
                
            try:
                lcd.clear()
                writeNumber(int(angle))
                time.sleep(1)
                
                lcd.message = "Angle: %d" % angle
                    
            except:
                print("Angle I2C Failed")
            
        elif(found != False and close != False and sentStop != True): # if the tape is close and the stop flag has not been sent, send a stop flag
            print("Stop")
            sentStop = True
                
            try:
                lcd.clear()
                writeNumber(100)
                time.sleep(1)
                
                lcd.message = "Stop"
                    
            except:
                print("Stop I2C Failed")
            
        rawCapture.truncate(0) # empties camera buffer
