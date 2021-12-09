# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# try to import packages for Arduino communication
try:
    import smbus
    import board
    import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
    from math import pi

    lcd_columns = 16
    lcd_rows = 2

    bus = smbus.SMBus(1)
    i2c = board.I2C()

    address = 0x04

    def writeNumber(value):
        bus.write_byte(address, value)
        return -1
    
except:
    print("Failed to initialize I2C")
    
# initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

lower = np.array([95, 50, 50])
upper = np.array([135, 255, 255]) # assign upper and lower bounds for blue in HSV
    
kernel = np.ones((5, 5), np.uint8) # generate kernel of 1's for morphology

count = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): # loop through frames
    
    try:
        img = frame.array # analyze each frame
    
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # generates HSV version of image
        mask = cv2.inRange(hsv, lower, upper) # generates mask for blue based on HSV image
            
        output = cv2.bitwise_and(img, img, mask = mask) # applies mask to image
        output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) # closes holes in masked image
        output = cv2.blur(output, (6, 6)) # blurs image
    
        key = cv2.waitKey(1) & 0xFF # define break key
    
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
        ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            
        items = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # calculate contours
    
        contours = items[0] if len(items) == 2 else items[1] # organizes contours
    
    except:
        print("Contour detection failed")
        
    width = -1 # initialize rectangle width
        
    try:
        
        if(len(contours) > 0): # if there are contours
            
            contour = max(contours, key=cv2.contourArea) # find max contour
            rect = cv2.minAreaRect(contour) # find minimum area rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            m = cv2.moments(contour) # find center of contour
            center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
            
            if(cv2.contourArea(box) > 1000): # if the contour area is large enough
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2) # draw minimum area rectangle
                cv2.circle(img, center, 5, (0, 0, 255), -1) # draw center of contour
                    
                width = np.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
                length = np.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2) # calculate width and length
                    
                relX = (center[0] - 320) / 640
                angle = relX * -53.5 # calculates phi in degrees
                    
                            
####################################### UNCOMMENT TO SEE VIDEO ##############################       
                    
        #cv2.imshow('Test', img) # display video
                    
#############################################################################################
                    
    except:
        print("Angle calculation failed")
            
    if(width != -1 and box[1][0] >= 30 and box[0][0] <= 610 and length >= 150 and np.abs(width - length) <= 50): # incrcease count if the rectangle is square-shaped
        count = count + 1
            
    if(count > 1): # if the cross is seen twice, send a stop flag
        print("Cross")
        print(np.abs(width - length))
        try:
            writeNumber(100)
        except:
            print("Arduino Communication Failed")
                        
    elif(width != -1): # else, if the rectangle is seen and is not a square, send the angle
        print(angle)
        try:
            writeNumber(int(angle))
        except:
            print("Arduino Communication Failed")

    rawCapture.truncate(0) # empties camera buffer
    
    if key == ord("q"): # if q is pressed, exit the loop
        cv2.destroyAllWindows()
        break
