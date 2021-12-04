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
    print("Failed to initialize I2C")
    
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

# automatic white balance
#camera.iso = 100
#time.sleep(2)
    
#camera.shutter_speed = camera.exposure_speed
#camera.exposure_mode = 'off'
    
#g = camera.awb_gains
#camera.awb_mode = 'off'
#camera.awb_gains = g

lower = np.array([110, 50, 50])
upper = np.array([130, 255, 255]) # assign upper and lower bounds for blue in HSV
    
kernel = np.ones((20, 20), np.uint8) # generate kernel of 1's for morphology

n = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    try:
        img = frame.array
    
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # generates HSV version of image
        mask = cv2.inRange(hsv, lower, upper) # generates mask for blue based on HSV image
            
        output = cv2.bitwise_and(img, img, mask = mask) # applies mask to image
        output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) # closes holes in masked image
        output = cv2.blur(output, (6, 6)) # blurs image
    
        #cv2.imshow('Test', np.hstack([img, output]))
        key = cv2.waitKey(1) & 0xFF
    
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
        ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            
        items = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        contours = items[0] if len(items) == 2 else items[1]
    
    except:
        print("Contour detection failed")
    
    #cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    #cv2.imshow('Test', np.hstack([img, output]))
    
    if(n > -1):
    
        #gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # converts image to grayscale
        #ret,gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY) # converts grayscale image to binary image
            
        #gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        try:
        
            if(len(contours) > 0):
            
                contour = max(contours, key=cv2.contourArea)
                #((x, y), radius) = cv2.minEnclosingCircle(contour)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                #print(box)
            
                m = cv2.moments(contour)
                center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
            
                if(cv2.contourArea(box) > 1000):
                    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
                    cv2.circle(img, center, 5, (0, 0, 255), -1)
                    #cv2.imshow('Test', img)
                    
                    #hull = cv2.convexHull(contour)
                    #cv2.convexityDefects(contour, hull, defects)
                    
                    width = np.sqrt((box[0][0] - box[1][0])^2 + (box[0][1] - box[1][1])^2)
                    
                    #width = box[0][0] - box[1][0]
                    
                    #print(width)
                    
                    relX = (center[0] - 320) / 640
                    angle = relX * -53.5 # calculates phi in degrees
                    
                            
            ####################################### UNCOMMENT TO SEE VIDEO ##############################       
                    
            #cv2.imshow('Test', img)
                    
            #############################################################################################
                    
        except:
            print("Angle calculation failed")
            
        try:
            if(width > 10):
                print("Cross")
                writeNumber(100)
                lcd.clear()
                time.sleep(1)
                
                lcd.message = "Cross found"
                        
            else:
                print(angle)
                print(angle)
                writeNumber(int(angle))
                lcd.clear()
                time.sleep(1)
            
                lcd.message = "Angle: %d" % angle
        
        #except:
            #print("I2C communcation failed")
            
            #if(radius > 100):
                #cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                #cv2.circle(img, center, 5, (0, 0, 255), -1)
                #cv2.imshow('Test', img)
            
            #contours = contours[0]
        #elif(len(contours) > 1):
         #   maxArea = cv2.contourArea(contours[0])
          #  i = 0
           # index = 0
            #for cont in contours:
             #   area = cv2.contourArea(cont)
              #  if(area > maxArea):
               #     maxArea = area
                #    index = i
                #i = i + 1
            #contours = contours[index]
    
        #if(len(contours) > 0):
         #   m = cv2.moments(contours)
          #  xPos = int(m["m10"] / m["m00"])
           # yPos = int(m["m01"] / m["m00"])
            #relX = (xPos - 320) / 640
            #angle = relX * -53.5 # calculates phi in degrees
    
            #print(angle)
            #print("X:", xPos)
            #print("Y:", yPos)
            #print()
            
            #cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
            #cv2.imshow('Test', np.hstack([img, output]))
            
    #cv2.imshow('Test', np.hstack([img, output]))
    
    n = n + 1
    if(n == 10):
        n = 0

    rawCapture.truncate(0) # empties camera buffer
    
    if key == ord("q"):
        cv2.destroyAllWindows()
        break
