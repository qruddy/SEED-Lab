# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

if __name__ == '__main__':
 
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    
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
    
    boundaries = [ # HSV values of red, green, blue, and yellow
        ([0, 50, 50], [10, 255, 255]),
        ([50, 50, 50], [70, 255, 255]),
        ([105, 50, 50], [135, 255, 255]),
        ([20, 50, 50], [40, 255, 255])
        ]
    
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
        #try:
        img = cv2.imread('test.jpg')
        img = img[:,:,::-1] # flips image colors
        img = cv2.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv2.INTER_CUBIC) # decreases image size by half
        
        cent = img[216:324, 384:576] # center 10% of image
        
        #unique, counts = np.unique(cent.reshape(-1, 3), axis=0, return_counts=True) # finds the amount of unique colors in cent
        #cent[:,:,0], cent[:,:,1], cent[:,:,2] = unique[np.argmax(counts)] # converts cent to most common color
        
        #cv2.imshow('Average Center', cent) # displays cleaned masked image
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        hsv = cv2.cvtColor(cent, cv2.COLOR_BGR2HSV) # generates HSV version of image
            
        angle = 0
        #length = int((324 - 216) / 2)
        #width = int((576 - 384) / 2)
            
        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
                
            mask = cv2.inRange(hsv, lower, upper)
            out = cv2.bitwise_and(cent, cent, mask=mask)
            
            out[:,:,0], out[:,:,1], out[:,:,2] = np.average(out, axis=(0,1))
            
            #cv2.imshow('Average Center', np.hstack([cent, out])) # displays cleaned masked image
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
                
            if(np.any(out[0, 0] != 0)):
                print(angle)
                
            #print(out[0, 0])
                    
            angle = angle + 90
            
        #cv2.imshow('Average Center', np.hstack([cent, out])) # displays cleaned masked image
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
            
        #except:
            #print("Couldn't load")
            
        rawCapture.truncate(0) # empties camera buffer
        