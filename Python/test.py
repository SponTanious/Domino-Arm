from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from cv2 import *
import ctypes
import _ctypes
import pygame
import sys

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

class main(object):
    def __init__(self):
        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)

    def run(self):
        while 1:
            cframe = 0
            if self._kinect.has_new_color_frame():
                cframe = self._kinect.get_last_color_frame()

            red = zeros((1080,1920))
            green = zeros((1080,1920))
            blue = zeros((1080,1920))
            for i in range(0, 1799):
                for i in range(0,1919):
                    
                
            

            imshow("sansakjdsa", cframe)

            if not type(cframe) == int:
                print(cframe.size)


            
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()

DominoSorter = main();
DominoSorter.run();

#PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth).close()
