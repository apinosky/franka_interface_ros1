#!/usr/bin/env python

# other python imports
import numpy as np
import roslibpy # to interface with ros
import sys
import base64
import cv2
import time

import os
import sys
import ctypes
from ctypes import *
from winreg import *
import os
import matplotlib.pyplot as plt
os.environ['SDL_HINT_WINDOWS_ENABLE_MESSAGELOOP'] = "0"

DEBUG = False
IP_ADDRESS = "192.168.0.102" # wherever the websocket is running for roslibpy or this ip address for imagezqm
render = True
usgfw_path = 'C://Users//allis//Downloads//ArtUs_RFDataControl_for-MATLAB_Python_LabVIEW'
# downloaded from https://www.pcultrasound.com/research/research_files/

class roslibpy_interface(object): ## SLOW -- might need to replace
    """ Sets up the ros interface. """
    def __init__(self,address,debug=False): 
        self.debug = debug
        self.data = None
        
        if not debug:
            self.client = roslibpy.Ros(host=address, port=9090)
            self.client.run()

            self.img_publisher = roslibpy.Topic(self.client, '/ultrasound/image/compressed', 'sensor_msgs/CompressedImage')
            self.img_publisher.advertise()
            ## not sure if needed
            # self.depth_publisher = roslibpy.Topic(self.client, '/ultrasound/image/compressedDepth', 'sensor_msgs/CompressedImage')
            # self.depth_publisher.advertise()

            ## optional 
            # self.subscriber = roslibpy.Topic(self.client, '/us_state', 'std_msgs/String')
            # self.listener.subscribe(self.us_state_callback)
        print('ros setup complete')

    def is_connected(self): 
        if self.debug: 
            return True 
        else: 
            return self.client.is_connected()

    def subscribe(self,msg): 
        self.data = msg.data

    def publish(self,us_img):
        try:
            if not (self.debug) and self.client.is_connected:
                # base64EncodedStr = base64.b64encode(us_img.tobytes()).decode('ascii')
                base64EncodedStr = base64.b64encode(us_img[:,:,:3].tobytes()).decode('ascii')
                self.img_publisher.publish(roslibpy.Message({'format' : "jpeg", 'data' : base64EncodedStr}))
                # base64EncodedStr_depth = base64.b64encode(us_img[:,:,3].tobytes()).decode('ascii')
                # self.depth_publisher.publish(roslibpy.Message({'format' : "jpeg", 'data' : base64EncodedStr_depth}))
        except: 
            pass

    def __del__(self):
        try: 
            self.publisher.unadvertise()
            self.client.terminate()
        except: 
            pass
  
import imagezmq

class imagezmq_interface(object): ## SLOW -- might need to replace
    """ Sets up the ros interface. """
    def __init__(self,address,debug=False): 
        self.debug = debug
        self.data = None
        port = 9090
        
        if not debug:
            self.client = imagezmq.ImageSender(connect_to=f'tcp://{address}:{port}',REQ_REP = False)
        print('imagezmq setup complete')

    def publish(self,us_img):
        try:
            if not (self.debug):
                self.client.send_image('ultrasound',us_img)
        except: 
            pass

    def is_connected(self): 
        return True 

    def __del__(self):
        print('trying to close image sender')
        self.client.close()

  

class Ultrasound(object):

    def __init__(self): 
        self.AutoSC = 0
        self.bits = 8

        self.w = 512 # B image width
        self.h = 512 # B image heigth
        self.ros_w = 180 # downsampled size
        self.ros_h = 180
    
        self.usgfw2 = cdll.LoadLibrary(usgfw_path + '/usgfw2wrapper.dll') 

        if render: 
            fig,ax= plt.subplots(1,1,figsize=(12,12))
            self.us_img = ax.imshow(np.zeros((180,180,3)))
            ax.axes.xaxis.set_visible(False)
            ax.axes.yaxis.set_visible(False)
            fig.tight_layout()
            plt.ion()
            plt.show(block=False)
            fig.canvas.blit(fig.bbox)
            self.fig = fig    

    def setup(self): 
        # you can use usgfw_path\ArtUsRF2python.py to set the ultrasound parameters

        self.usgfw2.on_init()
        ERR = self.usgfw2.init_ultrasound_usgfw2()

        if (ERR == 2):
            print('Main Usgfw2 library object not created')
            sys.exit()
        
        ERR = self.usgfw2.find_connected_probe()

        if (ERR != 101):
            print('Probe not detected')
            print(ERR)
            sys.exit()

        ERR = self.usgfw2.data_view_function()

        if (ERR < 0):
            print('Main ultrasound scanning object for selected probe not created')
            sys.exit()
        
        ERR = self.usgfw2.mixer_control_function(0,0,self.w,self.h,0,0,0) 
        if (ERR < 0):
            print('B mixer control not returned')
            sys.exit()


        #---------------Scan converter callback for B mode image acquisition ----
        self.usgfw2.set_callback_scan_converter()

        #---------------self.frequency control--------------------------------------------
        self.usgfw2.frequency_control()  
        self.freq_pntr = c_int(0)
        self.freq_no_pntr = c_long(0)

        self.usgfw2.B_FrequencySetPrevNext(0, ctypes.pointer(self.freq_pntr), ctypes.pointer(self.freq_no_pntr))  

        self.frequency = str((self.freq_pntr.value)/1000000)


        #---------------Depth control--------------------------------------------
        self.usgfw2.depth_control()
        self.depth_pntr = ctypes.c_int(0)
        self.usgfw2.DepthSetPrevNext(0, ctypes.pointer(self.depth_pntr))

        #---------------Gain control--------------------------------------------
        self.usgfw2.gain_control()  
        self.gain_pntr = ctypes.c_int(0)
        self.set_gain(70)

        #---------------Power control--------------------------------------------
        self.usgfw2.power_control()  
        self.power_pntr = ctypes.c_int(0)
        self.set_power(20)

        # --------------- View area control--------------------------------------------
        self.usgfw2.view_area()
        self.view_area_pntr = ctypes.c_int(0)
        self.view_area_direction = 5
        self.usgfw2.B_view_areaSetPrevNext(self.view_area_direction, ctypes.pointer(self.view_area_pntr)) 

        # --------------- Focus control ----------------------------------------------
        self.usgfw2.focus_control()

        self.focal_depth_pntr = ctypes.c_int(0)
        self.focal_zones_count_pntr = ctypes.c_int(0)
        self.focal_zone_idx_pntr = ctypes.c_int(0)

        self.usgfw2.B_FocusSetPrevNext(0, ctypes.pointer(self.focal_depth_pntr), ctypes.pointer(self.focal_zones_count_pntr), ctypes.pointer(self.focal_zone_idx_pntr)) 

        # --------------- Dynamic range control ----------------------------------------------
        self.usgfw2.B_dynamic_range()  
        self.dynamic_range_pntr = ctypes.c_int(0)
        self.usgfw2.B_DynamicRangeSetPrevNext(0, ctypes.pointer(self.dynamic_range_pntr))

        # --------------- TGC control ----------------------------------------------
        self.usgfw2.TGC_control()  
        self.TGC_depth1_pntr = ctypes.c_int(0)
        self.TGC_depth2_pntr = ctypes.c_int(0)
        self.TGC_depth3_pntr = ctypes.c_int(0)
        self.TGC_depth4_pntr = ctypes.c_int(0)
        self.TGC_depth5_pntr = ctypes.c_int(0)
        self.adjust_TGC(50,60,60,60,60)

        # --------------- UsgQualProp_control ------------------------------------
        self.usgfw2.UsgQualProp_control()  

        # --------------- Scan type control ---------------------------------------
        self.usgfw2.scan_type_control()  
        self.usgfw2.turn_on_scan_type(0)  

        # --------------- Wide view angle control --------------------------------
        self.usgfw2.wide_view_angle()

        # ------------- Compound angle control -----------------------------------
        self.usgfw2.compound_angle()
        self.compound_angle_pntr = ctypes.c_int(0)
        self.usgfw2.CompoundAngleSetPrevNext(0, ctypes.pointer(self.compound_angle_pntr))
        
        # ------------- Compound frames control ----------------------------------
        self.usgfw2.compound_frames_number()  
        self.compound_frames_pntr = ctypes.c_int(0)
        self.usgfw2.CompoundFramesSetPrevNext(0, ctypes.pointer(self.compound_frames_pntr))

        # -------------  Subframe index ----------------------------------
        self.SubFrameIndex_pntr = ctypes.c_int(0)
        self.usgfw2.CompoundSubframeSetPrevNext(0, ctypes.pointer(self.SubFrameIndex_pntr))

        if (self.compound_frames_pntr.value<=self.SubFrameIndex_pntr.value):
            self.usgfw2.CompoundSubframeSetPrevNext(-(self.compound_frames_pntr.value-self.SubFrameIndex_pntr.value), ctypes.pointer(self.SubFrameIndex_pntr))


        #--------------- RF stream control ---------------
        depth_max = self.depth_pntr.value

        self.depth1_pntr = ctypes.c_long(0)
        self.depth2_pntr = ctypes.c_long(0)
        self.line1_pntr = ctypes.c_long(0)
        self.line2_pntr = ctypes.c_long(0)
        self.lines_number_pntr = ctypes.c_long(0)

        self.usgfw2.RF_window_get(ctypes.pointer(self.depth1_pntr),ctypes.pointer(self.depth2_pntr),ctypes.pointer(self.line1_pntr),ctypes.pointer(self.line2_pntr),ctypes.pointer(self.lines_number_pntr))


        if (self.depth2_pntr.value == 0):
            self.usgfw2.RF_data_controls()
            self.source_ID = 1
            self.usgfw2.RF_Data_source_ID(self.source_ID)
            self.usgfw2.RF_lines_get(ctypes.pointer(self.line1_pntr),ctypes.pointer(self.line2_pntr))
            L1 = self.line1_pntr.value
            L2 = self.line2_pntr.value
            self.usgfw2.RF_window_set(round(depth_max*0.25),round(depth_max*0.75),round(0.25*(self.line2_pntr.value - self.line1_pntr.value)),round(0.75*(self.line2_pntr.value - self.line1_pntr.value)))

        self.usgfw2.RF_window_get(ctypes.pointer(self.depth1_pntr),ctypes.pointer(self.depth2_pntr),ctypes.pointer(self.line1_pntr),ctypes.pointer(self.line2_pntr),ctypes.pointer(self.lines_number_pntr))

        self.RF_Row_to_show =int(np.fix((self.line2_pntr.value - self.line1_pntr.value)/2))

        self.usgfw2.set_callback_Sample_Grabber()
        self.sampling_period_ns_pntr = ctypes.c_int(0)
        self.usgfw2.get_sampling_period(ctypes.pointer(self.sampling_period_ns_pntr))

        self.number_of_samples_in_window_for_beam = int(np.fix(2*((self.depth2_pntr.value-self.depth1_pntr.value)/1000)/1540/((self.sampling_period_ns_pntr.value)*1e-9)))

        print('ultrasound setup complete')

    def on_closing(self):
        # error when probe not detected
        self.usgfw2.Close_and_release()
        try: 
            plt.close(self.fig)
        except: 
            pass

    #---------------Frequency--------------------------------------------
    def increase_frequency(self):
        self.usgfw2.B_FrequencySetPrevNext(-1, ctypes.pointer(self.freq_pntr), ctypes.pointer(self.freq_no_pntr))  
        self.frequency = str((self.freq_pntr.value)/1000000)


    def decrease_frequency(self):
        self.usgfw2.B_FrequencySetPrevNext(+1, ctypes.pointer(self.freq_pntr), ctypes.pointer(self.freq_no_pntr))  
        self.frequency = str((self.freq_pntr.value)/1000000)

        
    #---------------Depth--------------------------------------------
    def decrease_depth(self):
        self.usgfw2.DepthSetPrevNext(-1, ctypes.pointer(self.depth))
        # self.adjust_TGC()

    def increase_depth(self):
        self.usgfw2.DepthSetPrevNext(+1, ctypes.pointer(self.depth))
        # self.adjust_TGC()
    
    #---------------TGC--------------------------------------------
    def adjust_TGC(self,val0,val1,val2,val3,val4):
        self.usgfw2.adjust_TGC(0, val0, ctypes.pointer(self.TGC_depth1_pntr))

        self.usgfw2.adjust_TGC(1, val1, ctypes.pointer(self.TGC_depth2_pntr))

        self.usgfw2.adjust_TGC(2, val2, ctypes.pointer(self.TGC_depth3_pntr))

        self.usgfw2.adjust_TGC(3, val3, ctypes.pointer(self.TGC_depth4_pntr))

        self.usgfw2.adjust_TGC(4, val4, ctypes.pointer(self.TGC_depth5_pntr))

    def set_gain(self,val):
        # 0 to 90
        self.usgfw2.B_GainSetByIdx(val, ctypes.pointer(self.gain_pntr))  

    def set_power(self,val):
        # 0 to 20
        self.usgfw2.B_PowerSetByIdx(val,ctypes.pointer(self.power_pntr)) 

    #---------------View area-----------------------------------
    def decrease_view_area(self):
        if (self.view_area_pntr.value > 50):
            self.usgfw2.B_view_areaSetPrevNext(-1, ctypes.pointer(self.view_area_pntr)) 

    def increase_view_area(self):
        if (self.view_area_pntr.value < 100):
            self.usgfw2.B_view_areaSetPrevNext(+1, ctypes.pointer(self.view_area_pntr)) 

    #---------------Focus-----------------------------------
    def decrease_focal_depth(self):        
        if (self.focal_zone_idx_pntr.value > 0):
            self.usgfw2.B_FocusSetPrevNext(-1, ctypes.pointer(self.focal_depth_pntr), ctypes.pointer(self.focal_zones_count_pntr), ctypes.pointer(self.focal_zone_idx_pntr))

    def increase_focal_depth(self):        
        if (self.focal_zone_idx_pntr.value < self.focal_zones_count_pntr.value-1):
            self.usgfw2.B_FocusSetPrevNext(+1, ctypes.pointer(self.focal_depth_pntr), ctypes.pointer(self.focal_zones_count_pntr), ctypes.pointer(self.focal_zone_idx_pntr))


    #---------------Dynamic range------------------------------
    def decrease_dynamic_range(self):
        self.usgfw2.B_DynamicRangeSetPrevNext(-1, ctypes.pointer(self.dynamic_range_pntr))

    def increase_dynamic_range_(self):
        self.usgfw2.B_DynamicRangeSetPrevNext(+1, ctypes.pointer(self.dynamic_range_pntr))
        
        
    def freeze(self):
        self.usgfw2.Freeze_ultrasound_scanning()

    def run(self):
        self.usgfw2.Run_ultrasound_scanning()

    def get_data(self,rf=False):
        p_array = (ctypes.c_uint*self.w*self.h*4)()

        p1 = ctypes.c_int(0) 
        p2 = ctypes.c_float(0.0) 

        
        self.usgfw2.return_pixel_values2(ctypes.pointer(p_array), ctypes.pointer(p1), ctypes.pointer(p2))
        buffer_as_numpy_array = np.frombuffer(p_array, np.uint)  
        reshaped_array = np.reshape(buffer_as_numpy_array,(self.w, self.h, 4))
        reshaped_array = cv2.resize(reshaped_array.astype(np.uint16), (self.ros_w,self.ros_h)) # shrink before sending

        if render: 
            self.us_img.set_data(reshaped_array[:,:,:3])
            self.us_img.autoscale()
            self.fig.canvas.blit(self.fig.bbox)
            self.fig.canvas.flush_events()



        if rf: 
            # -----------------------------------------------------------------------------   
            P1 = ctypes.c_int(0)  
            P2 = ctypes.c_double(0)     
            P3 = ctypes.c_int(0)      
            P4 = ctypes.c_int(0)       

            if (self.source_ID == 4):
                p_array_RF = (ctypes.c_int16*self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value*2)() 
                self.usgfw2.return_RF_data(ctypes.pointer(p_array_RF),self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value*2,ctypes.pointer(P1),ctypes.pointer(P2),ctypes.pointer(P3),ctypes.pointer(P4))
            else:
                p_array_RF = (ctypes.c_int16*self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value)() 
                self.usgfw2.return_RF_data(ctypes.pointer(p_array_RF),self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value,ctypes.pointer(P1),ctypes.pointer(P2),ctypes.pointer(P3),ctypes.pointer(P4))

            self.RF_buffer_as_numpy_array = np.frombuffer(p_array_RF, np.int16)

            RF_data_to_show = self.RF_buffer_as_numpy_array[self.RF_Row_to_show*self.number_of_samples_in_window_for_beam+3:self.RF_Row_to_show*self.number_of_samples_in_window_for_beam + self.number_of_samples_in_window_for_beam]

            if (self.source_ID == 4):
                RF_data_to_show2 = self.RF_buffer_as_numpy_array[(self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value) + (self.RF_Row_to_show*self.number_of_samples_in_window_for_beam+3):(self.number_of_samples_in_window_for_beam*self.lines_number_pntr.value)+self.RF_Row_to_show*self.number_of_samples_in_window_for_beam + self.number_of_samples_in_window_for_beam]
            else: 
                RF_data_to_show2 = None

            return reshaped_array, RF_data_to_show, RF_data_to_show2
        else:
            return reshaped_array
    

          

if __name__ == '__main__' :
    
    # ros = roslibpy_interface(IP_ADDRESS,DEBUG)
    ros = imagezmq_interface(IP_ADDRESS,DEBUG)

    us = Ultrasound()
    
    # added to save when exiting
    from signal import signal, SIGINT
    from sys import exit
    
    def handler(signal_received, frame):
        # Handle any cleanup here
        print('SIGINT or CTRL-C detected.')
        us.on_closing()
        print('Exiting gracefully')
        exit(0)

    # Tell Python to run the handler() function when SIGINT is recieved
    signal(SIGINT, handler)

    us.setup() 
    us.run()
    print('close matplotlib window to quit')
    ## to do : set up subscriber / publisher
    error = False
    while ros.is_connected and not(error):
        try:
            # get data from ultrasound 
            data = us.get_data()
            # publish data
            ros.publish(data)
        except Exception as e: 
            print(e) 
            error = True
        # sleep?

    ## clean up 
    us.on_closing()
    del ros