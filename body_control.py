# -*- coding: utf-8 -*-
"""
Created on Tue May 25 20:05:07 2021

@author: moroz
"""



from selenium import webdriver
from selenium.webdriver.common.keys import Keys
import time

import rtmidi


def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main():

    driver = webdriver.Firefox()
    
    midiout = rtmidi.MidiOut()
    available_ports = midiout.get_ports()
    midiout.open_port(1)
        
    driver.get("file:///D:/unreal/body/body_tacker.html")
    
    my_thumb = ""
    
    while my_thumb == "":
        my_thumb = driver.find_element_by_id("leftarm").text
        time.sleep(1)
    
    while my_thumb == "Looking for a Wrist...":
         my_thumb = driver.find_element_by_id("leftarm").text
         time.sleep(1)

       
    
    print (my_thumb)
    x=0
    tmp_ind=0
    tmp_ind2=0
    while 1:
        
        if driver.find_element_by_id("leftarm").text == "Right In":
            if x != 1:
                x=1
                midiout.send_message([0x90, 80, 100])
                print ("Right In")
                #time.sleep(0.1)
        if driver.find_element_by_id("leftarm").text == "Right Out":
            if x != 2:
                x=2
                midiout.send_message([0x90, 81, 100])
                print ("Right Out")                
        if int(driver.find_element_by_id("wrist").text) < 165 and int(driver.find_element_by_id("wrist").text) > 0:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 5:
                NewValue = remap(int(driver.find_element_by_id("wrist").text), 0, 165, 62, 127)
                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                midiout.send_message([176, 21, int(NewValue)])    
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
        if int(driver.find_element_by_id("wrist").text) < 0 and int(driver.find_element_by_id("wrist").text) > -90:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 5:
                NewValue = remap(int(driver.find_element_by_id("wrist").text)+90, 0, 90, 31, 61)
                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                midiout.send_message([176, 21, int(NewValue)])
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
        if int(driver.find_element_by_id("wrist").text) < 270 and int(driver.find_element_by_id("wrist").text) > 179:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 5:                
                NewValue = abs(remap(int(driver.find_element_by_id("wrist").text), 180, 270, 0, 30))
                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                midiout.send_message([176, 21, int(NewValue)])
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
        if abs(tmp_ind - int(driver.find_element_by_id("coord").text)) > 5:
            NewValue = remap(int(driver.find_element_by_id("coord").text)+180, 0, 360, 0, 127)
            tmp_ind  = int(driver.find_element_by_id("coord").text)
            midiout.send_message([176, 20, int(NewValue)])
            #print ("Shoulder - ", int(NewValue))
            #time.sleep(0.1)
        
        
    del midiout
        #time.sleep(1)
        
if __name__ == "__main__":
    main()



