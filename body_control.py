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

    #driver2.find_elements_by_xpath('//inner/input').send_keys(100)
    #driver2.find_elements_by_xpath("//div[5]/inner").send_keys()
    
    
    
    print (my_thumb)
    x=0
    tmp_ind=0
    tmp_ind2=0
    while 1:
        
        if driver.find_element_by_id("leftarm").text == "Right In":
            if x != 1:
                x=1
                midiout.send_message([0x90, 81, 100])
                #driver2.find_element_by_xpath("//inner/div").click()
                print ("Right In")
                #time.sleep(0.1)
                
                #time.sleep(5)
        if driver.find_element_by_id("leftarm").text == "Right Out":
            if x != 2:
                x=2
                midiout.send_message([0x90, 80, 100])
                #driver2.find_element_by_xpath("//inner/div[2]").click()
                print ("Right Out")
                #time.sleep(0.1)
                #time.sleep(5)
        if int(driver.find_element_by_id("wrist").text) < 165 and int(driver.find_element_by_id("wrist").text) > 0:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 10:

                #(x, in_min, in_max, out_min, out_max):
                NewValue = remap(int(driver.find_element_by_id("wrist").text), 0, 165, 62, 127)

                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                #driver2.find_elements_by_xpath("//div[11]/inner/canvas")[0].click()
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.CONTROL + "a")
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(int(NewValue))
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.ENTER)
                midiout.send_message([176, 20, int(NewValue)])
                print (NewValue)
                time.sleep(0.1)
        if int(driver.find_element_by_id("wrist").text) < 0 and int(driver.find_element_by_id("wrist").text) > -90:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 10:
                NewValue = remap(abs(int(driver.find_element_by_id("wrist").text)), 0, 90, 31, 61)
                
                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                #driver2.find_elements_by_xpath("//div[11]/inner/canvas")[0].click()
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.CONTROL + "a")
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(int(NewValue))
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.ENTER)
                midiout.send_message([176, 20, int(NewValue)])
                #print (tmp_ind2)
                time.sleep(0.1)
                
        if int(driver.find_element_by_id("wrist").text) < 270 and int(driver.find_element_by_id("wrist").text) > 179:
            if abs(tmp_ind2 - int(driver.find_element_by_id("wrist").text)) > 10:
                
                NewValue = remap(int(driver.find_element_by_id("wrist").text), 180, 270, 0, 30)
                
                tmp_ind2 = int(driver.find_element_by_id("wrist").text)
                #driver2.find_elements_by_xpath("//div[11]/inner/canvas")[0].click()
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.CONTROL + "a")
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(int(NewValue))
                #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.ENTER)
                #print (tmp_ind2)
                midiout.send_message([176, 20, int(NewValue)])
                time.sleep(0.2)
                
                #time.sleep(5)
            #if x != 3:
                #x=3
                #0-160 --> 40-210 ==> -50
        #driver2.find_elements_by_xpath("//div[10]/inner/canvas")[0].click()
        #driver2.find_elements_by_xpath("//inner/input").clear()
        #time.sleep(0.2)
        if abs(tmp_ind - int(driver.find_element_by_id("coord").text)) > 5:
            
            NewValue = remap(int(driver.find_element_by_id("coord").text)+180, 0, 360, 0, 127)
            
            tmp_ind = int(driver.find_element_by_id("coord").text)
            #driver2.find_elements_by_xpath("//div[10]/inner/canvas")[0].click()
            #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.CONTROL + "a")
            #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(int(NewValue))
            #driver2.find_elements_by_xpath("//inner/input")[0].send_keys(Keys.ENTER)
            #print (int(NewValue))
            midiout.send_message([176, 21, int(NewValue)])
            time.sleep(0.1)
        
        
        
        #time.sleep(1)
        
if __name__ == "__main__":
    main()



