# -*- coding: utf-8 -*-
"""
Created on Sun May 23 13:16:30 2021

@author: moroz
"""

from selenium import webdriver
import time
import os

def main():

    driver = webdriver.Firefox()
    driver2 = webdriver.Firefox()
    
    driver.get("file:///D:/unreal/emotion/hand_control.html")
    driver2.get("http://localhost:8080")
    my_thumb = ""
    
    while my_thumb == "":
        my_thumb = driver.find_element_by_id("thumb").text
        time.sleep(1)
    
    while my_thumb == "Looking for a Wrist...":
        my_thumb = driver.find_element_by_id("thumb").text
        time.sleep(1)
        
        
    print (my_thumb)
    x=0
    while 1:
        
        if driver.find_element_by_id("thumb").text == "Left":
            if x == 0:
                x=1
                driver2.find_element_by_xpath("//inner/div").click()
                print ("left")
                #time.sleep(5)
        if driver.find_element_by_id("thumb").text == "Right":
            if x == 1:
                x=0
                driver2.find_element_by_xpath("//inner/div").click()
                print ("right")
                #time.sleep(5)
            
        time.sleep(1)

if __name__ == "__main__":
    main()