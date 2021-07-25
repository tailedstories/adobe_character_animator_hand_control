from selenium import webdriver
import time


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
        
        if driver.find_element_by_id("thumb").text == "One":
            if x != 1:
                x=1
                driver2.find_element_by_xpath("//inner/div").click()
                print ("One")
                #time.sleep(5)
        if driver.find_element_by_id("thumb").text == "Two":
            if x != 2:
                x=2
                driver2.find_element_by_xpath("//inner/div[2]").click()
                print ("Two")
                #time.sleep(5)
        if driver.find_element_by_id("thumb").text == "Three":
            if x != 3:
                x=3
                driver2.find_element_by_xpath("//div[3]").click()
                print ("Three")
                #time.sleep(5)
        if driver.find_element_by_id("thumb").text == "Four":
            if x != 4:
                x=4
                driver2.find_element_by_xpath("//div[4]").click()
                print ("Four")
                #time.sleep(5)
        if driver.find_element_by_id("thumb").text == "Thumb":
            if x != 0:
                x=0
                driver2.find_element_by_xpath("//div[5]").click()
                print ("Thumb")
                #time.sleep(5)
        
        time.sleep(1)
        
if __name__ == "__main__":
    main()