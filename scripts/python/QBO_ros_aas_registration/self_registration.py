#!/usr/bin/env python
# coding=utf-8
import requests
import json

# register this device on ROBxTASK cloud platform with skill implementation documentation
registration_endpoint = 'https://api.jsonstorage.net/v1/json/f8d89bf8-6826-4434-9ca1-edd60405bf6d' # TODO change to real endpoint

#---------------------------------------------------------------------------------------------
# loadRegistrationFile
#---------------------------------------------------------------------------------------------
def loadRegistrationFile():

    with open('/opt/QBO/catkin_ws/src/rxt_skills_qbo/scripts/python/QBO_ros_aas_registration/QBO_registration_file.json', 'r') as dt_file:
        dt_data = json.load(dt_file) 
        # TODO
    
    return dt_data


# -------------------------------------------------------------------------------------------
# uploadAAS (upload full settings with all entries)
# -------------------------------------------------------------------------------------------
def uploadAAS(aas):
    
    try:     
        #r = requests.post('https://power2dm.salzburgresearch.at/robogen/DataBase/UploadJSON_MySettings', timeout=5, verify=False, json=aas)
        r = requests.put(registration_endpoint, timeout=5, verify=False, json=aas)
        headers = {'Content-type': 'application/json'}      
            
        if r.ok:
            print("Description uploaded succesfullly")
        else:
            print("--------------------------")
            print("Error in server response: " + str(r.status_code))
            print("--------------------------")
                        
    except requests.exceptions.RequestException as e:
        print e
        
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------	 
#if __name__ == '__main__':   
#    registration_file = loadRegistrationFile()
#    uploadAAS(registration_file)
    
    
    
    