#!/usr/bin/env python
# coding=utf-8
from requests.auth import HTTPBasicAuth
import requests
import json

# register this device on ROBxTASK cloud platform with skill implementation documentation
# INFO: needs Basic Authentication: USER: "devr" + PW: "DevReg!robXtask"
registration_endpoint = 'https://robxtask.salzburgresearch.at/robxtask/registration-service/v3/api-docs?group=all/device' 

#---------------------------------------------------------------------------------------------
# loadRegistrationFile
#---------------------------------------------------------------------------------------------
def loadRegistrationFile():

    with open('/opt/QBO/catkin_ws/src/rxt_skills_qbo/scripts/python/QBO_ros_aas_registration/QBO_registration_file.json', 'r') as json_file:
        json_data = json.load(json_file) 
    
    return json_data


# -------------------------------------------------------------------------------------------
# uploadAAS (upload full settings with all entries)
# -------------------------------------------------------------------------------------------
def uploadAAS(aas):
    
    try:
        headers = {'Content-type': 'application/json'}
        auth = HTTPBasicAuth('register_device_key', 'asDycMEj82yY9Jz1hySo')  
        r_get = requests.get(registration_endpoint + '/b8:27:eb:24:1f:b2', timeout=5, json=aas, headers=headers, auth=auth)

        if r_get.status_code == 200: # 200 = valid response with body
            r_add = requests.put(registration_endpoint, timeout=5, json=aas, headers=headers, auth=auth) 
        else:
            r_add = requests.post(registration_endpoint, timeout=5, json=aas, headers=headers, auth=auth) 
            
        if r_get.ok and r_add.ok:
            print("------------------------------------")
            print("Result of self registration:")
            print("Entry already existed and was updated")
            print("------------------------------------")
        elif r_add.ok:
            print("------------------------------------")
            print("Result of self registration:")
            print("Description uploaded succesfully")
            print("------------------------------------")
        else:
            print("------------------------------------")
            print("Result of self registration:")
            print("Error in server response: " + str(r_add.status_code))
            print("------------------------------------")
                        
    except requests.exceptions.RequestException as e:
        print (e)
    
    

