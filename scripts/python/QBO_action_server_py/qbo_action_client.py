#! /usr/bin/env python

import rospy
import time

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the qbo actions, including the
# goal message and the result message of the task modules "WaitForUserInput" and "VoiceOutput"
import rxt_skills_qbo.msg


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# client request implementations of QBO action server functions
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def qbo_request_VoiceOutput(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('VoiceOutput', rxt_skills_qbo.msg.VoiceOutputAction) # Creates SimpleActionClient with VoiceOutputAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.VoiceOutputGoal(outputMessage=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (VoiceOutputResult) of executing the action


def qbo_request_WaitForUserInput(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForUserInput', rxt_skills_qbo.msg.WaitForUserInputAction) # Creates SimpleActionClient with WaitForUserInputAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.WaitForUserInputGoal(inputContent=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action


def qbo_request_MoveToLocation(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('MoveToLocation', rxt_skills_qbo.msg.MoveToLocationAction) # Creates SimpleActionClient with MoveToLocationAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.MoveToLocationGoal(location=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (MoveToLocationResult) of executing the action
    
    
def qbo_request_WaitForExternalEvent(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForExternalEvent', rxt_skills_qbo.msg.WaitForExternalEventAction) # Creates SimpleActionClient WaitForExternalEventAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.WaitForExternalEventGoal(inputText=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForExternalEventResult) of executing the action
    
    
def qbo_request_GraphicalUserInteraction(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GraphicalUserInteraction', rxt_skills_qbo.msg.GraphicalUserInteractionAction) # Creates SimpleActionClient with GraphicalUserInteractionAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.GraphicalUserInteractionGoal(outputMessage=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GraphicalUserInteractionResult) of executing the action
    
    
def qbo_request_GetData(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GetData', rxt_skills_qbo.msg.GetDataAction) # Creates SimpleActionClient with GetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.GetDataGoal(inputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GetDataResult) of executing the action


def qbo_request_SetData(msgBytes):
    
    rospy.init_node('qbo_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('SetData', rxt_skills_qbo.msg.SetDataAction) # Creates SimpleActionClient with SetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_qbo.msg.SetDataGoal(outputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (SetDataResult) of executing the action
	
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:	
        
        # request VoiceOutput
        result = qbo_request_VoiceOutput(b'Hallo ich bin ein sozialer Roboter')
        if result:
            print ('----------------------------------')
            print("Action was: VoiceOutput")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request WaitForUserInput
        result = qbo_request_WaitForUserInput(b'void')
        if result:
            print ('----------------------------------')
            print("Action was: WaitForUserInput")
            print("Result was:", ', '.join([str(n) for n in result.returnMessage.decode("utf-8")]))
            print ('----------------------------------')
        
        # request MoveToLocation
        result = qbo_request_MoveToLocation(b'right')
        if result:
            print ('----------------------------------')
            print("Action was: MoveToLocation")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')

	# request GraphicalUserInteraction
        result = qbo_request_GraphicalUserInteraction(b'happy')
        if result:
            print ('----------------------------------')
            print("Action was: GraphicalUserInteraction")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
        
        # request WaitForExternalEvent
        result = qbo_request_WaitForExternalEvent(b'fear')
        if result:
            print ('----------------------------------')
            print("Action was: WaitForExternalEvent")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')      
        
        # request GetData
        result = qbo_request_GetData(b'robotName')
        if result:
            print ('----------------------------------')
            print("Action was: GetData")
            print("Result was:", ', '.join([str(n) for n in result.data.decode("utf-8")]))
            print ('----------------------------------')
        
        # request SetData
        result = qbo_request_SetData(b'Mario')
        if result:
            print ('----------------------------------')
            print("Action was: SetData")
            print("Result was: " + str(result.isOK))
            print ('----------------------------------')
          
        # shutdown node
        #print ('----------------------------------')
        #print ('All requests done: Now trying to shutdown everything...')
        #print ('----------------------------------')
        #rospy.signal_shutdown("Finished with success!")
        #rospy.spin()
             
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
