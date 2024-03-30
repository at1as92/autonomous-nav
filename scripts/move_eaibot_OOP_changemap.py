#!/usr/bin/env python

'''
SDRE 201 Autonomous Navigation
move_eaibot_OOP_changemap.py
4 Jan 23
Script by Gary Chew

Script serves the saved waypoints to the robot one at a time, prior to completion of current load waypoint
List of saved waypoints represents a route that the robot is supposed to take as part of navigation process
Script has map changing function which loads new map at certain user-defined waypoints
'''

#Import libraries
import rospy
import roslaunch
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import json
import numpy as np

#Create CurrentPose class
#Subscribes to and outputs the robot's AMCL pose
class CurrentPose:

    #/amcl_pose publishes at 1Hz; to monitor if acceptable
    #If not need to extrapolate from /odom or /cmd_vel (8Hz) to give higher frequency updates

    def __init__(self):
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        self.poseSub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.retrieveCurrentPose)
        self.posX = 0
        self.posY = 0

    def retrieveCurrentPose(self, msg):

        self.posX = msg.pose.pose.position.x
        self.posY = msg.pose.pose.position.y

#Create MoveThruWaypoint class
#Monitors and serves the waypoints by publishing waypoint to the /move_base_simple/goal topic
#Monitors and change maps at user specified waypoints
class MoveThruWaypoint:

    def __init__(self):

        self.changeWaypointThreshold = 0.75 #Threshold value to feed new waypoint
        self.moveBasePub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 2)
        #self.filename = "blk7_lvl1_waypoint_231228_1129.json" #List of saved waypoints
        self.filename = "/home/eaibot/dashgo_ws/src/dashgo/dashgo_tools/scripts/blk7_lvl1_waypoint_231228_1129.json" #List of saved waypoints
        self.rate = rospy.Rate(2) #2Hz should be sufficient when AMCL is updating at 1Hz
        self.isComplete = False #Flag when all waypoints are completed

        self.toChangeMap = False #Flag for changing map at when next waypoint is loaded
        self.currentMap = "map1"

    #Read waypoint data from JSON file
    def readWaypoint(self):

        with open(self.filename,'r') as f:
            self.savedWaypoints = json.load(f)
            print("{} saved waypoints loaded".format(len(self.savedWaypoints)))
            f.close()

    #Start navigation to the waypoints
    def startNavigation(self,currentPose):

        rospy.loginfo('Start moving through the waypoints')

        while (not rospy.is_shutdown()) & self.isComplete == False:

            for waypoint in self.savedWaypoints:

                self.changeMap(waypoint)
                
                print('{} loaded.'.format(waypoint['waypointName']))
                #self.isMapChange = False
                distRemain = self.changeWaypointThreshold + 1
                
                #Monitor the distance remaining to the loaded waypoint (2D euclidean distance)
                #Load new waypoint if distance remaining is less than threshold for robot smooth movement
                while distRemain >= self.changeWaypointThreshold:

                    distRemain = np.sqrt((currentPose.posX- waypoint['pose']['posX'])**2 + (currentPose.posY - waypoint['pose']['posY'])**2)
                    print('Remaining Distance: {:.2f}'.format(distRemain)) 

                    msg = PoseStamped()
                    msg.header.frame_id = "map"
                    msg.pose.position.x = waypoint['pose']['posX']
                    msg.pose.position.y = waypoint['pose']['posY']
                    msg.pose.position.z = waypoint['pose']['posZ']
                    msg.pose.orientation.x = waypoint['pose']['orientX']
                    msg.pose.orientation.y = waypoint['pose']['orientY']
                    msg.pose.orientation.z = waypoint['pose']['orientZ']
                    msg.pose.orientation.w = waypoint['pose']['orientW']

                    self.moveBasePub.publish(msg)
                    self.rate.sleep()

            self.isComplete = True #Set flag to true when all waypoints complete
            rospy.loginfo("All waypoints completed.")

    #Function to initiate map changing when robot arrives at the current loaded waypoint
    def changeMap(self, waypoint):

        #Initiate change map process through ROS launch files if change map flag is set to true
        if self.toChangeMap == True:
            if self.currentMap == 'map1':
                roslaunch_file = '/home/eaibot/dashgo_ws/src/dashgo/dashgo_nav/launch/map2.launch' 
                self.currentMap = 'map2'

            elif self.currentMap == 'map2':
                roslaunch_file = '/home/eaibot/dashgo_ws/src/dashgo/dashgo_nav/launch/map1.launch'
                self.currentMap = 'map1'
                    
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            parent = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file])
            parent.start()
            rospy.sleep(0.15)
            print("Changed current map to: {}".format(self.currentMap))
            
            self.toChangeMap = False #Set to false once change map process is complete

        #Check if need to change map when arrived at current waypoint
        if waypoint['changeMap'] == 1:
            self.toChangeMap = True


if __name__ == "__main__":

    rospy.init_node('move_base_publisher')
    
    current_pose = CurrentPose()

    eaibot = MoveThruWaypoint()
    eaibot.readWaypoint()
    eaibot.startNavigation(current_pose)
