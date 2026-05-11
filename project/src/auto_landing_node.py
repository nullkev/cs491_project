#!/usr/bin/env python
import rospy
import mavros
from enum import Enum
import numpy as np
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from apriltag_ros.msg import *

#global vars
pitch_gain = 60
roll_gain = 60
yaw_gain = 100
throttle_gain = 10
pitch_max = 100
roll_max = 100
yaw_max = 100
throttle_max = 100
landing_height = 10
search_counter = 5

def quat_to_yaw(x, y, z, w):
    sin_y = 2*(w*z + x*y)
    cos_y = 1-2*(y*y + z*z)
    return np.arctan2(sin_y, cos_y)

class AutoFlight:
    def __init__(self):
        rospy.init_node("auto_flight")
        self.state = "INIT"
        self.april_counter = 0
        self.april_pose = None
        
        rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, self.tag_cb)
        self.rc_pub = rospy.Publisher("/minihawk_SIM/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.arm = rospy.ServiceProxy("/minihawk_SIM/mavros/cmd/arming", CommandBool)
        self.set_mode = rospy.ServiceProxy("/minihawk_SIM/mavros/set_mode", SetMode)
        self.land_srv = rospy.ServiceProxy("/minihawk_SIM/mavros/cmd/land", CommandTOL)
        self.rate = rospy.Rate(20)

    def tag_cb(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            self.april_pose = detection.pose.pose.pose
            self.april_counter += 1
        if len(msg.detections) == 0 and self.april_counter > 0:
            self.april_counter -= 1
            self.april_pose = None

    def run(self):
        while not rospy.is_shutdown():
            if self.state == "INIT":
                rospy.loginfo("INIT -> AUTO")
                self.state = "AUTO"
            elif self.state == "AUTO":
                self.set_mode(custom_mode="AUTO")
                rospy.loginfo("ARMING")
                self.arm(True)
                rospy.loginfo("AUTO -> SEARCH")
                self.state = "SEARCH"
            elif self.state == "SEARCH":
                if self.april_counter >= search_counter:
                    self.set_mode(custom_mode="QLOITER")
                    rospy.loginfo("SEARCH -> ALIGN") 
                    self.state = "ALIGN"
            elif self.state == "ALIGN":
                pitch_in = 1500
                roll_in = 1500
                yaw_in = 1500
                z_in = 1500
                pitch_err = 0
                roll_err = 0
                yaw_err = 0
                z_err = 0
                #find errors
                if self.april_pose:
                    pos = self.april_pose.position
                    orient = self.april_pose.orientation
                    #pitch error
                    pitch_err = -pos.y
                    if abs(pitch_err) < 0.3:
                        pitch_err = 0
                    #roll error
                    roll_err = pos.x
                    if abs(roll_err) < 0.3:
                        roll_err = 0
                    #yaw error
                    yaw_err = -quat_to_yaw(orient.x, orient.y, orient.z, orient.w)
                    if abs(yaw_err) < 0.3:
                        yaw_err = 0
                    #height error
                    z_err = landing_height - pos.z
                    if abs(z_err) < 1:
                        z_err = 0
                rospy.loginfo("Pitch error: %.3f, Roll error: %.3f, Yaw error: %.3f, Height error: %.3f", pitch_err, roll_err, yaw_err, z_err)
                #adjust input
                pitch_in = pitch_err*pitch_gain
                roll_in = roll_err*roll_gain
                yaw_in = yaw_err*yaw_gain
                z_in = z_err*throttle_gain
                #pitch input adjustment
                if pitch_in > pitch_max:
                    pitch_in = pitch_max
                if pitch_in < -pitch_max:
                    pitch_in = -pitch_max
                pitch_in += 1500
                #roll input adjustment
                if roll_in > roll_max:
                    roll_in = roll_max
                if roll_in < -roll_max:
                    roll_in = -roll_max
                roll_in += 1500
                #yaw input adjustment
                if yaw_in > yaw_max:
                    yaw_in = yaw_max
                if yaw_in < -yaw_max:
                    yaw_in = -yaw_max
                yaw_in += 1500
                #z input adjustment
                if z_in > throttle_max:
                    z_in = throttle_max
                if z_in < -throttle_max:
                    z_in = -throttle_max
                z_in += 1500
                #inputs
                msg = OverrideRCIn()
                msg.channels = [1500] * 18
                msg.channels[0] = roll_in
                msg.channels[1] = pitch_in
                msg.channels[2] = z_in
                msg.channels[3] = yaw_in
                msg.channels[4] = 1800
                msg.channels[5] = 1000
                msg.channels[6] = 1000
                msg.channels[7] = 1800
                self.rc_pub.publish(msg)
                #landing
                if(self.april_pose and abs(pitch_err)<0.5 and abs(roll_err)<0.5 and abs(yaw_err)<0.2 and abs(z_err)<5) or self.april_counter == 0:
                    self.set_mode(custom_mode="QLAND")
                    self.state = "LAND"
                    rospy.loginfo("LANDING")
            self.rate.sleep()

if __name__ == "__main__":
    flight = AutoFlight()
    flight.run()
