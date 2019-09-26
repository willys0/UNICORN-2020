#!/usr/bin/env python

'''
 Copyright (c) 2016, Juan Jimeno

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
'''

import rospy
# Added by Marielle ---------------------------------------------------------------
from geometry_msgs.msg import PoseWithCovarianceStamped
# End -----------------------------------------------------------------------------
import tf
import localization as lx
import serial

# Added by Marielle ---------------------------------------------------------------
# Globals
 
#create publisher for position
#topic is "uwbpos" and node name is "uwb_pos_pub"
#the publisher named "publisher" will write to the topic with a PoseWithCovarianceStamped msg
publisher = rospy.Publisher('uwbpos', PoseWithCovarianceStamped, queue_size=50)
#rospy.init_node('uwb_pos_pub', anonymous=True)
# End -----------------------------------------------------------------------------

def get_transform(id):
    try:
        (trans,rot) = listener.lookupTransform('/map', id, rospy.Time(0))
        return trans
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def get_tag_location(anchors, ranges, transforms):
    P = lx.Project(mode="3D",solver="LSE")

    #define anchor locations
    for i in range(REQ_ANCHOR):
        P.add_anchor(anchors[i], transforms[i])
    t, label = P.add_target()

    #define anchor ranges
    for i in range(REQ_ANCHOR):
        t.add_measure(anchors[i], ranges[i])

    P.solve()
    B = t.loc
    return {'x':B.x, 'y':B.y, 'z':B.z}

def is_listed(anchors, id):
    for anchor in anchors:
        if anchor == id:
            return True
        else:
            pass

def get_serial_data():
    start = ser.read()
    # return ser.readline().strip('$\r\n').split(',')
    # expected data from the serial port is: $<anchor_id>,<range in cm>,\r\n
    if start == '$':
        parsed_data = ser.readline().strip('\r\n').split(',')
        # anchor id is stored in index 0 - parsed_data[0]
        # range is stored in index 1 - parsed_data[1]
        return parsed_data
    else:
        return None

# Added by Marielle ---------------------------------------------------------------
def uwb_pos_pub(x, y):
    
    #Publish pos as geometry_msgs/PoseWithCovarianceStamped for EKF/UKF
    uwb_tag_pos = PoseWithCovarianceStamped()
    
    #header information
    uwb_tag_pos.header.frame_id = "map"
    uwb_tag_pos.header.stamp = rospy.Time.now()
 
    # pos x and y, no z
    uwb_tag_pos.pose.pose.position.x = x
    uwb_tag_pos.pose.pose.position.y = y
    uwb_tag_pos.pose.pose.position.z = 0.0

    #no orientation
    uwb_tag_pos.pose.pose.orientation.x = 0.0
    uwb_tag_pos.pose.pose.orientation.y = 0.0
    uwb_tag_pos.pose.pose.orientation.z = 0.0
    uwb_tag_pos.pose.pose.orientation.w = 0.0
    
    #high value means ignore the variable(s) the sensor does not produce
    #in our case ignore z, roll, pitch and yaw
    #uwb_tag_pos.pose.covariance[0:5] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #uwb_tag_pos.pose.covariance[6:11] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    #uwb_tag_pos.pose.covariance[12:17] = [0.0, 0.0, 99999, 0.0, 0.0, 0.0]
    #uwb_tag_pos.pose.covariance[18:23] = [0.0, 0.0, 0.0, 99999, 0.0, 0.0]
    #uwb_tag_pos.pose.covariance[24:29] = [0.0, 0.0, 0.0, 0.0, 99999, 0.0]
    #uwb_tag_pos.pose.covariance[30:35] = [0.0, 0.0, 0.0, 0.0, 0.0, 99999]
    uwb_tag_pos.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999]   
    
    #publishes to our topic named uwbpos
    rospy.loginfo(uwb_tag_pos)
    publisher.publish(uwb_tag_pos)
# End -----------------------------------------------------------------------------

if __name__ == '__main__':

    rospy.init_node('ros_dwm1000')
    listener = tf.TransformListener()
    start_time = rospy.get_time()

    #create rosparameters
    MIN_RANGE = rospy.get_param('/ros_dwm1000/min_range', 0.5)
    MAX_RANGE = rospy.get_param('/ros_dwm1000/max_range', 12.0)
    REQ_ANCHOR = rospy.get_param('/ros_dwm1000/req_anchor', 3)
    FRAME_ID = rospy.get_param('/ros_dwm1000/frame_id', 'uwb_tag')

    SERIAL_PORT = rospy.get_param('/ros_dwm1000/serial_port', '/dev/charlieArduinoTagUWB')


    #rosparam logs just to make sure parameters kicked in
    rospy.loginfo("%s is %s", rospy.resolve_name('/ros_dwm1000/min_range'), MIN_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/ros_dwm1000/max_range'), MAX_RANGE)
    rospy.loginfo("%s is %s", rospy.resolve_name('/ros_dwm1000/req_anchor'), REQ_ANCHOR)
    rospy.loginfo("%s is %s", rospy.resolve_name('/ros_dwm1000/frame_id'), FRAME_ID)
    rospy.loginfo("%s is %s", rospy.resolve_name('/ros_dwm1000/serial_port'), SERIAL_PORT)

    ser = serial.Serial(SERIAL_PORT, 115200)
    ser.timeout = None
    rospy.loginfo("Connected to %s", ser.portstr)

    #lists to store anchors found
    ranges = []
    anchors = []
    transforms = []
    anchors_found = 0
    
    
    while not rospy.is_shutdown():
        #get the stream of data from the tag through the serial port
        parsed_data = get_serial_data()

        # print parsed_data
        if None != parsed_data:
            #check if the current range is within specified distance
            if MIN_RANGE < float(parsed_data[1]) < MAX_RANGE:
                #append respective arrays of the anchor found
                #list of anchor IDs found
                anchors.append(parsed_data[0])
		rospy.loginfo("Anchor found with ID %s", anchors)
                #list of distance between tag and anchors found
                ranges.append(parsed_data[1])
		rospy.loginfo("Distance from anchor with ID %s is %s", anchors, ranges)
                #list of static TFs of the anchors found.
                transforms.append(get_transform(parsed_data[0]))
                anchors_found += 1

        #perform trilateration once enough anchors have been found
        if anchors_found == REQ_ANCHOR:
            #do trilateration
            pos = get_tag_location(anchors,ranges,transforms)

            #broadcast the transform
            br = tf.TransformBroadcaster()
            br.sendTransform((pos['x'], pos['y'], pos['z']),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            FRAME_ID,
                            "map")

            #TODO: Publish pos as geometry_msgs/PoseWithCovarianceStamped for EKF and only broadcast TF as an option.

# Added by Marielle ---------------------------------------------------------------
            uwb_pos_pub(pos['x'], pos['y'])
# End -----------------------------------------------------------------------------

            # clear lists once trilateration is done for the next cycle
            anchors_found = 0
            ranges = []
            transforms = []
            anchors = []
