#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        #rospy.loginfo("Waypoint updater  ..........")

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.loop()
        #rospy.spin()
    
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                #closest_waypoint_idx = self.get_closest_waypoint_idx()
                #self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        #rospy.loginfo("value of x {} value of y {}".format(x,y))
        # check if (x,y) is ahea or behind 
        # the quesy returns the postion and the index
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        #dot product b/n the vectors to check if the closes point is ahead or behind us
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        val = np.dot(cl_vect-prev_vect,pos_vect-cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1)%len(self.waypoints_2d)
        return closest_idx

#    def publish_waypoints(self, closest_idx):
#        lane = Lane()
#        lane.header = self.base_waypoints.header
#        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+ LOOKAHEAD_WPS]
#        self.final_waypoints_pub.publish(lane)

    def publish_waypoints(self):
        #rospy.loginfo("publishing waypoints ..........")
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        #if self.stopline_wp_idx > 0:
        #    rospy.loginfo("stopline_wp_idx {} farthest_idx {} closest_idx {}".format(self.stopline_wp_idx,farthest_idx,closest_idx))
        #if np.abs(self.pose.pose.position.x - 1131) < 50 and self.stopline_wp_idx > 0:  
        #   self.stopline_wp_idx = farthest_idx
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            # no need to mofigy the waypoint
            lane.waypoints = base_waypoints
        else:
            # modify the way points
            lane.waypoints = self.deceleration_waypoints(base_waypoints, closest_idx)
        
        return lane
    
    def deceleration_waypoints(self,waypoints,closest_idx):
        # create a new waypoint
        temp =[]
        for i,wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # 2/3 waypoint back before the stop line
            stop_idx = max(self.stopline_wp_idx - closest_idx -2,0)
            # add the line segments
            dist = self.distance(waypoints,i,stop_idx)
            # velocity decelerate
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x)
            temp.append(p)
        return temp
        
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        #pass
    def traffic_cb(self,msg):
        # from traffic light
        self.stopline_wp_idx = msg.data
        #rospy.loginfo("****************** traffic_light_path: ")
                               
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        # TODO: Implement
        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
