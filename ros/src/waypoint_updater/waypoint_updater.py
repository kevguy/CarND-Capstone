#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DEBUG_MODE = False

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.next_waypoints = None

        rospy.logerr("What's up bitch")

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.logerr('got pose')
        self.current_pose = msg.pose
        self.publish_waypoints()
        # pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # only do this once
        rospy.logerr('got waypoints')
        if self.next_waypoints is None:
            rospy.logerr('waypoints are none')
            self.next_waypoints = waypoints.waypoints
            self.publish_waypoints()
        # pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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

    def get_closest_waypoint(self, pose, waypoints):
        # ripping off from path planning
        closest_len = 100000
        closest_waypoint_index = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for index, waypoint in enumerate(waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint_index = index

        return closest_waypoint_index

    def get_next_waypoint(self, pose, waypoints):
        # also ripping off path planning
        closest_waypoint_index = self.get_closest_waypoint(pose, waypoints)
        map_x = waypoints[closest_waypoint_index].pose.pose.position.x
        map_y = waypoints[closest_waypoint_index].pose.pose.position.y

        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)

        if (angle > (math.pi / 4)):
            closest_waypoint_index += 1

        return closest_waypoint_index

    def publish_waypoints(self):
        if self.current_pose is not None and self.next_waypoints is not None:
            rospy.logerr("Publishing waypoints")
            waypoints_size = len(self.next_waypoints)
            next_waypoint_index = self.get_next_waypoint(self.current_pose, self.next_waypoints)
            lookahead_waypoints = self.next_waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]

            # publish_waypoints
            final_waypoints_msg = Lane()
            final_waypoints_msg.header.frame_id = '/world'
            final_waypoints_msg.header.stamp = rospy.Time(0)
            final_waypoints_msg.waypoints = lookahead_waypoints
            self.final_waypoints_pub.publish(final_waypoints_msg)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
