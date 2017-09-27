#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.loginfo("The next traffic light state is %s, stop line at wp: %s", state, light_wp)
        print("The next traffic light state is %s, stop line at wp: %s", state, light_wp)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # self.waypoints, list of waypoints on the road
        if self.waypoints is None:
            return

        minimumDistance = 99999
        minimumDistanceIndex = None

        posePositionX = pose.position.x #position x coordinate of pose input
        posePositionY = pose.position.y #position y coordinate of pose input

    	# basic for loop for distance calculation between points,
    	# to find closest waypoint to the pose inputted, according to its X and Y coordinates.
        for index, waypoint in enumerate(self.waypoints):
            waypointPositionX = waypoint.pose.pose.position.x
            waypointPositionY = waypoint.pose.pose.position.y
            distanceCalculated = math.sqrt((posePositionX - waypointPositionX)**2 + (posePositionY - waypointPositionY)**2)
            if (distanceCalculated < minimumDistance):
                minimumDistance = distanceCalculated
		minimumDistanceIndex = index

        # returns the index of the closest waypoint
        return minimumDistanceIndex


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link", "/world", now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None, None

        if (trans and rot):
            rpy = tf.transformations.euler_from_quaternion(rot)
            yaw = rpy[2]

            (ptx, pty, ptz) = (point_in_world.pose.pose.position.x,
                point_in_world.pose.pose.position.y,
                point_in_world.pose.pose.position.z)

            point_to_cam = (ptx * math.cos(yaw) - pty * math.sin(yaw),
                ptx * math.sin(yaw) + pty * math.cos(yaw),
                ptz)
            point_to_cam = [sum(x) for x in zip(point_to_cam, transT)]

            point_to_cam[1] = point_to_cam[1] + offsetX
            point_to_cam[2] = point_to_cam[2] + offsetY

            ##########################################################################################
            # DELETE THIS MAYBE - MANUAL TWEAKS TO GET THE PROJECTION TO COME OUT CORRECTLY IN SIMULATOR
            # just override the simulator parameters. probably need a more reliable way to determine if
            # using simulator and not real car
            if fx < 10:
                fx = 2574
                fy = 2744
                point_to_cam[2] -= 1.0
                cx = image_height/2 + 70
                cy = image_height + 50
            ##########################################################################################

            x = -point_to_cam[1] * fx / point_to_cam[0];
            y = -point_to_cam[2] * fy / point_to_cam[0];

            x = int(x + cx)
            y = int(y + cy)
            #rospy.loginfo_throttle(3, "traffic light pixel (x,y): " + str(x) + "," + str(y))

            return (x, y)

        return (0, 0)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        '''
            The following code will draw a circle on the image
            cv2.circle(cv_image, (int(x), int(y)), 10, (255,0,0), thickness=-1)
            filename = '~/CarND-Capstone/imgs/trafficlights/Traffic_Detector-Img-X' + str(x) + "_Y" + str(y) + ".png"
            cv2.imwrite(filename, cv_image)
        '''
        cv2.circle(cv_image, (int(x), int(y)), 10, (255,0,0), thickness=-1)
        filename = '/home/student/CarND-Capstone/imgs/trafficlights/Traffic_Detector-Img-X' + str(x) + "_Y" + str(y) + ".png"
        cv2.imwrite(filename, cv_image)

        #TODO use light location to zoom in on traffic light in image
        light_state = None

        for traffic_light in self.lights:
            if (traffic_light.pose.pose.position == light.pose.pose.position):
                # a traffic light is found
                light_state = traffic_light.state
                break

        return light_state
        #Get classification (Kev: use this after we have our classifier)
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
