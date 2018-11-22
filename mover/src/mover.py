#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import *
from tf import *

class Mover():
    def __init__(self):

        # subscribe to transforms
        self.tf = TransformListener()

        # subscribe to /map topic to get the occupancy grid 
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # subscribe to /odom topic to get the position of the robot with respect to the odom origin
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def map_callback(self, data):
        rospy.loginfo("Got map!")

        map_header = data.header
        map_meta_data = data.info
        self.map_origin_x = map_meta_data.origin.position.x
        self.map_origin_y = map_meta_data.origin.position.y
        self.map_data = data.data
        # rospy.loginfo('Got new map %s', self.map_meta_data.origin.position.x)

    def odom_callback(self, data):
        pose = data.pose.pose #  the x,y,z pose and quaternion orientation
        # rospy.loginfo('Got new odom %s', pose)

        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        # transform the pose in the odom fram to the map frame
        self.odom_to_map_transformer(self.odom_x, self.odom_y)

        rospy.loginfo('transform to map frame %s', self.odom_to_map_transformer(self.odom_x, self.odom_y))


    def odom_to_map_transformer(self, init_x, init_y):
        '''
        return a new tuple, that is now correct with reference to the map frame (x,y)
        '''
        (trans,rot) = self.tf.lookupTransform('/map','/odom',rospy.Time(0))

        delta_x = trans[0]
        delta_y = trans[1]
        return (init_x + delta_x, init_y + delta_y)        


    
def main():
    # initialize node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mover', anonymous=True)

    # create new instance of the class
    Mover()


if __name__ == '__main__':
    main()

    