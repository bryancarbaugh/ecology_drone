#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import *
from tf import *

class Mover():
    def __init__(self):

        # subscribe to transforms
        self.tf = TransformListener()

        # some variables we'll use later
        # self.map_x
        # self.map_y
        # self.grid_res
        # self.grid_w
        # self.grid_h

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
        self.grid_origin_x = map_meta_data.origin.position.x
        self.grid_origin_y = map_meta_data.origin.position.y
        self.grid_data = data.data

        self.grid_res = map_meta_data.resolution
        self.grid_w = map_meta_data.width
        self.grid_h = map_meta_data.height





        rospy.loginfo('Got new map with res %s', self.grid_res)

    def odom_callback(self, data):
        pose = data.pose.pose #  the x,y,z pose and quaternion orientation
        # rospy.loginfo('Got new odom %s', pose)

        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        # transform the pose in the odom fram to the map frame
        self.odom_to_map_transformer(self.odom_x, self.odom_y)
        self.map_x, self.map_y = self.odom_to_map_transformer(self.odom_x, self.odom_y)

        # get the occupancy grid pixels the correlate to the map position
        pixel_x, pixel_y = self.map_coord_to_pixel(self.map_x, self.map_y)

        # get the value of the occupancy grid in that pixel
        pixe_value = self.grid_pixel_value(pixel_x, pixel_y)
        self.show_surrounding_pixels()
        # rospy.loginfo('transform to map frame %s', self.odom_to_map_transformer(self.odom_x, self.odom_y))


    def odom_to_map_transformer(self, init_x, init_y):
        '''
        return a new tuple, that is now correct with reference to the map frame (x,y)
        '''
        (trans,rot) = self.tf.lookupTransform('/map','/odom',rospy.Time(0))

        delta_x = trans[0]
        delta_y = trans[1]
        return (init_x + delta_x, init_y + delta_y)        

    def map_coord_to_pixel(self, x,y):
        '''
        returns the row and col that correspond to the map-frame coordinates in the occupancy map
        '''
        # localize the robot on the occupancy grid
        pixel_x = round((x - self.grid_origin_x)/self.grid_res)
        pixel_y = round((y - self.grid_origin_y)/self.grid_res)
        return (pixel_x, pixel_y)

    def grid_pixel_value(self, pixel_x, pixel_y):
        '''
        return the occupancy value [0,100] of the pixel of interest in the occupancy grid
        '''
        
        grid_index = int(pixel_x + pixel_y * self.grid_w)
        # rospy.loginfo('we are at x=%s y=%s in the grid, with value %s', pixel_x, pixel_y, self.grid_data[grid_index])

        return self.grid_data[grid_index]

    def show_surrounding_pixels(self):
        surrounding_list = []
        for i in range(-10, 10):
            x_delta = i/10.0
            x, y = self.map_coord_to_pixel(self.map_x + x_delta, self.map_y)
            surrounding_list.append(self.grid_pixel_value(x,y))
        print(surrounding_list)
            





        
    
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

    