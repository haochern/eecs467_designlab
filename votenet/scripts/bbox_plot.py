#!/home/hao/anaconda3/envs/votenet/bin/python

import rospy
from geometry_msgs.msg import Point

import rviz_tools as rviz_tools
from votenet_detection.msg import BoundingBox, BoundingBoxes

THRESHOLD = 0.75
# THRESHOLD = 0

# config for the bounded box
WIDTH = 0.02
LIFTTIME = 10

class bbox:
    def __init__(self) -> None:
        rospy.Subscriber('bbox', BoundingBoxes, self.plot_callback)
        self.markers = rviz_tools.RvizMarkers('imu_link', 'visualization_marker')

    def plot_callback(self, data):
        self.markers.deleteAllMarkers()
        
        for obj in data.array:
            corners = obj.bbox_corners
            for i in [0, 4]:
                path = [corners[i], corners[i+1], corners[i+2], corners[i+3], corners[i]]
                self.markers.publishPath(path, 'red', WIDTH, LIFTTIME)
            for i in range(4):
                self.markers.publishLine(corners[i], corners[4+i], 'red', WIDTH, LIFTTIME)
    

def main():
    rospy.init_node("bbox_plot")
    rospy.loginfo("start bbox_plot")
    bbox_ = bbox()
    rospy.spin()

if __name__ == '__main__':
    main()
    
    