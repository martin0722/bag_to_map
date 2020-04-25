#!/usr/bin/env python2

import json
import os
import rospy

import geometry_msgs.msg
import visualization_msgs.msg

def init_rviz_marker(id):
    rviz_marker = visualization_msgs.msg.Marker()
    rviz_marker.header.frame_id = "/map"
    rviz_marker.ns = "points_and_lines"
    rviz_marker.id = id
    rviz_marker.action = visualization_msgs.msg.Marker.ADD
    rviz_marker.pose.orientation.w = 1.0
    rviz_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
    rviz_marker.scale.x = 0.1
    rviz_marker.color.b = 1.0
    rviz_marker.color.g = 1.0
    rviz_marker.color.a = 1.0
    return rviz_marker

class Node:

    def __init__(self):
        rospy.init_node("vis_json_line")
        self.rviz_markers_pub = rospy.Publisher('/line_markers',
            visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)
        with open('/tmp/lines.json') as jf:
            self.lines = json.load(jf)

    def run(self):
        rviz_markers = visualization_msgs.msg.MarkerArray()

        id = 0
        for line in self.lines['lines']:
            marker = init_rviz_marker(id)
            id += 1
            for pt in line:
                gpt = geometry_msgs.msg.Point()
                gpt.x = pt['x']
                gpt.y = pt['y']
                gpt.z = 1.0
                marker.points.append(gpt)
            rviz_markers.markers.append(marker)

        self.rviz_markers_pub.publish(rviz_markers)
        rospy.spin()



if __name__ == "__main__":
    node = Node()
    node.run()
