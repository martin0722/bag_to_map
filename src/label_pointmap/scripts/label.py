#!/usr/bin/env python2

import json
import os
import rospy

import geometry_msgs.msg
import visualization_msgs.msg

class Node:

    def __init__(self):
        rospy.init_node("label_point_cloud")
        self.commit_subscriber = rospy.Subscriber(
            "/move_base_simple/goal", geometry_msgs.msg.PoseStamped,
            self.commit_callback)
        self.label_subscriber = rospy.Subscriber(
            "/initialpose", geometry_msgs.msg.PoseWithCovarianceStamped,
            self.label_callback)
        self.rviz_markers_pub = rospy.Publisher('/markers',
            visualization_msgs.msg.MarkerArray, queue_size=0)
        self.rviz_marker_pub = rospy.Publisher('/current_marker',
            visualization_msgs.msg.Marker, queue_size=0)
        self.rviz_cursor_pub = rospy.Publisher('/current_cursor',
            visualization_msgs.msg.Marker, queue_size=0)
        self.output_file_path = '/tmp/label.json'
        self.markers_type = 'label'
        self.marker_counter = 0
        self.commit_marker = False
        self.markers = list()
        self.current_marker = self.create_marker()
        self.create_file()
        self.rviz_markers = visualization_msgs.msg.MarkerArray()
        self.current_rviz_marker = visualization_msgs.msg.Marker()
        self.reset_rviz_marker()

    def reset_rviz_marker(self):
        self.current_rviz_marker = visualization_msgs.msg.Marker()
        self.current_rviz_marker.header.frame_id = "/map"
        self.current_rviz_marker.ns = "points_and_lines"
        self.current_rviz_marker.action = visualization_msgs.msg.Marker.ADD
        self.current_rviz_marker.pose.orientation.w = 1.0
        self.current_rviz_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        self.current_rviz_marker.scale.x = 0.1
        self.current_rviz_marker.color.r = 1.0
        self.current_rviz_marker.color.g = 1.0
        self.current_rviz_marker.color.a = 1.0
        self.current_rviz_marker.lifetime = rospy.Duration(1.0)

    def create_file(self):
        if not os.path.exists(self.output_file_path):
            with open(self.output_file_path, "w") as output_file:
                json_data = { self.markers_type: list() }
                json.dump(json_data, output_file, indent=4)

    def create_marker(self):
        marker = {
            "id": str(self.marker_counter),
            "points": list(),
        }
        self.marker_counter += 1
        return marker

    def show_current_cursor(self):
        cursor_marker = visualization_msgs.msg.Marker()
        cursor_marker.header.frame_id = "/map"
        cursor_marker.ns = "points_and_lines"
        cursor_marker.id = 0
        cursor_marker.action = visualization_msgs.msg.Marker.ADD
        cursor_marker.pose.orientation.w = 1.0
        cursor_marker.type = visualization_msgs.msg.Marker.SPHERE
        cursor_marker.scale.x = 0.3
        cursor_marker.scale.y = 0.3
        cursor_marker.scale.z = 0.3
        cursor_marker.color.r = 1.0
        cursor_marker.color.a = 1.0
        if len(self.current_marker['points']) > 0:
            cursor_marker.pose.position.x = self.current_marker['points'][-1][0];
            cursor_marker.pose.position.y = self.current_marker['points'][-1][1];
            cursor_marker.pose.position.z = self.current_marker['points'][-1][2];
        elif len(self.markers) > 0:
            cursor_marker.pose.position.x = self.markers[-1]['points'][-1][0];
            cursor_marker.pose.position.y = self.markers[-1]['points'][-1][1];
            cursor_marker.pose.position.z = self.markers[-1]['points'][-1][2];
        else:
            cursor_marker.color.a = 0.0
        self.rviz_cursor_pub.publish(cursor_marker)

    def undo_once(self):
        if len(self.current_rviz_marker.points) > 0:
            self.current_rviz_marker.points.pop()
        else:
            if len(self.rviz_markers.markers) > 0:
                self.current_rviz_marker = self.rviz_markers.markers[-1]
                self.current_rviz_marker.points.pop()
                self.rviz_markers.markers.pop()

        if len(self.current_marker['points']) > 0:
            self.current_marker['points'].pop()
        else:
            if len(self.markers) > 0:
                self.current_marker = self.markers[-1]
                self.current_marker['points'].pop()
                self.markers.pop()
                rospy.loginfo(
                    "Remove marker #{}".format(
                        self.current_marker["id"]))
                self.marker_counter -= 1

    def undo_marker(self):
        if len(self.current_rviz_marker.points) > 0:
            self.reset_rviz_marker()
        else:
            if len(self.rviz_markers.markers) > 0:
                self.rviz_markers.markers.pop()

        if len(self.current_marker['points']) > 0:
            self.current_marker['points'] = list()
        else:
            if len(self.markers) > 0:
                self.markers.pop()
                self.marker_counter -= 1
                self.current_marker["id"] = self.marker_counter - 1
                rospy.loginfo(
                    "Remove marker #{}".format(
                        self.current_marker["id"]))

    def run(self):
        rospy.set_param("~undo_once", "false")
        rospy.set_param("~undo_marker", "false")
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.rviz_marker_pub.publish(self.current_rviz_marker)
            self.rviz_markers_pub.publish(self.rviz_markers)
            self.show_current_cursor()

            if self.commit_marker == True:
                self.rviz_markers.markers.append(self.current_rviz_marker)
                self.reset_rviz_marker()
                rospy.loginfo(
                    "Committing marker #{}".format(
                        self.current_marker["id"]))

                self.markers.append(self.current_marker)
                self.current_marker = self.create_marker()

                self.commit_marker = False

            if rospy.get_param("~undo_once") == True:
                self.undo_once()
                rospy.set_param("~undo_once", "false")

            if rospy.get_param("~undo_marker") == True:
                self.undo_marker()
                rospy.set_param("~undo_marker", "false")

        try:
            self.dump_marker()
        except ValueError as e:
            rospy.logerr(e)

    def label_callback(self, msg):
        point = [ msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z ]
        self.current_marker["points"].append(point)
        pt = geometry_msgs.msg.Point()
        pt.x = point[0]
        pt.y = point[1]
        pt.z = point[2]
        self.current_rviz_marker.points.append(pt);
        self.current_rviz_marker.id = int(self.marker_counter)

    def commit_callback(self, msg):
        point = [ msg.pose.position.x, msg.pose.position.y,
            msg.pose.position.z ]
        self.current_marker["points"].append(point)
        pt = geometry_msgs.msg.Point()
        pt.x = point[0]
        pt.y = point[1]
        pt.z = point[2]
        self.current_rviz_marker.points.append(pt);
        self.current_rviz_marker.id = int(self.marker_counter)
        self.commit_marker = True

    def dump_marker(self):
        json_data = dict()
        json_data[self.markers_type] = self.markers
        with open(self.output_file_path, "w") as output_file:
            json.dump(json_data, output_file, indent=4)

if __name__ == "__main__":
    node = Node()
    node.run()
