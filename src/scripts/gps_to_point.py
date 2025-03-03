#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_point():
    # Initialize the node
    rospy.init_node('publish_gps_point', anonymous=True)

    # Create a publisher for the Marker messages
    pub = rospy.Publisher('/gps_points', Marker, queue_size=10)

    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "map"  # Ensure this frame matches your RViz setup
    marker.header.stamp = rospy.Time.now()  # Use the system time
    marker.ns = "gps_points"
    marker.id = 0
    marker.type = Marker.POINTS  # We're using POINTS type for visualization
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Size of the points
    marker.scale.y = 0.1
    marker.color.a = 1.0  # Fully opaque
    marker.color.r = 1.0  # Red color

    # Create a point and add it to the marker's points list
    point = Point()
    point.x = 1.0  # Set the x, y, z coordinates to your desired GPS point
    point.y = 2.0
    point.z = 0.0
    marker.points.append(point)

    # Publish the marker repeatedly (1Hz)
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()  # Update the timestamp
        pub.publish(marker)  # Publish the marker
        rospy.sleep(1)  # Sleep for 1 second between publishes

if __name__ == '__main__':
    try:
        publish_point()  # Run the function to publish points
    except rospy.ROSInterruptException:
        pass