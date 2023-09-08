#!/usr/bin/env python
import rospy
import cv2
import pykinect_azure as pykinect
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import sys
from std_srvs.srv import Empty

print("Hello")

np.set_printoptions(threshold=sys.maxsize)
rospy.wait_for_service('/clear_octomap')
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

print("Hello")


def create_point_cloud2_message(points):
    # Assuming your 'points' variable is a NumPy array with shape (num_points, 3),
    # where each row contains the X, Y, and Z coordinates of a point.

    # Rotate the point cloud by 90 degrees
    rotated_points = np.copy(points)
    rotated_points[:, 0] = -points[:, 1] + 600  # Swap X and Y
    rotated_points[:, 1] = -points[:, 0] + 250 # Swap X and Y
    rotated_points[:, 2] = -points[:, 2] + 1660  # Negate Z

    # Scale down the rotated points by a factor of 1000
    rotated_points = rotated_points / 1000.0

    # Create the PointCloud2 message
    point_cloud_msg = PointCloud2()

    # Set the header information
    header = Header()
    header.stamp = rospy.Time.now()  # Use the current time
    header.frame_id = 'base'  # Set the frame ID (e.g., 'base_link' or 'map')
    point_cloud_msg.header = header

    # Set the dimensions of the point cloud
    point_cloud_msg.height = 1
    point_cloud_msg.width = rotated_points.shape[0]

    # Create the list of fields (X, Y, Z coordinates)
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    point_cloud_msg.fields = fields

    # Set the byte order (little-endian or big-endian)
    point_cloud_msg.is_bigendian = False

    # Set the point step (size of each individual point in bytes)
    point_cloud_msg.point_step = 12  # (3 fields * 4 bytes each)

    # Set the row step (size of a row in bytes)
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width

    # Convert the points array to a byte buffer
    points_buffer = rotated_points.astype(np.float32).tobytes()
    point_cloud_msg.data = points_buffer

    # Set the is_dense flag to indicate that there are no invalid points in the cloud
    point_cloud_msg.is_dense = True

    return point_cloud_msg

def update_image_and_cloud():

    print("test")
    iterations = 0

    # Initialize the ROS node
    rospy.init_node('point_cloud_publisher', anonymous=True)

    # Initialize the library, if the library is not found, add the library path as an argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    # Start device
    device = pykinect.start_device(config=device_config)

    # Create a publisher for the PointCloud2 message
    pub = rospy.Publisher('/your_point_cloud_topic', PointCloud2, queue_size=10)

    # Create an empty variable for storing the points
    points = None

    cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        iterations = iterations + 1

        # Get capture
        capture = device.update()

        # Get the color depth image from the capture
        ret_depth, depth_image = capture.get_colored_depth_image()

        # Get the 3D point cloud
        ret_points, points = capture.get_pointcloud()

        if not ret_depth or not ret_points:
            continue

        # Convert your points to a PointCloud2 message with rotation
        point_cloud_msg = create_point_cloud2_message(points)
        
        #clear octomap
        if iterations % 100 == 0:
            clear_octomap()
        
        # Publish the PointCloud2 message
        pub.publish(point_cloud_msg)

        # Plot the image
        cv2.imshow('Depth Image', depth_image)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    # Close the OpenCV window
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("Hello World")
    update_image_and_cloud()