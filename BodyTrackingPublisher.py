import cv2
import pykinect_azure as pykinect
import rospy
from std_msgs.msg import Float64MultiArray

def transform_coords(x,y,z):
    tx = -y/-907 -0.63
    ty = -x/-965 - 0.27
    tz = -z/1216 +1.17
    return [tx, ty, tz]

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('kinect_skeleton_publisher')

    # Initialize the library, if the library is not found, add the library path as an argument
    pykinect.initialize_libraries(track_body=True)

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    # Start device
    device = pykinect.start_device(config=device_config)

    # Start body tracker
    bodyTracker = pykinect.start_body_tracker()

    # Create a ROS publisher for the joint coordinates
    joints_publisher = rospy.Publisher('/joint_coordinates', Float64MultiArray, queue_size=10)

    cv2.namedWindow('Depth image with skeleton', cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        # Get capture
        capture = device.update()

        # Get body tracker frame
        body_frame = bodyTracker.update()

        # Get the color depth image from the capture
        ret_depth, depth_color_image = capture.get_colored_depth_image()

        # Get the colored body segmentation
        ret_color, body_image_color = body_frame.get_segmentation_image()

        if not ret_depth or not ret_color:
            continue

        # Combine both images
        combined_image = cv2.addWeighted(depth_color_image, 0.6, body_image_color, 0.4, 0)

        # Draw the skeletons
        combined_image = body_frame.draw_bodies(combined_image)

        try:
            print("Body detected")
            # Get the body skeleton
            fskelly = body_frame.get_body_skeleton()

            # Create a list to store joint coordinates
            joint_coords = []

            for joint in fskelly.joints:
                x = joint.position.xyz.x
                y = joint.position.xyz.y
                z = joint.position.xyz.z
                coords = transform_coords(x, y, z)
                joint_coords.extend(coords)

            lhx = joint_coords[24]
            lhy = joint_coords[25]
            lhz = joint_coords[26]

            print("left hand is at: ", lhx,lhy,lhz)

            # Publish the joint coordinates as a Float64MultiArray
            joints_msg = Float64MultiArray(data=joint_coords)
            joints_publisher.publish(joints_msg)

        except:
            print("No body detected")

        # Overlay body segmentation on depth image
        cv2.imshow('Depth image with skeleton', combined_image)

        # Press 'q' key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    # Clean up
    device.stop()
    bodyTracker.stop()
    cv2.destroyAllWindows()
