import cv2
import pykinect_azure as pykinect

def transform_coords(x,y,z):
    tx = -y/-907 -0.63 #(-y + 600)/1000
    ty = -x/-965 - 0.27 #(-x + 250)/762
    tz = -z/1216 +1.17 #(-z + 1660)/667
    return [tx, ty, tz]

if __name__ == "__main__":
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

    cv2.namedWindow('Depth image with skeleton', cv2.WINDOW_NORMAL)

    while True:
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
            # left hand-wrist joint is 8, right 15
            fskelly = body_frame.get_body_skeleton()

            # Left hand joint
            lefthand = fskelly.joints[8]

            # Left hand x joint
            lhX = lefthand.position.xyz.x
            lhY = lefthand.position.xyz.y
            lhZ = lefthand.position.xyz.z

            coords = transform_coords(lhX, lhY, lhZ)



            print("Left hand is at:",coords[0],coords[1],coords[2])

        except:
            print("no body detected")

        # Overlay body segmentation on depth image
        cv2.imshow('Depth image with skeleton', combined_image)

        # Press 'q' key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    # Clean up
    device.stop()
    bodyTracker.stop()
    cv2.destroyAllWindows()

