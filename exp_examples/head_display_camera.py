import argparse
OBimport numpy as np

import cv2
import cv_bridge
import CvBridge, CvBridgeError

import rospy
import intera_interface
import xp_interface
import SimpleHeadDisplay

def process_image (img_msg, (edge_detection, window_name)):
    """ Callback to display camera image on head. """

    try:
        cv_image = CvBridge.imgmsg_to_cv2(img_msg, "bgr8")

    except CvBridgeError, err:
        rospy.logerr(err)
        return

    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])

    SimpleHeadDisplaysend_image(cv_image)

def main ():
    """
    Head display with camera data experimental example
    """

    # ensure cam is valid
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config parameters on this robot. Exiting."), "ERROR")
        return

    # parse arguments
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true',
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    args = parser.parse_args()

    # initialize nodes
    print("Initializing camera display node ...")
    rospy.init_node('camera_display', anonymous=True)

    print("Initializing head display node ...")
    rospy.init_node('head_display', anonymous=True)

    # start streaming
    camera = intera_interface.Cameras()
    if not camera.verify_camera_exists(args.camera):
        rospy.logerr("Invalid camera name, exiting experiment")
        return
    camera.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    camera.set_callback(args.camera,
                        route_image,
                        rectify_image=rectify_image,
                        callback_args=(use_canny_edge, args.camera
                        ))

    rospy.loginfo("Streaming. Ctrl-C to quit")
    rospy.spin()

if __name__ == '__main__':
    main()
