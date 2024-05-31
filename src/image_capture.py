#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



def main():
    window_title = "CSI Camera"
    rospy.init_node('image_capture_node', anonymous=True)


    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)

    video_capture = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    

    if not video_capture.isOpened():
        print("Failed to open camera.")
        return
    else:
	try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

	    print("Hit ESC to exit")
	    print("Hit enter to save img")

	    while not rospy.is_shutdown():
		ret_val, frame = video_capture.read()
		if not ret_val:
		    print("Capture read error")
		    break
		if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
		    cv2.imshow(window_title, frame)
		else:
		    break 

		keycode_trigger = cv2.waitKey(10) & 0xff
		if keycode_trigger == 13:
		    timestamp = rospy.Time.now()
		    timestamp_str = f"{timestamp.secs}_{timestamp.nsecs}"

		    base_path = "/home/uvify/Desktop/survey/"
		    original_filename = "captured_image"

		    filename = f"{base_path}{original_filename}_{timestamp_str}.jpg"
		    cv2.imwrite(filename, frame)
		    print("Image captured:", filename)

		keycode = cv2.waitKey(10) & 0xff
		if keycode == 27:
		    break
	finally:
	    	video_capture.release()
 		cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

