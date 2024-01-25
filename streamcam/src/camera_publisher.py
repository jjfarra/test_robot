#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)

    # Inicializa cv_bridge
    bridge = CvBridge()

    # Captura de la cámara (ajusta el índice de la cámara según sea necesario)
    cap = cv2.VideoCapture(0)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                # Convierte el frame de OpenCV a un mensaje ROS
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_pub.publish(ros_image)
            except CvBridgeError as e:
                print(e)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass

