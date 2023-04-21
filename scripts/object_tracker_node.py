import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

class ObjectTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.position_pub = rospy.Publisher("/object_position", Point, queue_size=10)
        self.home_pub = rospy.Publisher("/home", Empty, queue_size=10)
        self.depth_image = None
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback)

        self.create_slider_ui()

    def create_slider_ui(self):
        cv2.namedWindow('Tracked Object')
        cv2.createTrackbar('Min H', 'Tracked Object', 0, 180, self.nothing)
        cv2.createTrackbar('Min S', 'Tracked Object', 0, 255, self.nothing)
        cv2.createTrackbar('Min V', 'Tracked Object', 0, 255, self.nothing)
        cv2.createTrackbar('Max H', 'Tracked Object', 180, 180, self.nothing)
        cv2.createTrackbar('Max S', 'Tracked Object', 255, 255, self.nothing)
        cv2.createTrackbar('Max V', 'Tracked Object', 255, 255, self.nothing)
        cv2.createTrackbar('Min Size', 'Tracked Object', 0, 10000, self.nothing)

    def nothing(self, x):
        pass

    def track_largest_colored_object(self, image):
        min_h = cv2.getTrackbarPos('Min H', 'Tracked Object')
        min_s = cv2.getTrackbarPos('Min S', 'Tracked Object')
        min_v = cv2.getTrackbarPos('Min V', 'Tracked Object')
        max_h = cv2.getTrackbarPos('Max H', 'Tracked Object')
        max_s = cv2.getTrackbarPos('Max S', 'Tracked Object')
        max_v = cv2.getTrackbarPos('Max V', 'Tracked Object')
        min_size_value = cv2.getTrackbarPos('Min Size', 'Tracked Object')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_color = np.array([min_h, min_s, min_v])
        upper_color = np.array([max_h, max_s, max_v])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None

        valid_contours = [c for c in contours if cv2.contourArea(c) > min_size_value]
        if not valid_contours:
            return None, None

        largest_contour = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(image, "X: {}, Y: {}".format(x + w // 2, y + h // 2), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, (x + w // 2, y + h // 2)

    def image_callback(self, data):
        if self.depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        tracked_object, object_pos = self.track_largest_colored_object(cv_image)
        if tracked_object is not None:
            smaller_size = (320, 240)
            tracked_object_resized = cv2.resize(tracked_object, smaller_size)

            cv2.imshow("Tracked Object", tracked_object_resized)
            cv2.waitKey(1)

            x, y = object_pos
            depth_image_height, depth_image_width = self.depth_image.shape

            # Check if x and y are within the depth image dimensions
            if 0 <= x < depth_image_width and 0 <= y < depth_image_height:
                depth = selfdepth_image[y, x]
                point = Point(x, y, depth)
                self.position_pub.publish(point)

    def depth_image_callback(self, data):
        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        self.depth_image = cv_depth_image

if __name__ == '__main__':
    rospy.init_node('object_tracker', anonymous=True)
    ot = ObjectTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
