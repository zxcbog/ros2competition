from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from glob import glob
import numpy as np
import rclpy
import cv2
import os


class ImageVisualizer(Node):

    def __init__(self):
        super().__init__('image_visualizer')
        
        self.bridge = CvBridge()
        self.subscription_color = self.create_subscription(
            Image,
            '/color/image',
            self.image_callback,
            10
        )
        
        self.subscription_depth = self.create_subscription(
            Image,
            '/depth/image',
            self.depth_callback,
            10
        )
        
        self.depth_image = 0

        self.depth_is_here = 0
        
        pkg_project = get_package_share_directory('my_sos')

        self.signs_name = [os.path.join(pkg_project, 'materials', 'pedestrian_crossing_sign.png'),  # 0
                           os.path.join(pkg_project, 'materials', 'traffic_construction.png'),      # 1
                           os.path.join(pkg_project, 'materials', 'traffic_intersection.png'),      # 2
                           os.path.join(pkg_project, 'materials', 'traffic_parking.png'),           # 3
                           os.path.join(pkg_project, 'materials', 'tunnel.png'),                    # 4
                           os.path.join(pkg_project, 'materials', 'traffic_left.png'),              # 5
                           os.path.join(pkg_project, 'materials', 'traffic_right.png')]             # 6

        self.signs_png = [cv2.imread(sign, cv2.IMREAD_GRAYSCALE) for sign in self.signs_name]
        
        self.sift = cv2.SIFT_create(edgeThreshold = 10, nfeatures = 80)

        self.des = [self.sift.detectAndCompute(sign, None)[1] for sign in self.signs_png]

        self.min_for_match = 15
        
        self.prev_sign = -2

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=70)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    def depth_callback(self, msg):
        try:
            # For depth images, the encoding is usually '32FC1' or '16UC1'
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=10)

        self.depth_is_here = 1
        self.image_shape = depth_array.shape
        
        self.depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        self.depth_image = np.uint8(self.depth_image)

        self.depth_image = np.abs(1 - self.depth_image[0:self.image_shape[0]//2, :])
        self.depth_image[self.depth_image < 240] = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")      # grayscale
        except CvBridgeError as e:
            self.get_logger().error(f'Ошибка преобразования изображения: {e}')
        
        if self.depth_is_here:
            # image_wised = cv2.bitwise_and(cv_image[0:self.image_shape[0]//2, :], self.depth_image)
            
            image_wised = cv_image.copy()

            _, des = self.sift.detectAndCompute(image_wised, None)

            if des is None:
                return

            if len(des) > 10:
                cv2.imshow('Bitwise image', image_wised)
                cv2.imshow('Depth image', self.depth_image)
                cv2.waitKey(1)

                good_matches = [0 for i in self.des]
                
                for i in range(len(self.des)):
                    # self.get_logger().info(f'{len(self.des[i])}, {len(des)}')
                    matches = self.flann.knnMatch(self.des[i], des, k=2)
                    for m, n in matches:
                        if m.distance < 0.8 * n.distance:
                            good_matches[i] += 1

                index = np.argmax(good_matches)

                what_sign = -1
                if good_matches[index] >= self.min_for_match:
                    what_sign = index
                
                if what_sign != self.prev_sign:
                    self.get_logger().info(f'Sign index: {what_sign}. Best match: {good_matches[index]}, min for match: {self.min_for_match}\
                                            Array: {good_matches}')
                    self.prev_sign = what_sign

def main(args=None):
    rclpy.init(args=args)
    image_visualizer = ImageVisualizer()
    rclpy.spin(image_visualizer)
    image_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
