import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Float32


class ImageProjection(Node):
    def __init__(self):
        super().__init__('image_projection')

        self.publisher_ride = self.create_publisher(
            Int32,
            'ride_or_not',
            10
        )

        self.publisher_angle = self.create_publisher(
            Float32,
            'angle',
            10
        )

        self.msg = Int32()
        self.msg.data = -1

        self.publisher_ride.publish(self.msg)

        self.top_x      = 62
        self.top_y      = 4
        self.bottom_x   = 150
        self.bottom_y   = 120

        self.sub_image_original = self.create_subscription(Image, '/color/image', self.main_misl, 1)

        # self.pub_image_projected = self.create_publisher(Image, '/color/image_output', 1)

        self.cvBridge = CvBridge()

    def img_homog(self, cv_image_original):
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        pts_src = np.array([[424 - self.top_x, 240 - self.top_y], [424 + self.top_x, 240 - self.top_y], [424 + self.bottom_x, 240 + self.bottom_y], [424 - self.bottom_x, 240 + self.bottom_y]])

        pts_dst = np.array([[148, 0], [600, 0], [600, 480], [148, 480]])

        h, status = cv2.findHomography(pts_src, pts_dst)

        height, width = cv_image_original.shape[:2]

        # cv2.line(cv_image_original, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)

        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (848, 480))

        return cv_image_homography

    def find_center_of_mass_trapezoid(self, dot_mas):
        x1, y1 = dot_mas[0]
        x2, y2 = dot_mas[1]
        x3, y3 = dot_mas[3]
        x4, y4 = dot_mas[2]

        # Длины оснований
        L = abs(x2 - x1)
        l = abs(x4 - x3)
        
        # Высота трапеции
        h = abs(y2 - y3)
        
        # Координата x центра массы
        xc = ((L + 2 * l) * x1 + (2 * L + l) * x3) / (3 * (L + l))
        
        # Координата y центра массы
        yc = h * (y1 + y3) / 2
        
        return xc, yc

    def road_dots(self, image):
        h, w, c = image.shape

        heights = [100, h // 2, h - 100]

        dot_mas = []

        flag = 0

        for z in range(len(heights)):
            row = image[heights[z], :]

            threshold_yellow = [0, 200, 200]    # INVERT!!! BGR
            threshold_white = [230, 230, 230]

            thresholds = [threshold_yellow, threshold_white]

            for threshold in thresholds:
                start = None
                end = None
                for i in range(len(row)):
                    if (row[i] > threshold).all() and start is None:
                        start = i
                    elif (row[i] < threshold).any() and start is not None:
                        end = i
                        break

                if start is not None and end is not None:
                    center_x = (start + end) // 2
                    dot_mas.append([center_x, heights[len(heights) - z - 1]])
                    # print(f"Центр вертикальной полосы на высоте {heights[z]}: {center_x}")
                else:
                    dot_mas.append([None, heights[len(heights) - z - 1]])
                    flag = 1
                    # print("Полоса не найдена")
        
        if flag == 0:
            xc1, yc1 = self.find_center_of_mass_trapezoid(dot_mas[0:4])
            xc2, yc2 = self.find_center_of_mass_trapezoid(dot_mas[2:6])

            d_BC = np.sqrt((xc1 - xc2)**2 + (yc2 - yc2)**2)
            d_AC = np.sqrt((xc1 - xc1)**2 + (yc2 - yc1)**2)

            near = d_AC
            oppos = d_BC

            angle = np.arctan(oppos/near) * ((xc1 - xc2)/ np.abs(xc1 - xc2))
            # print(f"Острый угол равен: {angle} radians.")

            # angle_degrees = np.degrees(angle)
            # print(f"Острый угол равен: {angle_degrees} градусов.")
        else:
            angle = 0

        msg = Float32()
        msg.data = float(angle)

        self.publisher_angle.publish(msg)

    def main_misl(self, msg_img):
        cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        cv_image_homography = self.img_homog(cv_image_original)

        # self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))

        if self.msg.data == -1:
            lower_green = np.array([0, 100, 0])
            upper_green = np.array([50, 255, 50])

            green_mask = cv2.inRange(np.asarray(cv_image_homography), lower_green, upper_green)

            green_pixels_count = np.sum(green_mask > 0)
            
            if green_pixels_count > 40:
                self.msg.data = 1
                self.publisher_ride.publish(self.msg)
                # self.msg.data = -1
        
        if self.msg.data == 1:
            self.road_dots(cv_image_homography)
            self.publisher_ride.publish(self.msg)

        cv2.imshow("soser", cv_image_homography)
        cv2.imshow("doser", cv_image_original)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()