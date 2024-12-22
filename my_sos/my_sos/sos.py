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

        self.publisher_linear = self.create_publisher(
            Float32,
            'linear',
            10
        )

        self.asas = 0

        self.center_line = []

        self.msg = Int32()
        self.msg.data = -1

        self.publisher_ride.publish(self.msg)

        # self.top_x      = 62
        # self.top_y      = 4
        # self.bottom_x   = 150
        # self.bottom_y   = 120

        self.top_x      = 100
        self.top_y      = -20
        self.bottom_x   = 220
        self.bottom_y   = 200

        self.sub_image_original = self.create_subscription(Image, '/color/image', self.main_misl, 1)

        self.cvBridge = CvBridge()

    def img_homog(self, cv_image_original):
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        pts_src = np.array([[424 - self.top_x, 240 - self.top_y], [424 + self.top_x, 240 - self.top_y], [424 + self.bottom_x, 240 + self.bottom_y], [424 - self.bottom_x, 240 + self.bottom_y]])

        pts_dst = np.array([[148, 0], [500, 0], [500, 480], [148, 480]])

        h, status = cv2.findHomography(pts_src, pts_dst)

        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (848, 480))

        # if self.asas == 0:
        #     cv2.imwrite("damn_new.jpg", cv_image_original)
        #     self.asas = 1

        return cv_image_homography

    def find_center_of_mass_trapezoid(self, dot_mas):
        mas = np.unique(dot_mas, axis = 1)

        xc, yc = np.sum(mas, axis=0) / len(mas)

        return int(xc), int(yc)

    def road_dots(self, image):
        img = image.copy()

        h, w, _ = image.shape

        # heights = [h - 10, h - 110, h - 210]
        heights = [h - 10, h - 120, h - 230]

        widths = [20, 700]

        dot_mas = []

        flag = 0

        if len(self.center_line) == 0:
            self.center_line.append([w // 7 * 2 + 80, 0])
            self.center_line.append([w // 7 * 2 + 80, h])

        for z in range(len(heights)):
            row = img[heights[z], :]

            threshold_yellow = [0, 220, 230]    # INVERT!!! BGR
            threshold_white = [230, 230, 230]

            thresholds = [threshold_yellow, threshold_white]

            for threshold in thresholds:
                start = None
                end = None
                for i in range(len(row)):
                    if (row[i] > threshold).all() and start is None:
                        if threshold == threshold_yellow:
                            if row[i][0] < 20:
                                start = i
                        else:
                            start = i
                    elif (row[i] < threshold).any() and start is not None:
                        end = i
                        break
                if start is not None and end is None:
                    end = len(row) - 1

                if start is not None and end is not None:
                    center_x = (start + end) // 2
                    dot_mas.append([center_x, heights[z]])
                    # print(f"Центр вертикальной полосы на высоте {heights[z]}: {center_x}")
                else:
                    fl = 0
                    for width in widths:
                        start = None
                        end = None
                        column = img[:, width]
                        for i in range(len(column)):
                            if (column[i] > threshold).all() and start is None:
                                if threshold == threshold_yellow:
                                    if row[i][0] < 20:
                                        start = i
                                        # print('start: ', start)

                                else:
                                    start = i
                                    # print('start: ', start)
                            elif (column[i] < threshold).any() and start is not None:
                                end = i
                                # print('end: ', end)
                                break

                        if start is not None and end is not None:
                            center_y = (start + end) // 2
                            dot_mas.append([width, center_y])
                            fl = 1
                            break
                            
                    if fl == 0:
                        if z > 0:
                            print(dot_mas[2*z - 2])
                            dot_mas.append([dot_mas[2*z - 2][0], dot_mas[2*z - 2][1]])
                        else:
                            dot_mas.append([0, h])
            
        for point in dot_mas[::2]:
            x, y = point
            if x != None:
                cv2.circle(image, (x, y), radius=5, color=(0, 0, 0), thickness=-1)

        for point in dot_mas[1::2]:
            x, y = point
            if x != None:
                cv2.circle(image, (x, y), radius=5, color=(0, 255, 0), thickness=-1)
        
        angle = 0

        if flag == 0:
            xc1, yc1 = self.find_center_of_mass_trapezoid(dot_mas[0:4])
            xc2, yc2 = self.find_center_of_mass_trapezoid(dot_mas[2:6])

            if xc1 != xc2:
                oppos   = np.sqrt((xc1 - xc2)**2 + (yc1 - yc1)**2)
                near    = np.sqrt((xc2 - xc2)**2 + (yc1 - yc2)**2)

                angle   = np.arctan(oppos/near) * ((xc1 - xc2)/ np.abs(xc1 - xc2))

                angle_degrees = np.degrees(angle)
                cv2.putText(image, f"{angle_degrees} : {angle}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)


            cv2.line(image, dot_mas[0], dot_mas[3], (0, 0, 0), 1)
            cv2.line(image, dot_mas[1], dot_mas[2], (0, 0, 0), 1)

            cv2.line(image, dot_mas[2], dot_mas[5], (0, 0, 255), 1)
            cv2.line(image, dot_mas[3], dot_mas[4], (0, 0, 255), 1)

            cv2.line(image, (xc1, yc1), (xc2, yc2), (0, 255, 0), 1)
            cv2.line(image, (xc2, yc1), (xc2, yc2), (0, 0, 0), 1)
            cv2.line(image, (xc1, yc1), (xc2, yc1), (255, 255, 255), 1)

            cv2.line(image, (xc2, 420), (2 * xc2 - self.center_line[0][0], 320), (255, 0, 0), 2)

            angle_center_line = float(0)

            if xc2 != self.center_line[0][0]:
                oppos   = np.sqrt((xc2 - 2 * xc2 - self.center_line[0][0])**2 + (420 - 420)**2)
                near    = np.sqrt((xc2 - xc2)**2 + (420 - 320)**2)

                angle_center_line = np.arctan(oppos/near) * ((self.center_line[0][0] - xc2)/ np.abs(xc2 - self.center_line[0][0]))
                cv2.putText(image, f"{np.degrees(angle_center_line)} : {angle_center_line}", (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150, 0, 0), 2, cv2.LINE_AA)

        cv2.line(image, self.center_line[0], self.center_line[1], (0, 100, 100), 1)

        msg_angle = Float32()
        msg_linear = Float32()

        angle_coef = 1
        center_coef = 4
        norm_angle_const = 0.7

        angle = (angle / angle_coef + angle_center_line / center_coef) / 2
        norm_angle = angle / np.pi

        if np.abs(norm_angle) > norm_angle_const:
            angle = np.sign(angle) * norm_angle_const * np.pi
            norm_angle = np.sign(norm_angle) * norm_angle_const

        msg_angle.data = float(angle)

        linear = 0.4 * (0.55 - norm_angle)
        msg_linear.data = float(linear)

        self.publisher_linear.  publish(msg_linear)
        self.publisher_angle.   publish(msg_angle)

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

        cv2.imshow("soser_a", cv_image_homography)
        cv2.imshow("doser", cv_image_original)

        # cv2.imwrite("last.jpg", cv_image_original)
        # if self.asas == 0:
        #     cv2.imwrite("damn_new.jpg", cv_image_original)
        #     self.asas = 1
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()