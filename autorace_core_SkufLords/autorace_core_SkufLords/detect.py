import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO
import cv2
import time
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory

package_name = 'autorace_core_SkufLords'

def gauss_kernel(x,z=0,sigma=1):
    return 1/(sigma * np.sqrt(2 * np.pi)) * np.exp((-((x-z)**2))/(2*sigma**2))

def quaternion_to_euler_angle_vectorized(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 

class LidarGraphMaker(Node):
    def __init__(self):
        pkg_project = get_package_share_directory(package_name)
        model_path = os.path.join(pkg_project, 'util', 'best.pt')
        
        self.model: YOLO = YOLO(model_path)

        self.bridge = CvBridge()
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.depth_scan = self.create_subscription(Image, '/depth/image', self.depth_callback, 1)
        self.color_scan = self.create_subscription(Image, '/color/image', self.image_callback, 1)
        self.odom_scan = self.create_subscription(Odometry, '/odom', self.get_odom, 1)
        self.check_publisher = self.create_publisher(Int32, "/check", 1)
        self.lidar_scan = self.create_subscription(LaserScan, '/scan', self.move_with_odom, 1)
        self.start_pos = None
        self.start_orient = None
        self.odometry = None
        self.moving_start_pos = None
        self.state = 0
        self.signs = {
            "t": False,
            "direction": False,
            "work": False,
            "parking": False,
            "pedestrian": False,
            "tunnel": False
        }
        self.passed_work = False      # перевести в False
        self.passed_parking = False
        self.passed_pedestrian = False
        self.passed_tunnel = False
        self.depth_image = None
        self.avg_depth = None
        self.state = 0
        
        self.n_window   = 5
        self.n_angles   = 0
        self.angles     = []
        self.sum_angles = 0
    
    def get_odom(self, odom_data: Odometry):
        if self.odometry is None:
            self.odometry = {}
        pose_data = odom_data.pose.pose
        q_w, q_x, q_y, q_z = pose_data.orientation.w, pose_data.orientation.x, pose_data.orientation.y, pose_data.orientation.z
        x, y, z = quaternion_to_euler_angle_vectorized(q_w, q_x, q_y, q_z)
        if self.start_orient is None:
            self.start_orient = {}
            self.start_orient['x'] = x
            self.start_orient['y'] = y
            self.start_orient['z'] = z
        if self.start_pos is None:
            self.start_pos = {}
            self.start_pos['x'] = pose_data.position.x
            self.start_pos['y'] = pose_data.position.y
            self.start_pos['z'] = pose_data.position.z
        self.odometry['pos'] = {}
        self.odometry['pos']['x'] = self.start_pos['x'] - pose_data.position.x
        self.odometry['pos']['y'] = self.start_pos['y'] - pose_data.position.y
        self.odometry['pos']['z'] = self.start_pos['z'] - pose_data.position.z 
        self.odometry['orient'] = {}
        self.odometry['orient']['x'] = self.start_orient['x'] - x
        self.odometry['orient']['y'] = self.start_orient['y'] - y
        self.odometry['orient']['z'] = self.start_orient['z'] - z
        self.odometry['time'] = odom_data.header.stamp.sec + 1e-9 * odom_data.header.stamp.nanosec


    def start_moving(self, current_pos):
        if self.moving_start_pos is None:
            self.moving_start_pos = current_pos
        x0 = self.moving_start_pos['x']
        y0 = self.moving_start_pos['y']
        x1 = current_pos['x']
        y1 = current_pos['y']
        return np.sqrt((x1 - x0)**2 + (y1 - y0)**2)

    def work_move(self, data, current_pos, current_orient):
        vel_msg = Twist()
        window_size = 10
        right_data = data[180+90-window_size:180+90+window_size]
        left_data = data[180-90-window_size:180-90+window_size]
        right_backward_data = data[180+135-25:180+135+window_size]
        left_backward_data = data[180-135-window_size:180-135+window_size]
        forward_data = data[180-window_size:180+window_size]
        backward_data = np.concatenate((data[-window_size:],data[:window_size]))
        inertial_angle_bias = 8
        linear_speed = 0.2
        angular_speed = 0.3
        if self.state == 0:# прямо до препятствия
            if left_data.mean() > 0.5:
                vel_msg.linear.x = 0.1
            else:
                vel_msg.linear.x = 0.0
                self.state += 1
        elif self.state == 1:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.2:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 2:
            if current_orient['z'] > -90 + inertial_angle_bias:
                vel_msg.angular.z = angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 3:
            length = self.start_moving(current_pos=current_pos)
            if forward_data.mean() > 0.3:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 4:
            if current_orient['z'] > -176 + inertial_angle_bias:
                vel_msg.angular.z = angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 5:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.2:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 6:
            if current_orient['z'] < -90 - inertial_angle_bias:
                vel_msg.angular.z = -angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 7:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.3:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 8:
            if current_orient['z'] < 0 - inertial_angle_bias:
                vel_msg.angular.z = -angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 9:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.25:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 10:
            if current_orient['z'] > -90 + inertial_angle_bias:
                vel_msg.angular.z = angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 11:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.3:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 12:
            if current_orient['z'] > -150 + inertial_angle_bias:
                vel_msg.angular.z = angular_speed
            else:
                self.moving_start_pos = None
                self.state = 0
                vel_msg.angular.z = 0.0
                ans = Int32()
                ans.data = 1
                self.check_publisher.publish(ans)
                self.passed_work = True
            
        self.movement_publisher.publish(vel_msg)

    def parking_move(self, data, current_pos, current_orient):
        vel_msg = Twist()
        window_size = 30
        forward_data = data[180-window_size:180+window_size]
        right_data = data[180+90-window_size:180+90+window_size]
        left_data = data[180-90-window_size:180-90+window_size]
        inertial_angle_bias = 8
        linear_speed = 0.2
        angular_speed = 0.3
        print(self.state)
        if self.state == 0:
            length = self.start_moving(current_pos=current_pos)
            if length < 0.3:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        elif self.state == 1:
            length = self.start_moving(current_pos=current_pos)
            if current_orient['z'] > 90 + inertial_angle_bias:
                vel_msg.angular.z = angular_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.angular.z = 0.0
        elif self.state == 2:
            length = self.start_moving(current_pos=current_pos)
            if left_data.mean() < 0.5 or right_data.mean() < 0.5:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.state += 1 
                vel_msg.linear.x = 0.0
        self.movement_publisher.publish(vel_msg)

    def pedestrian_move(self, data, current_pos, current_orient):
        vel_msg = Twist()
        window_size = 30
        forward_data = data[180-window_size:180+window_size]
        linear_speed = 0.2
        angular_speed = 0.3
        if self.state == 0:
            length = self.start_moving(current_pos=current_pos)
            if forward_data.min() > 0.2 and length < 1:
                vel_msg.linear.x = linear_speed
            else:
                self.moving_start_pos = None
                self.passed_pedestrian = True
                self.state = 0 
                ans = Int32()
                ans.data = 1
                self.check_publisher.publish(ans)
                vel_msg.linear.x = 0.0
            
        self.movement_publisher.publish(vel_msg)

    def pass_tunnel(self, laser_data: LaserScan):
        data = np.array(laser_data.ranges[180:] + laser_data.ranges[:180])
        
        data[data>10] = 2
        data[data<0] = 0
        window_range = 15
        kernel = gauss_kernel(np.linspace(-1,1,window_range))
        bias = 0
        
        FOV = 90
        distances = np.convolve(data[180-FOV//2:180+FOV//2], np.ones(window_range)) / window_range
        #distances = np.convolve(data[180-FOV//2:180+FOV//2], kernel) / window_range
        #distances = median_filter(data[180-FOV//2:180+FOV//2], window_range)
        
        angle = np.argmax(distances) - (FOV // 2) - 5

        side_window = 20
        sideFOV = 45
        vel_msg = Twist()

        front_view  = data[170:190].mean()
        right_view  = data[170 - 50:190 - window_range].mean()
        left_view   = data[170 + window_range:190 + 50].mean()

        left_views = []
        right_views = []
        for i in range(35):
            left_views.append(data[170 + window_range + i:190 + window_range + i].mean())
            right_views.append(data[170 - window_range - i:190 - window_range - i].mean())
        
        left_view = min(left_views)
        right_view = min(right_views)
        # if max(left_views) > min(left_views) + 0.5:
        #     # need changes in angle

        # left_angle = data[180-sideFOV-side_window:180-sideFOV+side_window].mean()
        # right_angle = data[180+sideFOV-side_window:180+sideFOV+side_window].mean()
        print(angle, front_view, distances.mean())
        print(angle)

        print(left_view, front_view, right_view)

        angle_radians = float(np.deg2rad(angle))

        speed = 0.3

        border = 0.6
        delta_min_max = 0.4

        if distances.mean() < 1.8:
            if front_view > 0.3:
                if max(left_views) > min(left_views) + delta_min_max and left_view < border:
                    angle_radians = float(angle_radians - np.deg2rad(90 * (border - left_view)))
                    print(f'left_view: {left_view}, angle_radians: {angle_radians}')
                elif max(right_views) > min(right_views) + delta_min_max and right_view < border:
                    angle_radians = float(angle_radians + np.deg2rad(90 * (border - right_view)))
                    print(f'right_view: {right_view}, angle_radians: {angle_radians}')
                else:
                    print(f'front_view: {front_view}, angle_radians: {angle_radians}')

                if len(self.angles) < self.n_window:
                    self.sum_angles += angle_radians
                    self.angles.append(angle_radians)
                else:
                    self.sum_angles += angle_radians
                    self.sum_angles -= self.angles[0]
                    self.angles.append(angle_radians)
                    self.angles = self.angles[1:len(self.angles)]

                angle_radians = self.sum_angles / len(self.angles)

                vel_msg.linear.x = speed * (1 - 1.5*np.abs(angle_radians / np.pi))
                vel_msg.angular.z = angle_radians
            else:
                if len(self.angles) < self.n_window:
                    self.sum_angles += angle_radians
                    self.angles.append(angle_radians)
                else:
                    self.sum_angles += angle_radians
                    self.sum_angles -= self.angles[0]
                    self.angles.append(angle_radians)
                    self.angles = self.angles[1:len(self.angles)]

                angle_radians = self.sum_angles / len(self.angles)

                vel_msg.linear.x = -speed * (1 - np.abs(angle_radians))
                vel_msg.angular.z = - angle_radians
                
            self.movement_publisher.publish(vel_msg)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.movement_publisher.publish(vel_msg)

    def move_with_odom(self, laser_data: LaserScan):
        if self.odometry is None:
            return
        current_pos = self.odometry['pos']
        current_orient = self.odometry['orient']
        data = np.array(laser_data.ranges[180:] + laser_data.ranges[:180])
        
        data = data[::-1]
        data[data > 5] = 5
        data[data < 0] = 0
        
        if self.signs['work'] and not self.passed_work:
            print(current_pos)
            print(current_orient)
            self.work_move(data, current_pos, current_orient)
        elif self.signs['parking'] and not self.passed_parking:
            print(current_pos)
            print(current_orient)
            self.parking_move(data, current_pos, current_orient)
        elif self.signs['pedestrian'] and not self.passed_pedestrian:
            print(current_pos)
            print(current_orient)
            self.pedestrian_move(data, current_pos, current_orient)
        elif self.signs['tunnel'] and not self.passed_tunnel:
            self.pass_tunnel(laser_data)

    def depth_callback(self, msg):
        try:
            # For depth images, the encoding is usually '32FC1' or '16UC1'
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=10)
        
        self.depth_is_here = 1
        self.image_shape = depth_array.shape
        
        self.depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        self.depth_image = np.uint8(self.depth_image)

        self.depth_image = np.abs(1 - self.depth_image[0:self.image_shape[0]//2, :])
        
        if self.avg_depth is None:
            self.avg_depth = np.zeros_like(self.depth_image)
        self.depth_image[self.depth_image < 240] = 0
        img = self.depth_image
        _, threshold = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY) 
        i = 0

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        mask = np.zeros_like(img)
        for contour in contours:
            if i == 0:
                i = 1
                continue

            # Функция cv2.approxPolyDP() для аппроксимации формы
            
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # Использование функции drawContours() для рисования контуров
            #if len(approx):
            if 5 < len(approx) < 20:
                cv2.drawContours(mask, [contour], -1, (255), thickness=cv2.FILLED)

        self.depth_image = self.depth_image[:200,:]
        self.avg_depth = mask[:, 400:]
        cv2.imshow('depth', self.avg_depth)
        cv2.waitKey(1)

    def image_callback(self, msg: Image):
        if (self.signs['work'] and not self.passed_work) or \
           (self.signs['pedestrian'] and not self.passed_pedestrian):
            return
        if (self.depth_image is not None and 7 < self.avg_depth.mean() < 20 or \
            (self.passed_pedestrian and not self.signs['tunnel']) or \
            (self.signs['t'] and not self.signs['direction'])):
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            yolo_result = self.model(image)[0]
            boxes = []
            mask = self.avg_depth
            for box in yolo_result.boxes:
                class_id = int(box.cls)
                confidence = box.conf[0]
                label = yolo_result.names[class_id]
                width = box.xyxy[0][2] - box.xyxy[0][0]
                height = box.xyxy[0][3] - box.xyxy[0][1]
                aspect_ratio = width / height
                x0 = int(box.xyxy[0][0].item()) + 400
                y0 = int(box.xyxy[0][1].item())
                x1 = int(box.xyxy[0][2].item()) + 400
                y1 = int(box.xyxy[0][3].item())
                y0 = y0 if y0 < (image.shape[0] // 2) else 0
                y1 = y1 if y1 < (image.shape[0] // 2) else 0
                x0 = x0 if x0 < (image.shape[1]) else image.shape[1] - 1
                x1 = x1 if x1 < (image.shape[1]) else image.shape[1] - 1
                print(aspect_ratio, label, width * height)
                
                if (width * height > 5000) and \
                    0.7 < aspect_ratio < 1.5:
                
                    box_result = {
                        'x_min': x0,  # xmin coordinate
                        'y_min': y0,  # ymin coordinate
                        'x_max': x1,  # xmax coordinate
                        'y_max': y1,  # ymax coordinate
                        'width': width,  # width of the box
                        'height': height,  # height of the box
                        'area': (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]),  # area of the box
                        'label': label,
                        'conf': confidence,
                    }
                    boxes.append(box_result)
            if len(boxes) > 0:
                print(len(boxes), boxes[-1]['label'])
                if boxes[-1]['label'] == "T_crossroad" and not self.signs['t']:
                    self.signs['t'] = True
                    time.sleep(7.5)
                    vel_msg = Twist()
                    vel_msg.angular.z = 1.0
                    self.movement_publisher.publish(vel_msg)
                    ans = Int32()
                    ans.data = -1
                    self.check_publisher.publish(ans)
                    print("t")
                elif boxes[-1]['label'] == "left_sign" and not self.signs['direction']:
                    print("dir")
                    self.signs['direction'] = True
                    ans = Int32()
                    ans.data = 3
                    self.check_publisher.publish(ans)
                elif boxes[-1]['label'] == "right_sign" and not self.signs['direction']:
                    print("dir")
                    self.signs['direction'] = True
                    ans = Int32()
                    ans.data = 2
                    self.check_publisher.publish(ans)
                elif boxes[-1]['label'] == "works_sign" and self.signs['direction'] and not self.signs['work']:
                    print("work")
                    self.signs['work'] = True
                    ans = Int32()
                    ans.data = 1
                    self.check_publisher.publish(ans)
                    time.sleep(5)
                    ans.data = -1
                    self.check_publisher.publish(ans)
                elif boxes[-1]['label'] == "parking_sign" and self.passed_work:
                    print("parking")
                    ans = Int32()
                    ans.data = -1
                    self.signs['parking'] = True
                    self.check_publisher.publish(ans)
                    time.sleep(1)
                    vel_msg = Twist()
                    vel_msg.linear.x = 0.0
                    self.movement_publisher.publish(vel_msg)
                elif boxes[-1]['label'] == "crossing_sign" and self.passed_parking:
                    print("pedestrian")
                    ans = Int32()
                    self.signs['pedestrian'] = True
                    time.sleep(3)
                    ans.data = -1
                    self.check_publisher.publish(ans)
                elif boxes[-1]['label'] == 'tunnel_sign' and self.passed_pedestrian:
                    print("tunnel")
                    ans = Int32()
                    self.signs['tunnel'] = True
                    time.sleep(2)
                    ans.data = -1
                    self.check_publisher.publish(ans)
                plot_img = yolo_result.plot()
                image_np = np.array(plot_img)
                cv2.imshow('detect', image_np)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarGraphMaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
