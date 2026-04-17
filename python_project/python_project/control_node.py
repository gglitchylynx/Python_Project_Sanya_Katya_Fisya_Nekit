#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import time
import math

class AutonomousNavigationController(Node):
    """
    Основной контроллер автономного движения робота.
    Алгоритм следует по центру коридора и плавно избегает ям, не останавливаясь преждевременно.
    При обнаружении финиша робот следует к нему и НАДЕЖНО останавливается при потере из поля зрения.
    """
    
    def __init__(self):
        super().__init__('autonomous_navigation_controller')
        
        # Параметры робота
        self.robot_width = 0.6
        self.safe_margin = 0.15
        self.min_side_clearance = 0.4
        
        # Параметры движения
        self.max_speed = 1.0
        self.min_speed = 0.2
        self.max_angular = 2.0
        self.center_tolerance = 0.2
        
        # Зоны безопасности LiDAR
        self.front_warning_zone = 0.8
        self.front_critical_zone = 0.25
        
        # Параметры обнаружения ям
        self.pothole_detection_threshold = 0.012
        self.pothole_confirmation_frames = 2
        self.pothole_current_frames = 0
        self.pothole_confirmed = False
        self.pothole_distance_threshold = 1.5
        self.pothole_avoidance_gain = 0.4
        
        # Параметры обнаружения финиша
        self.finish_counter = 0
        self.finish_required = 8
        self.finish_confirmed = False
        self.finish_visible = False  
        self.finish_lost_time = 0.0  
        
        # Состояния контроллера
        self.state = "NORMAL"  
        self.state_start_time = time.time()
        self.target_turn_direction = None
        
        # Флаги завершения миссии
        self.mission_completed = False
        self.emergency_stop_issued = False
        
        # Внутренние переменные для обработки данных
        self.bridge = CvBridge()
        self.image = None
        self.lidar_ranges = None
        self.last_log_time = time.time()
        self.last_scan_time = time.time()
        
        # Подписка на топики сенсоров
        self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        
        # Издатель команд управления
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Таймер основного цикла управления
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Autonomous Navigation Controller initialized.")

    def camera_cb(self, msg):
        """Обработчик изображения с камеры."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def lidar_cb(self, msg):
        """Обработчик данных с LiDAR."""
        self.lidar_ranges = msg.ranges

    def detect_finish(self, img):
        """
        Детектирование зеленого финишного блока.
        Возвращает True если финиш обнаружен в текущем кадре.
        """
        if img is None:
            return False
        
        try:
            height = img.shape[0]
            upper_roi = img[:height//3, :]
            
            hsv = cv2.cvtColor(upper_roi, cv2.COLOR_BGR2HSV)
        
            green_lower = np.array([40, 50, 50])
            green_upper = np.array([80, 255, 255])
            
            mask = cv2.inRange(hsv, green_lower, green_upper)
        
    
            green_pixels = np.sum(mask > 0)
        
        # Если зеленых пикселей больше 1000 - финиш обнаружен
            return green_pixels > 1000
            
        except Exception as e:
            self.get_logger().error(f"Finish detection error: {e}")
            return False

    def detect_pothole(self, img):
        """
        Обнаружение ям на пути движения.
        Возвращает расстояние до ямы и её позицию для плавной коррекции.
        """
        if img is None or self.lidar_ranges is None:
            self.pothole_current_frames = max(0, self.pothole_current_frames - 1)
            return False, 0.0, "none"
        
        try:
            height, width = img.shape[:2]
            
            front_dist = self.get_front_distance() if hasattr(self, 'get_front_distance') else 2.0
            
            if front_dist < 0.6:
                roi_start = int(height * 0.8)
                roi_height = int(height * 0.15)
            elif front_dist < 1.2:
                roi_start = int(height * 0.7)
                roi_height = int(height * 0.2)
            else:
                roi_start = int(height * 0.6)
                roi_height = int(height * 0.25)
            
            roi_end = roi_start + roi_height
            lower_roi = img[roi_start:roi_end, :]
            
            if lower_roi.size == 0:
                return False, 0.0, "none"
            
            hsv = cv2.cvtColor(lower_roi, cv2.COLOR_BGR2HSV)
            white_lower = np.array([0, 0, 150])
            white_upper = np.array([180, 50, 255])
            white_mask = cv2.inRange(hsv, white_lower, white_upper)
            
            gray = cv2.cvtColor(lower_roi, cv2.COLOR_BGR2GRAY)
            _, bright_mask = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
            
            combined_mask = cv2.bitwise_or(white_mask, bright_mask)
            
            kernel_close = np.ones((11,11), np.uint8)
            kernel_open = np.ones((7,7), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel_close)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_open)
            
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            pothole_detected = False
            max_area = 0
            pothole_position = "none"
            pothole_distance = front_dist
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:
                    continue
                
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h if h > 0 else 0
                
                if (area > 250 and 
                    0.2 < aspect_ratio < 5.0 and 
                    y + h > roi_height * 0.5):
                    
                    if area > max_area:
                        max_area = area
                        contour_center_x = x + w/2
                        roi_center_x = lower_roi.shape[1] / 2
                        
                        if contour_center_x < roi_center_x * 0.6:
                            pothole_position = "left"
                        elif contour_center_x > roi_center_x * 1.4:
                            pothole_position = "right"
                        else:
                            pothole_position = "center"
                        
                        pothole_detected = True
            
            roi_area = lower_roi.shape[0] * lower_roi.shape[1]
            min_pothole_area = roi_area * self.pothole_detection_threshold
            
            if pothole_detected and max_area > min_pothole_area:
                self.pothole_current_frames += 1
                if self.pothole_current_frames >= self.pothole_confirmation_frames:
                    self.pothole_confirmed = True
                    return True, pothole_distance, pothole_position
            else:
                self.pothole_current_frames = max(0, self.pothole_current_frames - 1)
                if self.pothole_current_frames < self.pothole_confirmation_frames // 2:
                    self.pothole_confirmed = False
            
            return self.pothole_confirmed, pothole_distance, pothole_position
            
        except Exception as e:
            self.get_logger().error(f"Pothole detection error: {e}")
            self.pothole_current_frames = 0
            self.pothole_confirmed = False
            return False, 0.0, "none"

    def get_sector_distance(self, start_deg, end_deg):
        """Получение минимального расстояния в заданном секторе."""
        if self.lidar_ranges is None:
            return float('inf')
        
        n = len(self.lidar_ranges)
        min_angle = -1.5708
        max_angle = 1.5708
        angle_range = max_angle - min_angle

        start_rad = np.deg2rad(start_deg)
        end_rad = np.deg2rad(end_deg)
        
        start_idx = int(n * (start_rad - min_angle) / angle_range)
        end_idx = int(n * (end_rad - min_angle) / angle_range)
        start_idx = max(0, min(n-1, start_idx))
        end_idx = max(0, min(n-1, end_idx))

        if start_idx >= end_idx:
            return float('inf')
            
        sector = self.lidar_ranges[start_idx:end_idx]
        valid = [r for r in sector if np.isfinite(r) and r > 0.01]
        
        if not valid:
            return float('inf')
        
        return np.median(valid)

    def get_front_distance(self):
        """Минимальное расстояние в узком переднем секторе (±15°)"""
        return self.get_sector_distance(-15, 15)
    
    def get_front_wide_distance(self):
        """Минимальное расстояние в широком переднем секторе (±30°)"""
        return self.get_sector_distance(-30, 30)
    
    def get_side_distances(self):
        """Расстояния до боковых стен"""
        left_dist = self.get_sector_distance(-90, -60)
        right_dist = self.get_sector_distance(60, 90)
        return left_dist, right_dist
    
    def is_in_dead_end(self):
        """Проверка тупиковой ситуации."""
        front_dist = self.get_front_distance()
        left_dist, right_dist = self.get_side_distances()
        
        return (front_dist < self.front_critical_zone and
                left_dist < 0.8 and right_dist < 0.8 and
                min(left_dist, right_dist) < 0.4)

    def follow_center_line_with_pothole_avoidance(self, pothole_info):
        """
        Следование по центру коридора с плавным избеганием ям.
        Ямы обрабатываются как локальные препятствия без полной остановки.
        """
        cmd = Twist()
        
        if self.is_in_dead_end():
            self.state = "APPROACHING"
            self.state_start_time = time.time()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd, "Dead end detected. Initiating avoidance maneuver."
        
        left_dist, right_dist = self.get_side_distances()
        front_wide = self.get_front_wide_distance()
        
        center_error = left_dist - right_dist
        angular = 0.0
        
        if abs(center_error) > self.center_tolerance:
            gain = 0.5
            angular = -gain * center_error
            angular = max(-self.max_angular, min(self.max_angular, angular))
        
        pothole_detected, pothole_distance, pothole_position = pothole_info
        
        if pothole_detected and pothole_distance < self.pothole_distance_threshold:
            avoidance_angular = 0.0
            
            if pothole_position == "left":
                avoidance_angular = 0.3 * self.pothole_avoidance_gain
            elif pothole_position == "right":
                avoidance_angular = -0.3 * self.pothole_avoidance_gain
            else:  # center
                if left_dist > right_dist:
                    avoidance_angular = 0.4 * self.pothole_avoidance_gain
                else:
                    avoidance_angular = -0.4 * self.pothole_avoidance_gain
            
            angular = angular + avoidance_angular
            angular = max(-self.max_angular, min(self.max_angular, angular))
            
            speed_reduction = 0.2 if abs(avoidance_angular) > 0.3 else 0.1
            linear_speed = max(self.min_speed, self.max_speed * (1.0 - speed_reduction))
        else:
            linear_speed = self.max_speed
        
        if front_wide < self.front_warning_zone:
            if front_wide < self.front_critical_zone:
                linear_speed = 0.0
                angular = 0.0
                self.state = "APPROACHING"
                self.state_start_time = time.time()
            else:
                speed_factor = (front_wide - self.front_critical_zone) / (self.front_warning_zone - self.front_critical_zone)
                linear_speed = max(self.min_speed, self.max_speed * speed_factor)
        
        cmd.linear.x = linear_speed
        cmd.angular.z = angular
        
        status = f"Following center line. Left: {left_dist:.2f}m, Right: {right_dist:.2f}m"
        if pothole_detected:
            status += f" | Avoiding pothole ({pothole_position}) at {pothole_distance:.2f}m"
        
        return cmd, status

    def approaching_obstacle(self):
        """
        Плавное замедление и анализ препятствия.
        """
        cmd = Twist()
        front_wide = self.get_front_wide_distance()
        
        if front_wide > self.front_warning_zone * 1.3:
            self.state = "NORMAL"
            return self.follow_center_line_with_pothole_avoidance((False, 0.0, "none"))
        
        if front_wide < self.front_critical_zone * 1.5:
            cmd.linear.x = 0.0
            
            left_dist = self.get_sector_distance(-90, -30)
            right_dist = self.get_sector_distance(30, 90)
            
            if left_dist > right_dist:
                self.target_turn_direction = "left"
                cmd.angular.z = 0.4
            else:
                self.target_turn_direction = "right"
                cmd.angular.z = -0.4
            
            return cmd, f"Obstacle very close. Turning {self.target_turn_direction}."
        
        speed_factor = (front_wide - self.front_critical_zone) / (self.front_warning_zone - self.front_critical_zone)
        linear_speed = max(self.min_speed, self.max_speed * speed_factor * 0.7)
        cmd.linear.x = linear_speed
        cmd.angular.z = 0.0
        
        return cmd, f"Approaching obstacle. Slowing to {linear_speed:.2f}m/s. Distance: {front_wide:.2f}m"

    def turning(self):
        """
        Плавный поворот для обхода препятствий.
        """
        cmd = Twist()
        current_time = time.time()
        
        if current_time - self.state_start_time > 3.0:
            self.state = "NORMAL"
            return self.follow_center_line_with_pothole_avoidance((False, 0.0, "none"))
        
        front_dist = self.get_front_distance()
        turn_speed = 0.3 + (0.5 * (1.0 - min(1.0, front_dist / 1.0)))
        
        if self.target_turn_direction == "left":
            cmd.angular.z = turn_speed
        else:
            cmd.angular.z = -turn_speed
        
        cmd.linear.x = 0.15
        
        if front_dist > self.front_warning_zone * 1.2:
            left_dist, right_dist = self.get_side_distances()
            center_error = left_dist - right_dist
            
            if abs(center_error) < self.center_tolerance * 1.5:
                self.state = "NORMAL"
                return self.follow_center_line_with_pothole_avoidance((False, 0.0, "none"))
        
        direction = "left" if self.target_turn_direction == "left" else "right"
        return cmd, f"Turning {direction} at {cmd.angular.z:.2f}rad/s. Front: {front_dist:.2f}m"

    def finish_approach(self):
        """
        Режим приближения к финишу с остановкой при потере из поля зрения.
        """
        cmd = Twist()
        current_time = time.time()
        
        # Проверяем, виден ли финиш в текущем кадре
        current_finish_visible = self.detect_finish(self.image)
        
        if current_finish_visible:
            self.finish_visible = True
            self.finish_lost_time = 0.0
            
            # Движемся к финишу с умеренной скоростью
            cmd.linear.x = self.max_speed * 0.6
            cmd.angular.z = 0.0
            
            elapsed = current_time - self.state_start_time
            return cmd, f"Approaching finish line. Visible. Elapsed: {elapsed:.1f}s"
        else:
            # Финиш не виден в текущем кадре
            if self.finish_visible:
                # Финиш был виден, но сейчас пропал - НЕМЕДЛЕННО ОСТАНАВЛИВАЕМСЯ
                self.mission_completed = True
                elapsed = current_time - self.state_start_time
                self.get_logger().info(f"MISSION COMPLETED. Finish line lost from view after {elapsed:.2f} seconds.")
                
                # Отправляем команду остановки СРАЗУ здесь для надежности
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                return stop_cmd, "MISSION COMPLETED. Robot stopped."
            
            # Финиш никогда не был виден - продолжаем поиск с осторожностью
            cmd.linear.x = self.max_speed * 0.3
            cmd.angular.z = 0.0
            return cmd, "Finish not visible. Continuing cautious search."
        
        return cmd, "Approaching finish line (processing)"

    def control_loop(self):
        """
        Основной цикл управления с остановкой при завершении миссии.
        """
        current_time = time.time()
        
        # если миссия завершена, немедленно останавливаемся
        if self.mission_completed:
            if not self.emergency_stop_issued:
                # Отправляем команду остановки несколько раз для надежности
                stop_cmd = Twist()
                for _ in range(15):
                    self.cmd_pub.publish(stop_cmd)
                    time.sleep(0.03)
                self.emergency_stop_issued = True
                self.get_logger().info("EMERGENCY STOP ISSUED. Robot should be stationary.")
            
            # Продолжаем отправлять команды остановки для гарантии
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return
        
        # Аварийная проверка данных LiDAR
        if self.lidar_ranges is not None:
            self.last_scan_time = current_time
        elif current_time - self.last_scan_time > 1.0:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            if current_time - self.last_log_time >= 1.0:
                self.get_logger().warn("LIDAR data lost. Emergency stop activated.")
            return
        
        # Обнаружение финиша
        if self.detect_finish(self.image):
            self.get_logger().info(" FINISH DETECTED! Stopping robot immediately.")
    
            
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
    
            
            rclpy.shutdown()
            return
        
        # Обработка финишного режима
        if self.state == "FINISH_APPROACH":
            cmd, state_msg = self.finish_approach()
            
            # Если миссия завершена в finish_approach(), следующая итерация остановит робота
            if self.mission_completed:
                return
            
            self.cmd_pub.publish(cmd)
            return
        
        # Обнаружение ям
        pothole_info = self.detect_pothole(self.image)
        
        # Основной цикл управления
        if self.state == "NORMAL":
            cmd, state_msg = self.follow_center_line_with_pothole_avoidance(pothole_info)
            
            front_dist = self.get_front_distance()
            if front_dist < self.front_critical_zone * 1.2:
                self.state = "APPROACHING"
                self.state_start_time = current_time
        
        elif self.state == "APPROACHING":
            cmd, state_msg = self.approaching_obstacle()
            
            front_wide = self.get_front_wide_distance()
            if front_wide > self.front_warning_zone:
                self.state = "NORMAL"
        
        elif self.state == "TURNING":
            cmd, state_msg = self.turning()
            
            front_dist = self.get_front_distance()
            if front_dist > self.front_warning_zone * 1.2:
                self.state = "NORMAL"
        
        else:
            cmd = Twist()
            state_msg = "Unknown state. Emergency stop."
        
        self.cmd_pub.publish(cmd)
        
        # Логирование с интервалом 0.5 секунды
        if current_time - self.last_log_time >= 0.5:
            status = f"[{self.state}] {state_msg}"
            
            if self.lidar_ranges is not None:
                front_dist = self.get_front_distance()
                left_dist, right_dist = self.get_side_distances()
                status += f" | F={front_dist:.2f}m, L={left_dist:.2f}m, R={right_dist:.2f}m"
            
            pothole_detected, pothole_distance, pothole_position = pothole_info
            if pothole_detected:
                status += f" | POTHOLE: {pothole_position} at {pothole_distance:.2f}m"
            
            if self.state == "FINISH_APPROACH":
                status += f" | FINISH: {'visible' if self.finish_visible else 'lost'}"
            
            self.get_logger().info(status)
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(20):
            node.cmd_pub.publish(stop)
            time.sleep(0.05)
        
        node.destroy_node()
        rclpy.shutdown()
        print("Navigation system terminated. Robot is stationary.")

if __name__ == '__main__':
    main()