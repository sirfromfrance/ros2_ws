#!/usr/bin/env python3
import sys
import os
import time

local_pip_path = os.path.expanduser('~/.local/lib/python3.10/site-packages')
if local_pip_path not in sys.path:
    sys.path.insert(1, local_pip_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mavros_msgs.msg import ManualControl
from mavros_msgs.srv import SetMode
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class AITrackerNode(Node):
    def __init__(self):
        super().__init__('ai_tracking_node')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/uav0/camera/image_raw', self.image_callback, 10)
        self.control_pub = self.create_publisher(ManualControl, '/uav0/mavros/manual_control/send', 10)
        self.set_mode_client = self.create_client(SetMode, '/uav0/mavros/set_mode')

        self.model = YOLO('yolov8n.pt')
        self.tracker = None
        self.tracking_active = False
        self.target_bbox = None
        
        self.ai_control_enabled = False  
        self.last_key_time = time.time()
        
        self.man_x, self.man_y, self.man_z, self.man_r = 0.0, 0.0, 0.0, 0.0
        self.BASE_THROTTLE = 500.0  
        self.MANUAL_SPEED = 400.0   
        
        self.TARGET_AREA = 6000
        self.AREA_TOLERANCE = 2500
        self.CENTER_TOLERANCE_X = 30
        self.CENTER_TOLERANCE_Y = 25
        self.K_YAW = 1.8
        self.K_X = 0.08
        self.K_Z = 0.8
        
        self.FRAME_WIDTH = 480
        self.FRAME_HEIGHT = 360
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        self.CONF_THRESHOLD = 0.35    
        self.MIN_BBOX_SIZE = 12       
        self.MAX_BBOX_SIZE = 400      

        self.control_timer = self.create_timer(0.05, self.publish_control)
        self.get_logger().info("Клікни на вікно! Керування: WASD + IJKL. 'T' для ШІ.")

    def switch_to_posctl(self):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = 'POSCTL'
            self.set_mode_client.call_async(req)
            self.get_logger().info("Перемкнуто в режим POSCTL")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        key = cv2.waitKey(1) & 0xFF
        
        if key != 255:  
            self.last_key_time = time.time()
            if key == ord('t'):
                self.ai_control_enabled = not self.ai_control_enabled
                status = "ON" if self.ai_control_enabled else "OFF. Control via keyboard"
                self.get_logger().info(f"AI control: {status}")
                if self.ai_control_enabled:
                    self.switch_to_posctl()
            elif not self.ai_control_enabled:
                if key == ord('w'): self.man_x = self.MANUAL_SPEED
                elif key == ord('s'): self.man_x = -self.MANUAL_SPEED
                elif key == ord('a'): self.man_y = self.MANUAL_SPEED
                elif key == ord('d'): self.man_y = -self.MANUAL_SPEED
                elif key == ord('i'): self.man_z = self.MANUAL_SPEED
                elif key == ord('k'): self.man_z = -self.MANUAL_SPEED
                elif key == ord('l'): self.man_r = -self.MANUAL_SPEED
                elif key == ord('j'): self.man_r = self.MANUAL_SPEED
        elif time.time() - self.last_key_time > 0.3:
            self.man_x, self.man_y, self.man_z, self.man_r = 0.0, 0.0, 0.0, 0.0

        if self.ai_control_enabled:
            yolo_fixed_drift = False

            if not self.tracking_active:
                results = self.model.predict(frame, classes=[4, 14], verbose=False)
                
                if len(results[0].boxes) > 0:
                    
                    for box in results[0].boxes:
                        conf = float(box.conf[0])
                        box_data = box.xywh[0].cpu().numpy()
                        w, h = int(box_data[2]), int(box_data[3])
                        x_m = int(box_data[0] - w / 2)
                        y_m = int(box_data[1] - h / 2)
                        
                        cv2.rectangle(frame, (x_m, y_m), (x_m+w, y_m+h), (0, 0, 255), 1)
                        cv2.putText(frame, f"Conf: {conf:.2f}", (x_m, y_m - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                    best_conf, best_box = 0.0, None
                    
                    for box in results[0].boxes:
                        if float(box.conf[0]) > best_conf:
                            best_conf = float(box.conf[0])
                            best_box = box

                    if best_conf > self.CONF_THRESHOLD and best_box is not None:
                        box_data = best_box.xywh[0].cpu().numpy()
                        w, h = int(box_data[2]), int(box_data[3])
                        x_min = int(box_data[0] - w / 2)
                        y_min = int(box_data[1] - h / 2)

                        if y_min >= 20 and (self.MIN_BBOX_SIZE <= h <= self.MAX_BBOX_SIZE) and (self.MIN_BBOX_SIZE <= w <= self.MAX_BBOX_SIZE):
                            self.target_bbox = (x_min, y_min, w, h)
                            
                            try: 
                                self.tracker = cv2.TrackerCSRT_create()
                            except AttributeError: 
                                self.tracker = cv2.legacy.TrackerCSRT_create()
                                
                            self.tracker.init(frame, self.target_bbox)
                            self.tracking_active = True
                            yolo_fixed_drift = True

            if self.tracking_active:
                if not yolo_fixed_drift:
                    success, bbox = self.tracker.update(frame)
                    if not success: 
                        self.tracking_active = False
                    else: 
                        self.target_bbox = bbox
                
                if self.tracking_active:
                    bbox = self.target_bbox
                    cv2.rectangle(frame,  (int(bbox[0]), int(bbox[1])), (int(bbox[0]+bbox[2]), int(bbox[1]+bbox[3])), (0, 255, 0), 2)
                    cv2.putText(frame, "TRACKING", (int(bbox[0]), int(bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            #cv2.putText(frame, "detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Drone", frame)

    def publish_control(self):
        cmd = ManualControl()
        if not self.ai_control_enabled:
            cmd.x, cmd.y, cmd.z, cmd.r = self.man_x, self.man_y, self.BASE_THROTTLE + self.man_z, self.man_r
            self.control_pub.publish(cmd)
            
        elif self.ai_control_enabled and self.tracking_active and self.target_bbox is not None:
            bbox = self.target_bbox
            obj_x, obj_y, obj_area = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2, bbox[2] * bbox[3]
            error_x, error_y = obj_x - self.CENTER_X, obj_y - self.CENTER_Y

            cmd.z = self.BASE_THROTTLE
            
            if abs(error_x) < self.CENTER_TOLERANCE_X: cmd.r = 0.0
            else: cmd.r = float(max(min(error_x * self.K_YAW, 400.0), -400.0))

            area_error = self.TARGET_AREA - obj_area
            if abs(area_error) < self.AREA_TOLERANCE: cmd.x = 0.0
            else: cmd.x = float(max(min(area_error * self.K_X, 800.0), -800.0))

            if abs(error_y) < self.CENTER_TOLERANCE_Y: 
                cmd.y = 0.0
            else:
                cmd.z = float(max(min(self.BASE_THROTTLE - error_y * self.K_Z, 700.0), 300.0))
                cmd.y = 0.0
                
            self.control_pub.publish(cmd)
            
        elif self.ai_control_enabled and not self.tracking_active:
            cmd.x, cmd.y, cmd.r, cmd.z = 0.0, 0.0, 0.0, self.BASE_THROTTLE
            self.control_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AITrackerNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__': 
    main()