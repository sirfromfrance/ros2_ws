import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow')
        self.drone_name = self.get_namespace()
        
        self.cv_bridge = CvBridge()
        self.drone_camera = self.create_subscription(
            Image, 
            f'{self.drone_name}/camera/image_raw', 
            self.frame_processing, 
            qos_profile_sensor_data)
            
        self.skip_frame = 5
        self.optical_flow_init = False
        self.prev_frame_gray = None
        self.color = np.random.randint(0, 255, (100, 3))
        
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.prev_features = None

    def frame_processing(self, msg):
        if self.skip_frame > 0:
            self.skip_frame -= 1
            return
            
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if not self.optical_flow_init:
                self.prev_features = cv2.goodFeaturesToTrack(cv_image_gray, mask=None, **self.feature_params)
                self.prev_frame_gray = cv_image_gray
                if self.prev_features is not None:
                    self.optical_flow_init = True
            else:
                features, st, err = cv2.calcOpticalFlowPyrLK(self.prev_frame_gray, cv_image_gray, self.prev_features, None, **self.lk_params)
                
                if features is not None:
                    good_new = features[st == 1]
                    good_old = self.prev_features[st == 1]
                    
                    for i, (new, old) in enumerate(zip(good_new, good_old)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        cv_image = cv2.line(cv_image, (int(a), int(b)), (int(c), int(d)), self.color[i].tolist(), 2)
                        cv_image = cv2.circle(cv_image, (int(a), int(b)), 5, self.color[i].tolist(), -1)
                    
                    cv2.imshow(f"Optical Flow {self.drone_name}", cv_image)
                    cv2.waitKey(1)
                    
                    self.prev_frame_gray = cv_image_gray.copy()
                    self.prev_features = good_new.reshape(-1, 1, 2)
        except Exception as e:
            self.get_logger().error(f"Помилка обробки відео: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()