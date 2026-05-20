import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped
from simple_ros2_interfaces.srv import DronePosition
from std_srvs.srv import Trigger

class Control_Node(Node):
    def __init__(self):
        super().__init__('companion_control')

        self.current_state = State()
        self.target_pose = PoseStamped()
        
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0
        self.target_pose.pose.orientation.w = 1.0

        self.waypoints = []
        self.current_wp_index = 0
        self.mission_active = False
        self.distance_threshold = 0.5

        self.create_service(DronePosition, '~/start_flight', self.waypoint_service_cb)
        self.create_service(Trigger, '~/master_takeoff', self.master_takeoff_cb)

        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.param_client = self.create_client(ParamSet, 'mavros/param/set')

        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile_sensor_data)
        self.local_pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.local_pose_cb, qos_profile_sensor_data)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        self.current_pose = PoseStamped()
        self.pose_ready = False
        
        self.takeoff_requested = False
        self.takeoff_state = 0
        self.takeoff_start_time = None
        self.auto_takeoff_initiated = False

        self.loop_timer = self.create_timer(0.05, self.timer_callback)

    def state_cb(self, msg):
        self.current_state = msg

    def local_pose_cb(self, msg):
        self.current_pose = msg
        self.pose_ready = True
        
        if self.mission_active:
            self.check_navigation_progress()

    def check_navigation_progress(self):
        if self.current_wp_index >= len(self.waypoints):
            return

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_z = self.current_pose.pose.position.z

        target = self.waypoints[self.current_wp_index]
        distance = math.sqrt((target['x'] - curr_x)**2 + (target['y'] - curr_y)**2 + (target['z'] - curr_z)**2)

        if distance < self.distance_threshold:
            self.get_logger().info(f'WP {self.current_wp_index} reached! (distance: {distance:.2f}m)')
            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints):
                next_target = self.waypoints[self.current_wp_index]
                self.target_pose.pose.position.x = next_target['x']
                self.target_pose.pose.position.y = next_target['y']
                self.target_pose.pose.position.z = next_target['z']
            else:
              
                self.mission_active = False
                self.get_logger().info('(AUTO.LAND)...')
                
               
                mode_req = SetMode.Request()
                mode_req.custom_mode = 'AUTO.LAND'
                
                
                self.set_mode_client.call_async(mode_req)

    def waypoint_service_cb(self, request, response):
        self.waypoints.append({'x': request.x, 'y': request.y, 'z': request.z})
        self.get_logger().info(f'Точку додано до маршруту: X:{request.x}, Y:{request.y}, Z:{request.z}')
        
        if not self.mission_active and self.current_wp_index < len(self.waypoints):
            self.target_pose.pose.position.x = self.waypoints[self.current_wp_index]['x']
            self.target_pose.pose.position.y = self.waypoints[self.current_wp_index]['y']
            self.target_pose.pose.position.z = self.waypoints[self.current_wp_index]['z']
            self.mission_active = True
            
            mode_req = SetMode.Request()
            mode_req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(mode_req)
            
        response.success = True
        return response
    
    def master_takeoff_cb(self, request, response):
        if not self.pose_ready:
            response.success = False
            response.message = 'No local pose.'
            return response

        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pose.pose.position.z
            
        self.takeoff_requested = True
        self.takeoff_state = 0
        self.takeoff_start_time = self.get_clock().now()
        
        response.success = True
        return response

    def timer_callback(self):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map' 
        self.local_pos_pub.publish(self.target_pose)

        if self.pose_ready and not self.auto_takeoff_initiated:
            self.target_pose.pose.position.x = self.current_pose.pose.position.x
            self.target_pose.pose.position.y = self.current_pose.pose.position.y
            self.target_pose.pose.position.z = self.current_pose.pose.position.z
            
            self.takeoff_requested = True
            self.takeoff_state = 0
            self.takeoff_start_time = self.get_clock().now()
            self.auto_takeoff_initiated = True
            self.get_logger().info('Автоматичний зліт після завантаження системи!')

        if self.takeoff_requested and self.takeoff_start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.takeoff_start_time).nanoseconds / 1e9

            if self.takeoff_state == 0 and elapsed > 0.5:
                req1, req2 = ParamSet.Request(), ParamSet.Request()
                req1.param_id, req1.value.integer = 'COM_RC_IN_MODE', 1
                req2.param_id, req2.value.integer = 'NAV_RCL_ACT', 0
                self.param_client.call_async(req1)
                self.param_client.call_async(req2)
                self.takeoff_state = 1

            elif self.takeoff_state == 1 and elapsed > 1.0:
                mode_req = SetMode.Request()
                mode_req.custom_mode = 'OFFBOARD'
                self.set_mode_client.call_async(mode_req)
                self.takeoff_state = 2

            elif self.takeoff_state == 2 and elapsed > 2.0:
                arm_req = CommandBool.Request()
                arm_req.value = True
                self.arming_client.call_async(arm_req)
                self.takeoff_state = 3
            
            elif self.takeoff_state == 3 and elapsed > 5.0:
                self.target_pose.pose.position.z = self.current_pose.pose.position.z + 1.0
                self.takeoff_state = 4
                self.takeoff_requested = False
                self.get_logger().info('Зліт ініційовано!')

def main(args=None):
    rclpy.init(args=args)
    node = Control_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()