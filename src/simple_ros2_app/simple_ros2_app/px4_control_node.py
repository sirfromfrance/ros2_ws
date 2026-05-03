import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from geometry_msgs.msg import PoseStamped
from simple_ros2_interfaces.srv import DronePosition
from std_srvs.srv import Trigger

class Control_Node(Node):
    def __init__(self, drone_id='uav0'):
        super().__init__('companion_control')

        self.current_state = State()
        self.target_pose = PoseStamped()
        
        # We start with the target pose at 0, 0, 0 with a valid quaternion
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0
        self.target_pose.pose.orientation.w = 1.0
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 0.0

        # Create service for position waypoints
        self.create_service(
            DronePosition, 
            '~/start_flight',
            self.waypoint_service_cb
        )

        # Create simple master takeoff service
        self.create_service(Trigger, '~/master_takeoff', self.master_takeoff_cb)

        # MAVROS clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.param_client = self.create_client(ParamSet, 'mavros/param/set')

        # MAVROS subscriptions
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb, 
            qos_profile_sensor_data
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.local_pose_cb,
            qos_profile_sensor_data
        )
        
        # Publisher for local setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        self.current_pose = PoseStamped()
        self.pose_ready = False
        self.state_ready = False
        
        # State machine variables
        self.takeoff_requested = False
        self.takeoff_state = 0
        self.takeoff_start_time = None

        # Loop timer at 20Hz
        self.loop_timer = self.create_timer(0.05, self.timer_callback)

    def state_cb(self, msg):
        self.current_state = msg
        self.state_ready = True

    def local_pose_cb(self, msg):
        self.current_pose = msg
        self.pose_ready = True

    def waypoint_service_cb(self, request, response):
        self.target_pose.pose.position.x = request.x
        self.target_pose.pose.position.y = request.y
        self.target_pose.pose.position.z = request.z
        
        # Ensure we are in OFFBOARD mode for waypoints to work
        if self.current_state.mode != 'OFFBOARD':
            self.get_logger().info('Waypoint received but not in OFFBOARD mode! Switching to OFFBOARD...')
            mode_req = SetMode.Request()
            mode_req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(mode_req)
            
        response.success = True
        self.get_logger().info(f'New Target -> x:{request.x}, y:{request.y}, z:{request.z}')
        return response

    def master_takeoff_cb(self, request, response):
        self.get_logger().info('--- MASTER TAKEOFF SEQUENCE REQUESTED ---')
        self.get_logger().info(f'[Status] Connected: {self.current_state.connected}, Armed: {self.current_state.armed}, Mode: {self.current_state.mode}')
        
        self.get_logger().info('[Step 1] Initializing setpoints to current position and waiting for stream to establish...')
        if self.pose_ready:
            self.target_pose.pose.position.x = self.current_pose.pose.position.x
            self.target_pose.pose.position.y = self.current_pose.pose.position.y
            self.target_pose.pose.position.z = self.current_pose.pose.position.z
            self.target_pose.pose.orientation = self.current_pose.pose.orientation
        else:
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 0.0
            self.target_pose.pose.orientation.w = 1.0
            self.target_pose.pose.orientation.x = 0.0
            self.target_pose.pose.orientation.y = 0.0
            self.target_pose.pose.orientation.z = 0.0
            
        self.takeoff_requested = True
        self.takeoff_state = 0
        self.takeoff_start_time = self.get_clock().now()
        
        response.success = True
        response.message = 'Takeoff sequence initiated.'
        return response

    def timer_callback(self):
        # We ALWAYS publish the setpoints here to keep PX4 happy in OFFBOARD mode!
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map' 
        self.local_pos_pub.publish(self.target_pose)

        # Process the takeoff state machine non-blocking!
        if self.takeoff_requested and self.takeoff_start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.takeoff_start_time).nanoseconds / 1e9

            # Step 1.5: Disable RC checks to prevent failsafe
            if self.takeoff_state == 0 and elapsed > 0.5:
                self.get_logger().info('[Step 1.5] Disabling PX4 RC Checks (COM_RC_IN_MODE=1, NAV_RCL_ACT=0)...')
                
                req1 = ParamSet.Request()
                req1.param_id = 'COM_RC_IN_MODE'
                req1.value.integer = 1
                self.param_client.call_async(req1)
                
                req2 = ParamSet.Request()
                req2.param_id = 'NAV_RCL_ACT'
                req2.value.integer = 0
                self.param_client.call_async(req2)
                
                self.takeoff_state = 1

            # Wait 2 seconds for setpoints to stream, then ARM
            elif self.takeoff_state == 1 and elapsed > 2.0:
                self.get_logger().info('[Step 2] Sending arming command...')
                arm_req = CommandBool.Request()
                arm_req.value = True
                self.arming_client.call_async(arm_req)
                self.takeoff_state = 2

            # Wait another 1 second, then set OFFBOARD mode
            elif self.takeoff_state == 2 and elapsed > 3.0:
                self.get_logger().info('[Step 3] Setting OFFBOARD mode...')
                mode_req = SetMode.Request()
                mode_req.custom_mode = 'OFFBOARD'
                self.set_mode_client.call_async(mode_req)
                self.takeoff_state = 3
            
            # Wait another 1 second, then set altitude to 10m to take off
            elif self.takeoff_state == 3 and elapsed > 4.0:
                takeoff_height = self.target_pose.pose.position.z + 10.0
                self.get_logger().info(f'[Step 4] Taking off! Changing target setpoint altitude to {takeoff_height}m...')
                self.target_pose.pose.position.z = takeoff_height
                self.takeoff_state = 3
                self.takeoff_requested = False
                self.get_logger().info('--- MASTER TAKEOFF SEQUENCE COMPLETED SUCCESSFULLY ---')

def main(args=None):
    rclpy.init(args=args)
    node = Control_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
