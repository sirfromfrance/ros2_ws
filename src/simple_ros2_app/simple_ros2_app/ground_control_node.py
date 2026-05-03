import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from simple_ros2_interfaces.srv import DronePosition

class GroundControlNode(Node):
    def __init__(self):
        super().__init__('ground_control_node')

        self.drones = ['/uav0', '/uav1', '/uav2']
        self.drones_position = [[0.0, 3.0, 0.0],
                                [10.0, 5.0, 0.0],
                                [7.0, 4.0, 0.0]]
        
        self.my_clients = []
        for i, drone in enumerate(self.drones):
            client = self.create_client(DronePosition, f'{drone}/companion_control/start_flight')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {drone}/companion_control/start_flight...')
            self.my_clients.append(client)
        
        self.mission_stage = 0
        
        self.start_time = self.get_clock().now()
        self.mission_timer = self.create_timer(1.0, self.mission_loop)

        
    def send_waypoint(self, index, x, y, z):
        req = DronePosition.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        self.my_clients[index].call_async(req)
        self.get_logger().info(f'Sent to UAV{index}: X:{x}, Y:{y}, Z:{z}')
        # ПРИБРАНО створення таймера та скидання часу звідси!

    
    def mission_loop(self):
        now = self.get_clock().now()
        time_from_start = (now - self.start_time).nanoseconds 

        if time_from_start < 15e9 and self.mission_stage == 0:
            self.get_logger().info('STAGE 1: TAKEOFF')
            for i in range(3):
                self.send_waypoint(i, self.drones_position[i][0], self.drones_position[i][1], 3.0)
            self.mission_stage = 1

        elif time_from_start > 15e9 and time_from_start < 45e9 and self.mission_stage == 1:
            self.get_logger().info('STAGE 2: MOVING FORWARD')
            for i in range(3):
                self.send_waypoint(i, self.drones_position[i][0] + 10.0, self.drones_position[i][1], 5.0)
            self.mission_stage = 2

        elif time_from_start > 45e9 and self.mission_stage == 2:
            self.get_logger().info('STAGE 3: LANDING')
            for i in range(3):
                self.send_waypoint(i, self.drones_position[i][0] + 10.0, self.drones_position[i][1], 2.0)
            self.mission_stage = 3
    
def main(args=None):
    rclpy.init(args=args)
    node = GroundControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()