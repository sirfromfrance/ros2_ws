import rclpy
from rclpy.node import Node
from simple_ros2_interfaces.srv import DronePosition
from std_srvs.srv import Trigger

class GroundControlNode(Node):
    def __init__(self):
        super().__init__('ground_control_node')

        self.drones = ['uav0', 'uav1', 'uav2']
        
        self.drones_position = [[0.0, 0.0],
                                [2.0, 0.0],
                                [-2.0, 0.0]]
        
        self.nav_clients = []
        self.takeoff_clients = []
        
        for drone in self.drones:
            self.nav_clients.append(self.create_client(DronePosition, f'/{drone}/companion_control/start_flight'))
            self.takeoff_clients.append(self.create_client(Trigger, f'/{drone}/companion_control/master_takeoff'))
        
        self.mission_state = 0 # 0: Очікування, 1: Зліт, 2: Маршрут
        self.ready_time = None
        self.takeoff_time = None
        
        self.timer = self.create_timer(1.0, self.mission_loop)
        self.get_logger().info('Ground Control for 3 Drones Initialized. Waiting for Gazebo...')

    def mission_loop(self):
        now = self.get_clock().now()

        if self.mission_state == 0:
            all_services_ready = True
            for i in range(len(self.drones)):
                if not self.takeoff_clients[i].service_is_ready() or not self.nav_clients[i].service_is_ready():
                    all_services_ready = False
                    break
            
            if all_services_ready:
                if self.ready_time is None:
                    self.ready_time = now
                    self.get_logger().info('all drones are ready!')
                else:
                    elapsed_ready = (now - self.ready_time).nanoseconds / 1e9
                    if elapsed_ready > 10.0:
                        self.get_logger().info('starting takeoff')
                        for i in range(len(self.drones)):
                            self.takeoff_clients[i].call_async(Trigger.Request())
                        
                        self.takeoff_time = now
                        self.mission_state = 1

        elif self.mission_state == 1:
            elapsed_takeoff = (now - self.takeoff_time).nanoseconds / 1e9
            
            if elapsed_takeoff > 12.0:
                self.get_logger().info('sending waypoints')
                
                for i in range(len(self.drones)):
                    base_x = self.drones_position[i][0]
                    base_y = self.drones_position[i][1]                    
                    self.send_waypoint(i, base_x, base_y, 4.0)
                    self.send_waypoint(i, base_x + 5.0, base_y, 4.0)
                    self.send_waypoint(i, base_x + 500, base_y + 5.0, 5.0)
                    self.send_waypoint(i, base_x, base_y + 5.0, 4.0)
                    self.send_waypoint(i, base_x, base_y, 3.0)             
                    self.send_waypoint(i, base_x, base_y, 0.0)                                    
                self.get_logger().info('mission started')
                self.mission_state = 2
                self.timer.cancel()

    def send_waypoint(self, index, x, y, z):
        req = DronePosition.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        self.nav_clients[index].call_async(req)
        self.get_logger().info(f'UAV{index} -> WP: X:{x}, Y:{y}, Z:{z}')

def main(args=None):
    rclpy.init(args=args)
    node = GroundControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()