import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from simple_ros2_interfaces.srv import DronePosition

class SwarmControlNode(Node):
    def __init__(self):
        super().__init__('swarm_control')
        self.drone_name = self.get_namespace()
        self.position_msg = None
        self.drone_async_call = None
        self.swarm_position_sub = self.create_subscription(
            PoseStamped,
            '/swarm/broadcast/position',
            self.position_callback,
            10
        )

        self.swarm_position_publish = self.create_publisher(
            PoseStamped,
            '/swarm/broadcast/position',
            10
        )

        self.drone_service = self.create_client(
            DronePosition,
            'companion_control/start_flight'
        )

        self.timer = self.create_timer(1.0, self.broadcast_callback)
        self.get_logger().info(f'Вузол ройового керування для {self.drone_name} запущено.')

    def position_callback(self, msg):
        if self.position_msg is None or msg.header.stamp.sec > self.position_msg.header.stamp.sec:
            self.position_msg = msg
            self.get_logger().info(
                f'Отримано нову команду рою: x={msg.pose.position.x}, y={msg.pose.position.y}'
            )
            self.update_drone_flight_position()

    def broadcast_callback(self):
        if self.position_msg is not None:
            self.swarm_position_publish.publish(self.position_msg)

    def update_drone_flight_position(self):
        req = DronePosition.Request()
        req.x = self.position_msg.pose.position.x
        req.y = self.position_msg.pose.position.y
        req.z = self.position_msg.pose.position.z
        
        if 'uav1' in self.drone_name:
            req.x += 2.0
            req.y -= 2.0
        elif 'uav2' in self.drone_name:
            req.x -= 2.0
            req.y -= 2.0
            
        self.drone_async_call = self.drone_service.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()