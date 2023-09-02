# ros2 imports
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import SetBool

from uf_interfaces.msg import Position


class VehiclePositionChecker(Node):
    def __init__(self):
        super().__init__('vehicle_position_checker_node')

        # max health - will die if it reaches zero
        self.declare_parameter('health', 100)
        self.declare_parameter('deduction_const', 5.0)
        self.declare_parameter('range', 1.0)
        self.declare_parameter('vehicle_id', 'alpha')
        self.declare_parameter('rate', 1/10)

        # Creates a dictionary to store the positions
        self.vehicle_positions = {}

        # Creates a publisher for the bool distance response
        self.publisher_ = self.create_publisher(SetBool, 'health_status', 10)

        # Subscribes to the vehicle position topics
        # These topics should publish Position messages with the ID of the vehicle in the header.frame_id field
        self.create_subscription(PoseStamped, 'pose', self.current_position_callback, 10)
        self.create_subscription(Position, '/ground/agent/pose', self.vehicle_position_callback, 10)

        # Clock
        # self.rate = 1/10
        self.timer_ = self.create_timer(self.rate, self.check_positions)

    def vehicle_position_callback(self, msg):
        # Called when a new position message is received for a vehicle
        vehicle_id = msg.header.frame_id
        position = msg.point

        # Stores the position in a dictionary
        self.vehicle_positions[vehicle_id] = position

    def current_position_callback(self, msg):
        # Called when a new position message is received for a vehicle
        vehicle_id = msg.header.frame_id
        position = msg.pose.position

        # Stores the position in a dictionary
        self.vehicle_positions[vehicle_id] = position

    def check_positions(self):
        vehicles_within_range = False
        # Loops through all pairs of vehicles
        vehicle1_pos = self.vehicle_positions[self.vehicle_id]
        for vehicle2_id, vehicle2_pos in self.vehicle_positions.items():
            if self.vehicle_id != vehicle2_id:
                # Calculates the distance between the positions
                distance = ((vehicle1_pos.x - vehicle2_pos.x)**2 +
                            (vehicle1_pos.y - vehicle2_pos.y)**2)**0.5
                if distance < range:
                    # If the distance is less than above, changes the return value
                    vehicles_within_range = True
                    break
            if vehicles_within_range:
                break

        # Reduce health if vehicle_within_range
        self.health -= self.rate*self.deduction_const

        # Publishes the bool
        msg = SetBool()
        if self.health < 0:
            msg.data = 0
        else:
            msg.data = 1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    health_node = VehiclePositionChecker()
    rclpy.spin(health_node)
    health_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
