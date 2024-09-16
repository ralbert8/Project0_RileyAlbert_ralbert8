import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

class openLoop(Node):
    def __init__(self):
        super().__init__('tb_openLoop')

        # Velocity Command Publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 100)

        # Odometry Data Subscriber
        self.create_subscription(Odometry, '/odom', self.odometry_cb, 100)

        # Define Rate for Publishing Velocity Commands
        self.timer = self.create_timer(0.01, self.move)

        # Create a Twist Message to Store Velocity Commands
        self.vel_msg = Twist()

        # Initialize Pose vs. Time Data
        self.positions = []
        self.times = []
        self.start_time = None
        self.current_position = 0.0

        # Define Variables for Movement Using a (1/4, 1/2, 1/4) Trapezoidal Velocity Profile
        self.total_distance = 5.0  # [m]
        self.total_time = 10.0  # [s]
        self.acceleration_time = self.total_time / 4  # [s]
        self.deceleration_time = self.total_time / 4  # [s]

        # Define Variables for Turning
        self.turn_time = 2.0  # [s]
        self.turning = False
        self.turn_start_time = None

    def odometry_cb(self, data):
        # Gathering One-Dimensional Pose Data in X Direction
        self.current_position = data.pose.pose.position.x

        if self.start_time is not None:
            # Get Elapsed Simulation Time
            elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
            
            # Append Current Pose Data
            self.positions.append(self.current_position)

            # Append Current Time Data
            self.times.append(elapsed_time)

    def move(self):
        # Calculate elapsed time
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        max_velocity_time = self.total_time / 2  # [s]
        max_velocity = (4 * self.total_distance) / (3 * self.total_time)  # Max velocity

        if not self.turning:
            # Linear motion: Accelerate, cruise, decelerate
            if elapsed_time < self.acceleration_time:
                velocity = max_velocity * (elapsed_time / self.acceleration_time)
            elif elapsed_time < self.acceleration_time + max_velocity_time:
                velocity = max_velocity
            elif elapsed_time < self.total_time:
                remaining_time = self.total_time - elapsed_time
                velocity = max_velocity * (remaining_time / self.deceleration_time)
            else:
                # Stop linear motion and initiate turn
                self.stop()
                self.initiate_turn()
                return

            # Set linear velocity and restrict rotation
            self.vel_msg.linear.x = velocity
            self.vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.vel_msg)

        else:
            # Perform Turn
            turn_elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.turn_start_time

            if turn_elapsed_time < self.turn_time:
                angle_deg = -90.0
                angle_rad = (angle_deg * math.pi) / 180
                angular_velocity = angle_rad / self.turn_time  # [rad/s]
                self.vel_msg.linear.x = 0.0  # [m/s]
                self.vel_msg.angular.z = angular_velocity
            else:
                # Stop after turn
                self.stop()
                rclpy.shutdown()

            self.velocity_publisher.publish(self.vel_msg)

    def initiate_turn(self):
        self.turning = True
        self.turn_start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def stop(self):
        # Stop robot motion
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)

    def plot_pose(self):
        # Plot Time on X and Pose on Y
        plt.plot(self.times, self.positions)

        # Set Axis Bounds Using Total Time and Distance
        plt.xlim(0, self.total_time)  # Time from 0 to total_time seconds
        plt.ylim(0, self.total_distance)  # Position from 0 to total_distance meters

        # Set Plot Visuals
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.title('Robot Pose vs. Time')
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = openLoop()
    node.start_time = node.get_clock().now().seconds_nanoseconds()[0]
    rclpy.spin(node)

if __name__ == '__main__':
    main()
