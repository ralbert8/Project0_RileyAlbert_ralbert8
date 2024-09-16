import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class openLoop(Node):
    # Initialize tb_openLoop Node
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
        self.deceleration_time = self. total_time / 4  # [s]

    # Define Odometry Callback Function for Storing Pose vs. Time Data
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

    # Define Move Function
    def move(self):

        max_velocity_time = self.total_time / 2 # [s]

        # Calculate the Max Velocity from Averaging Velocity over Accel and Decel Times
        max_velocity = (4 * self.total_distance) / (3 * self. total_time)

        # Start Timing for Simulation
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Accelerate Robot
        if elapsed_time < self.acceleration_time:
            velocity = max_velocity * (elapsed_time / self.acceleration_time) # [m/s]

        # Maintain Max Velocity
        elif elapsed_time < self.acceleration_time + max_velocity_time:
            velocity = max_velocity # [m/s]

        # Decelerate Robot
        else:
            remaining_time = self.total_time - elapsed_time
            velocity = max_velocity * (remaining_time / self.deceleration_time) # [m/s]

        # Set Linear Velocity in X Direction and Restrict Rotation Around Z
        self.vel_msg.linear.x = velocity
        self.vel_msg.angular.z = 0.0 

        # Publish Velocity Command
        self.velocity_publisher.publish(self.vel_msg)

        # Stop Robot and Plot Pose vs. Time at Simulation End
        if elapsed_time >= self.total_time:
            self.stop()
            self.plot_pose()
            rclpy.shutdown()

    # Define Function to Stop Robot
    def stop(self):
        # Set Linear Velocity in X Direction to Zero and Publish
        self.vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(self.vel_msg)

    # Define Function to Plot Pose vs. Time
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
