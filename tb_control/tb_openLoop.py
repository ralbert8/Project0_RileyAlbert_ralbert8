import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class openLoop(Node):

    """
    A ROS2 node for open-loop velocity control of a turtlebot using a trapezoidal velocity profile.

    The robot will accelerate, move at a constant speed, and decelerate over fixed time/distance,
    and log its position from odometry data and plot.
    """


    
    def __init__(self):

        """
        Initializes the node, sets up publishers, subscribers, and timing, and configures movement
        profile.
        """

        # Inherit Node capabilities
        super().__init__('tb_openLoop')

        # Create velocity command publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 100)

        # Create odometry data subscriber
        self.create_subscription(Odometry, '/odom', self.odometry_cb, 100)

        # Create timer for calling the move function
        self.timer = self.create_timer(0.01, self.move)

        # Create twist message for velocity commands
        self.vel_msg = Twist()

        # Store position and time data for plotting
        self.positions = []
        self.times = []
        self.start_time = None
        self.current_position = 0.0

        # Trapezoidal velocity profile parameters
        self.total_distance = 5.0                      # Total distance to travel [m]
        self.total_time = 10.0                         # Total time of motion [s]
        self.acceleration_time = self.total_time / 4   # Time to accelerate [s]
        self.deceleration_time = self. total_time / 4  # Time to decelerate [s]


    
    def odometry_cb(self, data):

        """
        Callback for `/odom` subscription. Stores robot's current x-position over time.

        Parameters:
            data (nav_msgs.msg.Odometry): Odometry message containing robot pose.
        """
        
        # Extract x-position from odometry
        self.current_position = data.pose.pose.position.x

        if self.start_time is not None:
            
            # Compute elapsed time
            elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
            
            # Store position and time for plottting
            self.positions.append(self.current_position)
            self.times.append(elapsed_time)


    
    def move(self):

        """
        Controls robot motion using a trapezoidal velocity profile.
        Publishes velocity commands and stops the robot after the total time.
        """

        # Compute time at max velocity
        max_velocity_time = self.total_time / 2

        # Cpmpute the required max velocity
        max_velocity = (4 * self.total_distance) / (3 * self. total_time)

        # Compute elapsed time
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Determine velocity based on elapsed time and motion phase
        if elapsed_time < self.acceleration_time:

            # Acceleration phase
            velocity = max_velocity * (elapsed_time / self.acceleration_time)

        elif elapsed_time < self.acceleration_time + max_velocity_time:

            # Constant velocity phase
            velocity = max_velocity

        else:

            # Deceleration phase
            remaining_time = self.total_time - elapsed_time
            velocity = max_velocity * (remaining_time / self.deceleration_time) # [m/s]

        # Set linear x velocity and zero angular velocity
        self.vel_msg.linear.x = velocity
        self.vel_msg.angular.z = 0.0 

        # Publish velocity command
        self.velocity_publisher.publish(self.vel_msg)

        # Stop robot and plot if motion is complete
        if elapsed_time >= self.total_time:
            self.stop()
            self.plot_pose()
            rclpy.shutdown()


    
    def stop(self):

        """
        Stops the robot by sending zero velocity.
        """
        
        self.vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(self.vel_msg)


    
    def plot_pose(self):

        """
        Plots the recorded robot position vs. time.
        """

        plt.plot(self.times, self.positions)

        plt.xlim(0, self.total_time)
        plt.ylim(0, self.total_distance)

        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.title('Robot Pose vs. Time')
        plt.grid(True)
        plt.show()



def main(args=None):

    """
    Entry point of script. Initializes and spins ROS2 node.
    """
    
    rclpy.init(args=args)
    node = openLoop()
    node.start_time = node.get_clock().now().seconds_nanoseconds()[0]
    rclpy.spin(node)



if __name__ == '__main__':
    main()
