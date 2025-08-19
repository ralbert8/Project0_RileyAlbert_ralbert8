import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt



class openLoop(Node):

    """
    ROS2 node that performs open-loop control by driving a robot at a contstant velocity for a certain distance and time.
    Records and plots position vs. time.
    """


    
    def __init__(self):

        """
        Initializes ROS2 node, sets up publishers, subscribers, timers, and internal state.
        """

        # Inherit Node capabilities
        super().__init__('tb_openLoop')

        # Publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 100)

        # Subscriber to read odometry data
        self.create_subscription(Odometry, '/odom', self.odometry_cb, 100)

        # Timer to call move function
        self.timer = self.create_timer(0.01, self.move)

        # Twist message to store velocity command
        self.vel_msg = Twist()

        # Initialize data collection for plotting
        self.positions = []
        self.times = []
        self.start_time = None
        self.current_position = 0.0

        # Movement configuration
        self.total_distance = 5.0
        self.total_time = 10.0


    
    def odometry_cb(self, data):

        """
        Callback for odometry subscriber. Records x-position and elapsed time.

        Parameters:
            data (nav.msgs.msg.Odometry): Received odometry message.
        """
        
        # Extract x-position of robot
        self.current_position = data.pose.pose.position.x

        if self.start_time is not None:
            
            # Get elapsed time
            elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
            
            # Store pose and time information
            self.positions.append(self.current_position)
            self.times.append(elapsed_time)


    
    def move(self):

        """
        Sends constant velocity commands to the robot and stops after total time is reached.
        Triggers data plotting at end of run.
        """

        # Calculate constant velocity
        velocity = self.total_distance / self.total_time

        # Get elapsed time
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Set robot velocity
        self.vel_msg.linear.x = velocity
        self.vel_msg.angular.z = 0.0 

        # Publish velocity command
        self.velocity_publisher.publish(self.vel_msg)

        # Stop robot when time is reached
        if elapsed_time >= self.total_time:
            self.stop()
            self.plot_pose()
            rclpy.shutdown()


    
    def stop(self):

        """
        Stops robot by setting velocity to zero.
        """
        
        self.vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(self.vel_msg)


    
    def plot_pose(self):

        """
        Plots robot x-position over time using collected odometry data.
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
    Initializes ROS client library, created node, and spins.
    """
    
    rclpy.init(args=args)
    node = openLoop()
    node.start_time = node.get_clock().now().seconds_nanoseconds()[0]
    rclpy.spin(node)



if __name__ == '__main__':
    main()
