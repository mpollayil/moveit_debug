import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import time

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Parameters
        self.publish_freq = 0.01

        # Subscriber to JointTrajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.joint_trajectory_callback,
            10
        )

        # Publisher to JointStates
        self.publisher = self.create_publisher(JointState, 'simulated_joint_states', 1)
        
        self.timer = self.create_timer(timer_period_sec=self.publish_freq, callback=self.timer_callback)
        
        self.js = JointState()

        # Publish initial states
        self.initial_publish()

    def joint_trajectory_callback(self, msg):

        if msg.joint_names != []:
            self.js.name = msg.joint_names
        
        start_time = 0.0

        for point in msg.points:
            exec_time = point.time_from_start.sec + 1e-9*point.time_from_start.nanosec
            time.sleep(exec_time - start_time)

            self.js.position = point.positions
            self.js.velocity = point.velocities
            self.js.effort = point.effort
            self.js.header.stamp = self.get_clock().now().to_msg()

            self.publisher.publish(self.js)
            
            start_time = exec_time
            
    def timer_callback(self):
        self.js.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.js)

    def initial_publish(self):
        # Publish states
        self.js.name = ['_joint' + str(i+1) for i in range(6)]
        self.js.position = [0.0] * 6
        self.js.velocity = [0.0] * 6
        self.js.effort = [0.0] * 6
        self.js.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(self.js)
               
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_subscriber = TrajectoryPublisher()
    rclpy.spin(joint_trajectory_subscriber)
    joint_trajectory_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()