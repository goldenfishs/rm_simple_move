import rclpy
from rclpy.node import Node
from rm_msgs.msg import GoalPose

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(GoalPose, 'goal_pose', 10)
        self.goal_poses = [
            {'x': 0.0, 'y': 0.0, 'angle': 0.0, 'max_speed': 10.0, 'tolerance': 0.1, 'rotor': False},
            {'x': 2.8, 'y': -2.6, 'angle': 0.0, 'max_speed': 10.0, 'tolerance': 0.1, 'rotor': False},
            {'x': 4.2, 'y': -1.6, 'angle': 0.0, 'max_speed': 10.0, 'tolerance': 0.1, 'rotor': False},
            {'x': 4.2, 'y': -1.6, 'angle': 0.0, 'max_speed': 10.0, 'tolerance': 0.1, 'rotor': True},
        ]

    def publish_goal_pose(self, index):
        if index < 0 or index >= len(self.goal_poses):
            self.get_logger().error('Invalid index')
            return

        goal_pose = GoalPose()
        goal_pose.x = self.goal_poses[index]['x']
        goal_pose.y = self.goal_poses[index]['y']
        goal_pose.angle = self.goal_poses[index]['angle']
        goal_pose.max_speed = self.goal_poses[index]['max_speed']
        goal_pose.tolerance = self.goal_poses[index]['tolerance']
        goal_pose.rotor = self.goal_poses[index]['rotor']

        self.publisher_.publish(goal_pose)
        self.get_logger().info(f'Published goal pose: {goal_pose}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    try:
        while rclpy.ok():
            index = int(input('Enter the index of the goal pose to publish: '))
            node.publish_goal_pose(index)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()