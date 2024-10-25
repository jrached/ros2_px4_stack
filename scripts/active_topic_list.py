import rclpy 
from rclpy.node import Node

class TopicFilter(Node):
    def __init__(self):
        super().__init__('topic_filter')

        self.active_topics = self.get_active_topics()
        self.get_logger().info(f'Number of active topics: {len(self.active_topics)}')
        self.get_logger().info(f'Active topics: \n {self.active_topics}')


    def is_active(self, topic_name):
        publishers = self.get_publishers_info_by_topic(topic_name)
        return len(publishers) > 0


    def get_active_topics(self):
        all_topics = self.get_topic_names_and_types()
        return  [topic for topic, types in all_topics if self.is_active(topic)]


def main(args=None):
    rclpy.init(args=args)
    topic_filter = TopicFilter()
    try:
        rclpy.spin(topic_filter)
    except KeyboardInterrupt:
        topic_filter.get_logger().info('Keyboard Interrupt Received. Shutting down...')
    finally:
        topic_filter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()