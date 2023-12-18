import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LaserTfNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.tms_scan_publisher = self.create_publisher(
            LaserScan,
            '/tms_scan',
            10
        )

    def scan_callback(self, msg):
        transformed_scan = msg
        transformed_scan.ranges.reverse()
        transformed_scan.angle_min, transformed_scan.angle_max = \
                -transformed_scan.angle_max, -transformed_scan.angle_min
        self.tms_scan_publisher.publish(transformed_scan)
        # try:
        #     # `/scan` 데이터의 timestamp에 해당하는 변환 정보를 얻어옴
        #     transform = self.tf_buffer.lookup_transform(
        #         'base_link',  # 변환의 목적 프레임
        #         msg.header.frame_id,  # 현재 프레임
        #         msg.header.stamp,  # 변환을 수행할 timestamp
        #         rclpy.duration.Duration(seconds=1/15)  # 타임아웃 설정
        #     )

        #     temp_ranges = msg.ranges
        #     temp_ranges.reverse()

        #     # 변환된 `/scan` 데이터 생성
        #     transformed_scan = LaserScan()
        #     transformed_scan.header.frame_id = 'base_link'
        #     transformed_scan.header.stamp = msg.header.stamp
        #     transformed_scan.angle_min = msg.angle_min
        #     transformed_scan.angle_max = msg.angle_max
        #     transformed_scan.angle_increment = msg.angle_increment
        #     transformed_scan.time_increment = msg.time_increment
        #     transformed_scan.scan_time = msg.scan_time
        #     transformed_scan.range_min = msg.range_min
        #     transformed_scan.range_max = msg.range_max
        #     transformed_scan.ranges = msg.ranges
        #     transformed_scan.intensities = msg.intensities
        #     transformed_scan.ranges = temp_ranges
        #     # 변환된 `/scan` 데이터 발행
        #     self.tms_scan_publisher.publish(transformed_scan)

        # except tf2_ros.LookupException as ex:
        #     self.get_logger().warn(f"TF LookupException: {ex}")
        # except tf2_ros.ExtrapolationException as ex:
        #     self.get_logger().warn(f"TF ExtrapolationException: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
