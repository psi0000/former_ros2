import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros
from tf2_ros import Buffer
from tf2_ros import TransformListener
import numpy as np

class LaserScanTransformer(Node):
    def __init__(self):
        super().__init__('laser_scan_tms')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        


        # LaserScan 구독자 및 발행자 생성 
        original_laser_scan_topic='/scan' # TODO
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            original_laser_scan_topic,
            self.scan_callback,
            10
        )
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/tms_scan',
            10
        )

    def scan_callback(self, msg):

        #  frame set 
        lidar_frame = 'laser_link'  # TODO
        target_frame = 'base_link'  # TODO

        try:
            
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                lidar_frame,
                rclpy.time.Time().to_msg(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_scan = self.transform_laserscan(msg, transform_stamped)

            # 변환된 LaserScan 메시지 발행
            self.scan_publisher.publish(transformed_scan)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF transformation failed: {e}")

    def transform_laserscan(self, scan_msg, transform_stamped):
        # LaserScan 메시지의 각도 및 거리 데이터 가져오기
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        # TransformStamped 메시지로부터 변환 매트릭스 계산
        rotation = transform_stamped.transform.rotation
        translation = transform_stamped.transform.translation

        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = tf2_ros.transformations.quaternion_matrix(q)

        translation_vector = np.array([translation.x, translation.y, translation.z])
        transformed_ranges = ranges.copy()

        # 좌표 변환 적용
        for i in range(len(angles)):
            point = np.array([ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i]), 0.0])
            transformed_point = np.dot(rotation_matrix, point) + translation_vector
            transformed_ranges[i] = np.linalg.norm(transformed_point[:2])

        # 변환된 LaserScan 메시지 생성
        transformed_scan_msg = LaserScan()
        transformed_scan_msg.header = scan_msg.header
        transformed_scan_msg.angle_min = scan_msg.angle_min
        transformed_scan_msg.angle_max = scan_msg.angle_max
        transformed_scan_msg.angle_increment = scan_msg.angle_increment
        transformed_scan_msg.range_min = scan_msg.range_min
        transformed_scan_msg.range_max = scan_msg.range_max
        transformed_scan_msg.ranges = transformed_ranges.tolist()
        transformed_scan_msg.intensities = scan_msg.intensities

        return transformed_scan_msg

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()