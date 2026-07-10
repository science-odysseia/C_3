import rclpy
import json

from rclpy.node import Node
from std_msgs.msg import String

class CentersSubscriber(Node):
    def __init__(self):
        super().__init__('centers_subscriber')
        self.sub = self.create_subscription(String, 'centers', self.centers_callback, 10)

    def centers_callback(self, msg):
        self.get_logger().info('받은 원본 블록 리스트: "%s"' % msg.data)
        
        try:
            # json으로 들어온 날것의 리스트 변수로 저장
            blocks_raw_list = json.loads(msg.data)

            blocks_centers_queue = sorted(blocks_raw_list, key=lambda block: block[1][2])
            self.get_logger().info(f"성공적으로 정렬된 조립 큐: {blocks_centers_queue}, {type(blocks_centers_queue)}")

            tcp_queue = [[block_id, [x-0.5, y-0.5, z]] for block_id, (x, y, z) in blocks_centers_queue]
            self.get_logger().info(f"성공적으로 정렬된 TCP 큐: {tcp_queue}, {type(tcp_queue)}")
        except Exception as e:
            self.get_logger().error(f"데이터 파싱 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        centers_subscriber = CentersSubscriber()
        rclpy.spin(centers_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        centers_subscriber.destroy_node()

if __name__ == '__main__':
    main()