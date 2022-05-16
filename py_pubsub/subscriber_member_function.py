# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self): # 퍼블리셔의 내용과 거의 동일
        super().__init__('minimal_subscriber')
        #self.publisher_ = self.create_publisher    (String, 'topic', 10) 
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
                                                    # 퍼블리셔와 다르게     callback이 여기 있음
        self.subscription  # prevent unused variable warning

    # 퍼블리셔의 callback과는 사뭇 다름
    # 퍼블리셔와 다르게 timer의 정의가 필요하지 않음
    # 메시지를 수신하는 즉시 콜백이 호출됨
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    # 참고! 아래는 퍼블리셔의 callback 함수
    # def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i   # msg의 data 안에 Hello World와 self.i(timer callback이 호출된 횟수를 집어 넣음)
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)   # 퍼블리싱된 내용을 콘솔에 게시함
        # self.i += 1


# 퍼블리셔와 거의 동일한 구조
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
