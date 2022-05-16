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

import rclpy    # import Ros Client PYthon
from rclpy.node import Node     #? Ros Client PYthon node에서 node 추가

from std_msgs.msg import String #? Standart message.message에서 String 추가


class MinimalPublisher(Node):   # Class 추가

    def __init__(self):
        super().__init__('minimal_publisher')   # super().__init__('minimal_publisher') : node class의 생성자를 호출하고 node 이름을 정의
        self.publisher_ = self.create_publisher(String, 'topic', 10)    # publisher의 msg type을 String으로 선언, 토픽의 이름을 'topic'으로 선언, 큐 size를 10으로 설정
                                                                        # 큐 size는 subscriber가 msg를 충분히 빨리 수신하지 못하는 경우 대기 중인 메시지의 양을 제한함
        timer_period = 0.5  # seconds 타이머가 0.5sec 마다 실행할 callback과 함께 timer가 생성됨
        self.timer = self.create_timer(timer_period, self.timer_callback)   #? 타이머 생성
        self.i = 0  #? "self.i"는 counter에 사용되는 callback 입니다.

    # timer_callbakc은 counter 값이 추가된 msg를 만들고 'get_logger()'를 사용해 콘솔에 게시함
    def timer_callback(self):
        msg = String()  #?
        msg.data = 'Hello World: %d' % self.i   # msg의 data 안에 Hello World와 self.i(timer callback이 호출된 횟수를 집어 넣음)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)   # 퍼블리싱된 내용을 콘솔에 게시함
        self.i += 1     #? 그냥 퍼블리셔의 메시지에 넣기 위한 것인지 아니면 큐 size와 연관이 있는지  큐 size와 상관 없는 듯


def main(args=None):
    rclpy.init(args=args)   # rclpy 라이브러리 초기화

    minimal_publisher = MinimalPublisher()  # node 생성

    rclpy.spin(minimal_publisher)   # node를 회전시켜 callback을 호출함

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
