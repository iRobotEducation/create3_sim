#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
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
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Run a node to republish a topic into a different topic name.
from pydoc import locate

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node


class TopicRepublisher(Node):
    """
    Node to republish topics.

    A Node to republish topic A (current_topic) to the new topic B (new_topic) both of which are
    specified by parameters.
    """

    def __init__(self):
        super().__init__('topic_republisher')
        self.current_topic = None
        self.new_topic = None
        self.timer = None
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('current_topic', ''),
                ('new_topic', ''),
            ])

        self.timer = self.create_timer(2.0, self.check_published_topic)

    def check_published_topic(self):
        """
        Check status of published topic.

        On every Timer timeout checks if the new_topic is advertised and if all parameters were set
        before creating subscription and publisher.
        """
        current_topic_subscriptions = self.get_subscriptions_info_by_topic(self.new_topic)
        if len(current_topic_subscriptions) == 0 or not (self.current_topic and self.new_topic):
            self.get_logger().debug('self.current_topic not yet ready for republish...')
        else:
            self.timer.destroy()
            self.init_pub_sub(current_topic_subscriptions[0].topic_type,
                              current_topic_subscriptions[0].qos_profile.history)

    def init_pub_sub(self, msg_type, qos):
        """
        Create the subscription to current_topic and the publisher to new_topic.

        msg_type: String representation of the message type.
        qos: The QoS profile history depth to apply to subscription and publisher.
        """
        self.get_logger().info(f'Republishing {self.current_topic} to {self.new_topic}.')

        # Convert type string to Python module format to extract class object
        msg_class = locate(msg_type.replace('/', '.'))

        self._ = self.create_subscription(
            msg_class,
            self.current_topic,
            self.listener_callback,
            qos)
        self.publisher = self.create_publisher(msg_class, self.new_topic, qos)

    def listener_callback(self, msg):
        """
        Subscription callback. Republish messages on current_topic to new_topic.

        msg: The message to republish.
        """
        self.publisher.publish(msg)

    def parameters_callback(self, params):
        """
        Parameters callback. For each parameter the corresponding variable is set.

        params: Array of parameters set.
        """
        for param in params:
            if param.name == 'current_topic':
                self.current_topic = param.value
            if param.name == 'new_topic':
                self.new_topic = param.value

        return SetParametersResult(successful=True)


def main(args=None):
    """
    Entrypoint for the TopicRepublisher.

    This tool can be used by simply passing the current_topic and new_topic parameters when
    executing the node. The republisher will wait until parameters are set and atleast one
    subscription to new_topic exists before republishing from current_topic.
    """
    rclpy.init(args=args)
    topic_republisher = TopicRepublisher()
    rclpy.spin(topic_republisher)
    topic_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
