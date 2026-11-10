#!/usr/bin/env python3
"""Stand-in for Nav2's navigate_to_pose, for mission_edge_test.sh.

  python3 fake_nav_server.py <accept_delay_s>
The goal is accepted after accept_delay_s (simulates a busy/hung bt_navigator),
then "drives" forever until cancelled. Every event is printed for the test to grep.
"""
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeNav(Node):
    def __init__(self, delay):
        super().__init__('fake_nav')
        self.delay = delay
        ActionServer(self, NavigateToPose, 'navigate_to_pose',
                     execute_callback=self.execute,
                     goal_callback=self.on_goal,
                     cancel_callback=lambda _: self.say('CANCEL_REQUESTED') or CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())

    def say(self, what):
        print(f'{time.time():.3f} {what}', flush=True)

    def on_goal(self, _):
        self.say('GOAL_RECEIVED')
        time.sleep(self.delay)
        self.say('GOAL_ACCEPTED')
        return GoalResponse.ACCEPT

    def execute(self, handle):
        while rclpy.ok():
            if handle.is_cancel_requested:
                handle.canceled()
                self.say('GOAL_CANCELED')
                return NavigateToPose.Result()
            time.sleep(0.1)
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav(float(sys.argv[1]))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
