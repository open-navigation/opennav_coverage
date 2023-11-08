#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
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

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.append(self.toPolygon(field))

        print('Navigating to with field of size: ' + str(len(field)) + '...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return


def main():
    rclpy.init()

    navigator = CoverageNavigatorTester()
    navigator.startup()

    # Some example field
    field = [[5.0, 5.0], [5.0, 15.0], [15.0, 15.0], [10.0, 5.0], [5.0, 5.0]]
    navigator.navigateCoverage(field)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
