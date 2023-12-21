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

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import ChangeState, GetState
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate  # Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None

        self.coverage_client = ActionClient(self, ComputeCoveragePath,
                                            'compute_coverage_path')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def getCoveragePath(self, goal_msg):
        """Send a `ComputeCoveragePath` action request."""
        # self.debug("Waiting for 'ComputeCoveragePath' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            # self.info("'ComputeCoveragePath' action server not available, waiting...")
            pass

        # self.info(f'Computing coverage path....')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            # self.error('Get coverage path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            # self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result

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

    def startup(self):
        """Activate coverage server."""
        change_state_client = self.create_client(
               GetState, 'row_coverage_server/get_state')
        req = GetState.Request()
        fut = change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result().current_state.id == 3:
            return

        change_state_client = self.create_client(
               ChangeState, 'row_coverage_server/change_state')
        req = ChangeState.Request()
        req.transition.id = 1
        fut = change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        req.transition.id = 3
        fut = change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)


def toMsg(val):
    coord = Coordinate()
    coord.axis1 = val[0]
    coord.axis2 = val[1]
    return coord


def main():
    rclpy.init()

    tester = CoverageTester()
    tester.startup()

    goal = ComputeCoveragePath.Goal()

    # Populate different setting combinations
    goal.generate_headland = True
    goal.generate_route = True
    goal.generate_path = True

    goal.use_gml_file = True
    goal.gml_field = get_package_share_directory('opennav_coverage') + '/irregular_test_field.xml'

    # goal.row_swath_mode.mode = "CENTER" # OFFSET
    # goal.row_swath_mode.offset = 0.5

    # goal.route_mode.mode = "SPIRAL" # BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM
    # goal.route_mode.spiral_n = 3
    # goal.route_mode.custom_order

    # goal.path_mode.mode = "DUBIN" # DUBIN, REEDS_SHEPP
    # goal.path_mode.continuity_mode = "CONTINUOUS" # CONTINUOUS, DISCONTINUOUS
    # goal.path_mode.turn_point_distance = 0.1

    coverage_info = tester.getCoveragePath(goal)
    result = tester.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got coverage path!')
    elif result == TaskResult.CANCELED:
        print('Coverage path request was canceled!')
    elif result == TaskResult.FAILED:
        print('Failed to obtain coverage path!')
    else:
        print('Goal has an invalid return status!')

    if coverage_info is not None:
        print(f'Coverage path has {len(coverage_info.coverage_path.swaths)} swaths')
        print(f'Coverage path has {len(coverage_info.coverage_path.turns)} turns')
        print(f'Coverage path has {len(coverage_info.nav_path.poses)} nav path poses')

    print('exiting')
    exit(0)


if __name__ == '__main__':
    main()
