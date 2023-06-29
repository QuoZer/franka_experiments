# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import torch

from polymetis import RobotInterface


if __name__ == "__main__":
    # Initialize robot interface
    robot = RobotInterface(
        ip_address="localhost",
    )

    # Reset
    robot.go_home()

    # Get ee pose
    ee_pos, ee_quat = robot.get_ee_pose()
    print(f"Current ee position: {ee_pos}")
    print(f"Current ee orientation: {ee_quat}  (xyzw)")

    # Command robot to ee xyz position
    ee_pos_desired = torch.Tensor([0.4, -0.35, 0.2])
    print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
    state_log = robot.move_to_ee_pose(
        position=ee_pos_desired, orientation=None, #time_to_go=3.0
        op_space_interp=True
    )

    # Command robot to ee xyz position
    ee_pos_desired = torch.Tensor([0.4, 0.35, 0.2])
    print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
    state_log = robot.move_to_ee_pose(
       position=ee_pos_desired, orientation=None, #time_to_go=7.0,
       op_space_interp=True
    )

    # Command robot to ee xyz position
    ee_pos_desired = torch.Tensor([0.4, 0.35, 0.5])
    print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
    state_log = robot.move_to_ee_pose(
       position=ee_pos_desired, orientation=None, #time_to_go=10.0
       op_space_interp=True
    )

    # Command robot to ee xyz position
    ee_pos_desired = torch.Tensor([0.4, -0.35, 0.5])
    print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
    state_log = robot.move_to_ee_pose(
       position=ee_pos_desired, orientation=None, #time_to_go=10.0
       op_space_interp=True
    )

    # Get updated ee pose
    ee_pos, ee_quat = robot.get_ee_pose()
    print(f"New ee position: {ee_pos}")
    print(f"New ee orientation: {ee_quat}  (xyzw)")
