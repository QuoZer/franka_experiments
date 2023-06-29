# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import torch
import time

from polymetis import RobotInterface


if __name__ == "__main__":
    # Initialize robot interface
    robot = RobotInterface(
        ip_address="localhost",
    )

    rate = 100
    dt = 1/rate

    # Reset
    robot.go_home()

    # Cartesian impedance control
    print("Performing Cartesian impedance control...")
    ee_pos, ee_quat = robot.get_ee_pose()

    robot.start_cartesian_impedance()

    distance = 0.3   # m
    ex_time = 3      # s
    samples = ex_time*rate
    dy = distance/samples

    for i in range(ex_time*rate):
        ee_pos += torch.Tensor([0.0, -dy, 0.0])
        robot.update_desired_ee_pose(position=ee_pos)
        time.sleep(dt)

    for i in range(60):
        ee_pos += torch.Tensor([0.0, +0.01, 0.0])
        robot.update_desired_ee_pose(position=ee_pos)
        time.sleep(0.1)

    robot.terminate_current_policy()



   #  # Command robot to ee xyz position
   #  ee_pos_desired = torch.Tensor([0.4, -0.35, 0.2])
   #  print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
   #  state_log = robot.move_to_ee_pose(
   #      position=ee_pos_desired, orientation=None, #time_to_go=3.0
   #      op_space_interp=True
   #  )

   #  # Command robot to ee xyz position
   #  ee_pos_desired = torch.Tensor([0.4, 0.35, 0.2])
   #  print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
   #  state_log = robot.move_to_ee_pose(
   #     position=ee_pos_desired, orientation=None, #time_to_go=7.0,
   #     op_space_interp=True
   #  )

   #  # Command robot to ee xyz position
   #  ee_pos_desired = torch.Tensor([0.4, 0.35, 0.5])
   #  print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
   #  state_log = robot.move_to_ee_pose(
   #     position=ee_pos_desired, orientation=None, #time_to_go=10.0
   #     op_space_interp=True
   #  )

   #  # Command robot to ee xyz position
   #  ee_pos_desired = torch.Tensor([0.4, -0.35, 0.5])
   #  print(f"\nMoving ee pos to: {ee_pos_desired} ...\n")
   #  state_log = robot.move_to_ee_pose(
   #     position=ee_pos_desired, orientation=None, #time_to_go=10.0
   #     op_space_interp=True
   #  )

    # Get updated ee pose
    ee_pos, ee_quat = robot.get_ee_pose()
    print(f"New ee position: {ee_pos}")
    print(f"New ee orientation: {ee_quat}  (xyzw)")
