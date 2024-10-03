#!/usr/bin/env python
#
# Copyright (C) 2023 Inria

import numpy as np

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError:
    print("This example loads robot descriptions from robot_descriptions.py:")
    print("\n\tpip install robot_descriptions")

print("Goal: load a legged robot from its URDF, then modify leg lengths")

print("Loading robot description from URDF...")
robot = load_robot_description("upkie_description")
model = robot.model

# Joint placement offsets compared to the leg length
# We know from the robot description that both are along the local y-axis
known_offsets = {"knee": 0.072, "wheel": 0.065}


def check_limb_lengths(limb_length: float) -> bool:
    print(f"Checking that limbs are {limb_length} m long... ", end="")
    for side in ("left", "right"):
        for joint in ("knee", "wheel"):
            joint_id = model.getJointId(f"{side}_{joint}")
            if not np.allclose(
                model.jointPlacements[joint_id].translation[1],
                limb_length - known_offsets[joint],
            ):
                print("{side}_{joint} placement is wrong!")
                return False
    return True


# We know from the description that the leg length should be 24 cm
if check_limb_lengths(0.24):
    print("OK, the model is as we expect it")


def update_limb_lengths(length: float) -> None:
    """Update femur and tibia lengths in the robot model.

    Args:
        length: New femur and tibia length, in meters.
    """
    for side in ("left", "right"):
        for joint in ("knee", "wheel"):
            joint_id = model.getJointId(f"{side}_{joint}")
            model.jointPlacements[joint_id].translation[1] = (
                length - known_offsets[joint]
            )


update_limb_lengths(0.3)  # update femurs and tibias to 30 cm
if check_limb_lengths(0.3):
    print("OK, the update worked!")
