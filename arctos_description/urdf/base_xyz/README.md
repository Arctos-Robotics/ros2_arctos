# arctos_description/urdf/base_xyz

This folder contains the URDF files for the base of the robot. The base is composed of the following parts:

**Joints**:
- `world_joint`: The joint that connects the world frame to the base link. (`world` to `base_link`) (fixed joint)
- `X_joint`: The joint that connects the base link to the first link of the robot. (`base_link` to `Link_1_1`)
- `Y_joint`: The joint that connects the first link of the robot to the second link. (`Link_1_1` to `Link_2_1`)
- `Z_joint`: The joint that connects the second link of the robot to the third link. (`Link_2_1` to `Link_3_1`)