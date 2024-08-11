# humanoid_wb_mpc

This repo is based on the work done in [wb_mpc_centauro](https://github.com/ADVRHumanoids/wb_mpc_centauro) to extend the MPC for use with humanoids like the REEM-C robot.

## Setup
The setup is the same as [wb_mpc_centauro](https://github.com/ADVRHumanoids/wb_mpc_centauro) with additional details described in the issue [here](https://github.com/ADVRHumanoids/wb_mpc_centauro/issues/1)

## Running the MPC
The MPC setup is currently only intended to be used with Rviz and can be run with one of the launch files:
- `roslaunch humanoid_wb_ros reemc_ddp.launch`
- `roslaunch humanoid_wb_ros reemc_sqp.launch`