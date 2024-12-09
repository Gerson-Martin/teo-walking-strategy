# teo-walking-strategy
The repository focuses on the TEO humanoid walk.

# Commands

## Stable Gait

To launch this repository, you need to use the following commands:

- `--distance` : The distance you want the robot to travel (**required**).
- `--dry` : Simulates the robot's movement without actual motion (**optional**).
- `--sensorRemote` : YARP port name of the Force Torque sensor (**required**).
- `--robotRemote` : YARP port name of the robot (**required**).
- `--sensorIMURemote` : YARP port name of the IMU sensor (**optional**).

### Example Commands

#### Simulation with the TEO Humanoid Robot
```bash
stableGait --distance 1 --dry --sensorRemote /teoSim/forceTorque --robotRemote /teoSim --sensorIMURemote /teoSim/imu
```
## One Foot Stand

To use the One Foot Stand functionality, you must specify which leg you are moving. This repository is used to test controllers. For the other leg, use the configuration file oneFootStand-leftLeg.ini.
```bash
oneFootStand --from oneFootStand-rightLeg.ini --sensorRemote /teoSim/forceTorque --robotRemote /teoSim
```
