# Automated Extrinsic Calibration of Multiple 3D Sensors on a Mobile Base

This project performs extrinsic calibration for multiple sensors based on a known target geometry, position and rotation. The registration process is implemented using ICP (Iterative Closest Point) and other transformation methods to align point clouds accurately. This README provides an overview of the setup, configuration, and available scripts.

## Project Overview

The project registers point clouds from different 3D sensors and applies extrinsic calibration. This can be done either using a known target as ground truth, or (but less preferred) overlaps the point cloud with an already calibrated point cloud.

### Main Features
- **Extrinsic Calibration with Target**: Uses a known target model for calibration, aligning sensor point clouds to the target.
- **Point Cloud Overlap**: Provides an option to register sensor point clouds to each other, without using the target model.
- **Flexible Configuration**: YAML configuration files (`sensors.yaml`, `target.yaml`, `overlap.yaml`) allow easy setup of sensors, target specifications, and registration parameters.

## Configuration Files

All optional keys of the yaml file, can be commented out.

### `sensors.yaml`

Defines each sensor's parameters and configuration:
- **Topics**: ROS topics from which point clouds are published.
- **Parent Frame**: The parent frame for each sensor where the point cloud should be calibrated towards
- **Cropping** (optional but preferred): Boundaries for point cloud cropping in terms of min and max coordinates.
- **Initial Guess** (optional): Approximate translation and rotation (roll, pitch, yaw in radians) to help guide the registration process if needed.
- **ransac** (optional): Perform global registration with the RANSAC algorithm

Make sure to not use the initial guess and ransac at the same time!

```yaml
sensorname:
  topic: /sensor/points
  parent: parentframe
  color: pointcloudcolor
  crop: # wrt the base_footprint frame
    min:
      x: 0
      y: -1
      z: 0
    max:
      x: 2
      y: 1
      z: 2
  guess: 
    x: 1
    y: 1
    z: 1
    roll: 1
    pitch: 1
    yaw: 1
  ransac:
    voxelsize: 0.01
```

### `target.yaml`

Specifies the known target's configuration, so it sets the ground truth:
- **Filepath**: Path to the target model file.
- **Scaling Factor**: Scaling applied to the model.
- **Cropping** (optional): Boundaries for mesh cropping in order to only have a pointcloud of the visible region.
- **Transformation**: Translation and rotation applied to align the model with the ground truth.
- **Points**: Number of points to sample to make a point cloud.

```yaml
filepath: path/to/stl
scaling_factor: 0.001
crop: # origin is at minima
  min:
    x: -1
    y: -1
    z: -1
  max:
    x: 1
    y: 1
    z: 1
translate:  # without transformation origin is at minima
  x: 0
  y: 0
  z: 0
rotate_degrees:
  roll: 0
  pitch: 0
  yaw: 0
points: 10000
```

### `overlap.yaml`

Sets the source and target sensors for point cloud overlap calibration.
The target is the 'ground truth' or in this case the already calibrated sensor, the source is the sensor that needs the calibration.
Both sensors should be available in the sensors.yaml file.

```yaml
target: calibrated-sensor
source: to-calibrate-sensor
```

## Scripts

1. **`adding_noise.py`**: Adds noise to the transformation, useful for testing and validating calibration robustness.

2. **`extract_data.py`**: Extracts point cloud data from the robot sensors as defined in `sensors.yaml`.

3. **`icp_pointclouds.py`**: Implements the ICP method for point cloud registration between source and target clouds.

4. **`main_overlap.py`**: Overlaps two sensor point clouds, as defined in `overlap.yaml`.

5. **`main.py`**: Performs extrinsic calibration with the known target by aligning sensor point clouds.

6. **`target.py`**: Generates a point cloud from the target model for registration as defined in the `target.yaml`.

## Example Usage

### Calibration with Known Target
To perform calibration with a known target, use `main.py`:

```bash
python main.py
```

When running the code first first all point clouds are visualised before calibration.

Then for each sensor:
- The initial guess is visualised with the target, this is done with the cropped point clouds. **Make sure that the target is visible in this cropped point cloud.**
- The full calibrated point cloud is visualised with the target. **Check if the targets overlap!** If the point clouds is too sparse or the initial guess is off, the algorithm could coverge to a local minima. If this happens, try it again or use the overlapping method with an already calibrated sensor.

At the end all calibrated point clouds are visualised.

The code will output the calibration input. This is the transformation from the parent frame to the child frame in x, y, z translation and roll, pitch, yaw rotations in radians. It also indicates the parent and child frame.

```terminal
Calibration input for the SENSORNAME which is the transform from PARENTFRAME to CHILDFRAME:
- translation [x y z]:       [0, 0, 0]
- rotation [roll pitch yaw]: [0, 0, 0]
```

### Point Cloud Overlap Calibration

To perform point cloud overlap calibration, use `main_overlap.py`:

```bash
python main_overlap.py
```

- The initial guess is visualised with the target, this is done with the cropped point clouds. **Make sure that the target is visible in this cropped point cloud.**
- The full calibrated point cloud is visualised with the target. **Check if the targets overlap!** If the point clouds is too sparse or the initial guess is off, the algorithm could coverge to a local minima. If this happens, try it again or use the overlapping method with an already calibrated sensor.

The code will output the calibration input. This is the transformation from the parent frame to the child frame in x, y, z translation and roll, pitch, yaw rotations in radians. It also indicates the parent and child frame.

```terminal
Calibration input for the SENSORNAME which is the transform from PARENTFRAME to CHILDFRAME:
- translation [x y z]:       [0, 0, 0]
- rotation [roll pitch yaw]: [0, 0, 0]
```

## Requirements

Ensure the following packages are installed. 
Packages are stated in `requirements.txt` and can be downloaded when running

```bash
pip install -r requirements.txt
```

Make sure that open3D version 0.18.0 is installed by running
```bash
open3d --version
```

If this is not (at least) the 0.18.0 version, open3d should be installed alternatively.
To do this go to [THIS LINK](https://www.open3d.org/docs/release/getting_started.html) to download the wheel.
Then install the wheel as follows

```bash
pip install --upgrade pip setuptools wheel
pip install PATH/TO/WHEEL
```

### Virtual environment
To use the virtual environment (needed to run the code on the PAL laptop), make sure that the virtual environment extension is installed.

```bash
sudo -i
apt install python3.8-venv
```

Initialise the virtual environment in Linux as 

```bash
cd path/to/your/project
python -m venv calibration-env
source calibration-env/bin/activate
pip install -r requirements.txt
```

or on Windows

```bash
cd path/to/your/project
python -m venv calibration-env
.\calibration-env\Scripts\activate
pip install -r requirements.txt
```

To deactivate the virtual environment use 
```bash
deactivate
```