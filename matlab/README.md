# MATLAB WRO Simulation Report

## Table of Contents
- [Problem Statement](#problem-statement)
- [Project Objective](#project-objective)
- [Key Features of the Model](#key-features-of-the-model)
- [Simulation Architecture](#simulation-architecture)
  - [Arena Layer](#arena-layer)
  - [Robot Layer](#robot-layer)
  - [Navigation Layer](#navigation-layer)
  - [Camera Layer](#camera-layer)
- [MATLAB Simulation Screenshots](#matlab-simulation-screenshots)
- [MATLAB Code](#matlab-code)
- [Methodology](#methodology)
  - [Stage 1 – Arena Geometry](#stage-1--arena-geometry)
  - [Stage 2 – Texture Mapping](#stage-2--texture-mapping)
  - [Stage 3 – Robot Modeling](#stage-3--robot-modeling)
  - [Stage 4 – Path and Waypoints](#stage-4--path-and-waypoints)
  - [Stage 5 – Steering Physics](#stage-5--steering-physics)
  - [Stage 6 – Wiggle Noise](#stage-6--wiggle-noise)
  - [Stage 7 – Camera Simulation](#stage-7--camera-simulation)
- [Conclusion](#conclusion)

---

## Problem Statement

Testing robot navigation directly on the physical WRO arena is slow, inconsistent, and prone to human and hardware error. During development, small mistakes—such as incorrect turning radius, heading overshoot, or path drift—consume valuable time and may damage motors or sensors. 

To solve this, our objective was to build a realistic digital twin of the WRO Open Challenge arena using MATLAB. This virtual environment allows us to test:

- Robot movement
- Steering and heading corrections
- Cornering behavior
- Wiggle / vibration effects
- Camera perspectives
- Strategy around the inner square

—all without touching the physical robot.

## Project Objective

The purpose of this MATLAB module is to:

1. Simulate the WRO 2025 arena accurately in 3D and 2D.
2. Replicate the robot's movement using realistic physics (car-like steering).
3. Provide two essential debug views:
   - A front chase camera (like a real robot camera)
   - A top-down map showing path, walls, and waypoints
4. Visualize the WRO arena mat using the original PNG/JPEG texture.
5. Test path-following logic around the central square.
6. Add noise/wiggle to replicate real hardware behaviour.
7. Allow rapid iteration before deploying code to the actual robot.

## Key Features of the Model

- ✔ Accurate 3×3 m WRO arena
- ✔ Real arena texture mapped using wro.jpeg
- ✔ 3D walls + 2D outlines
- ✔ Robot rendered as a 3D cube
- ✔ Waypoint navigation around the central square
- ✔ Car-like steering (limited turning rate)
- ✔ Adjustable speed, turning gain, wiggle amount
- ✔ Persistent front camera perspective
- ✔ Smooth animation using MATLAB's rendering loop

## Simulation Architecture

### Arena Layer

- Outer walls (3D & 2D)
- Inner square wall (3D & 2D)
- Arena floor textured with the official WRO mat

### Robot Layer

- 3D cube robot body
- Red dot in top view
- Position updated each time step
- Heading updated based on steering angle
- Wiggle noise added to simulate vibration

### Navigation Layer

- Four waypoints around the inner square
- Robot follows path anticlockwise
- When near a waypoint → switches to next
- Steering direction calculated using angle error

### Camera Layer

- Virtual chase camera follows the robot
- Looks ahead toward a future point
- Has adjustable FOV, height, and offset

## MATLAB Simulation Screenshots

### Front Camera View

**Figure 1** – Robot front-camera / chase-view simulation in MATLAB.

![Front Camera View](./matlab_report/images/page_4_image_1.png)

### Official WRO Mat Texture

Official WRO mat used as texture in simulation.

![WRO Mat Texture](./matlab_report/images/page_5_image_1.png)

## MATLAB Code

[MATLAB code available - Click here to view the code](https://github.com/Devansh-awat/greenbotics/blob/main/matlab/open_challenge.m)

## Methodology

### Stage 1 – Arena Geometry

The arena dimensions and walls were defined in meters to match WRO specifications. Cuboid meshes were used to create walls, ensuring they render correctly in the 3D view.

### Stage 2 – Texture Mapping

The official WRO mat was loaded and scaled per meter. This converts pixel coordinates into real-world coordinates.

### Stage 3 – Robot Modeling

The robot was modeled as a simple rigid cube. Orientation changes are handled using a rotation matrix.

### Stage 4 – Path and Waypoints

A rectangle around the inner square was chosen as the baseline path.

### Stage 5 – Steering Physics

Instead of teleporting or rotating instantly, the robot:

- Slowly adjusts heading
- Limits turn rate (max 140° per second)
- Moves at 0.12 m/s

This models a real car-like system.

### Stage 6 – Wiggle Noise

Random Gaussian noise simulates motor vibration, wheel imbalance, or surface friction.

### Stage 7 – Camera Simulation

A dynamic camera was positioned behind the robot and updated every frame.

## Conclusion

The developed MATLAB simulation successfully models the WRO 2025 arena, robot kinematics, and camera perspectives in a realistic and structured way. By providing visual debugging, real mat texture, and steering physics, the system acts as a complete digital twin for testing strategies before deploying to the physical robot. This reduces trial-and-error, saves time during practice, and significantly improves reliability and planning.
