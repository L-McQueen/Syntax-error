# Webots Sim2Real Maze Solver Setup Guide

This guide explains how to configure the Webots environment and Robot PROTO to work with the generated python controllers.

## 1. Directory Structure

Ensure your project looks like this:
```
WebotsSim2Real/
├── controllers/
│   ├── maze_supervisor/
│   │   └── maze_supervisor.py
│   └── robot_maze_solver/
│       └── robot_maze_solver.py
└── worlds/
    └── sim2real_maze.wbt
```

## 2. World File (`sim2real_maze.wbt`) Setup

1. Open Webots and create a new project directory or new world.
2. Add a `TexturedBackground` and `TexturedBackgroundLight`.
3. Add a `Robot` node and change its `controller` to `maze_supervisor`. Set its `supervisor` field to `TRUE`.
4. Ensure the `maze_supervisor` has its `children` field exposed (or it can just use the root children field as the script currently does with `self.getRoot().getField('children')`).
5. **DO NOT** add walls or obstacles manually. The supervisor will spawn them on reset.

## 3. Robot Design (Sim2RealRobot)

You must build a differential drive robot (or create a PROTO) with the following specific configuration to match the controller logic:

### A. Chassis & Physics
- **Dimensions:** 0.15m x 0.15m x 0.15m.
- **Center of Mass (CoM):** In the `Physics` node of the main chassis, set `centerOfMass` to `[0, -0.05, 0.05]` (shifted low and towards the front wheels). This prevents flipping on the 15-degree ramps.

### B. Drivetrain & Friction
- **Front Wheels:** Two standard `HingeJoint` driven wheels (names: `left wheel motor`, `right wheel motor`). Ensure their `ContactProperties` or `Material` has high static/dynamic friction (e.g., rubber).
- **Rear Support (The Skis):** Do not use rear caster wheels. Add two curved `Shape` nodes (like sled runners) at the rear bottom.
- **Ultra-low Friction:** In the `WorldInfo` node, add a `ContactProperties` node defining the interaction between the "SkiMaterial" and the floor. Set `coulombFriction` to `[0.01]`. Assign the `SkiMaterial` to the rear skis.

### C. Sensors
- **DistanceSensors (ToF):** Add three `DistanceSensor` nodes pointing Left, Front, and Right. Name them `tof_left`, `tof_front`, and `tof_right`. Set their `type` to `infra-red` and `maxRange` to 2.0m.
- **IMU:** Add an `InertialUnit` named `imu`.
- **Accelerometer:** Add an `Accelerometer` named `accelerometer`.

### D. The Shaky Camera (HingeJoint)
To achieve the physical camera vibration when hitting speedbumps:
1. Instead of attaching the `Camera` node directly to the chassis, add a `HingeJoint`.
2. Set the `jointParameters` axis to `[1, 0, 0]` (Pitch).
3. Add a `Spring` and `Damping` field to the `jointParameters`:
   - `springConstant`: 10 N/m
   - `dampingConstant`: 1 Ns/m
4. Place the `Camera` node inside the `endPoint` `Solid` of this `HingeJoint`.
5. Name the camera `camera`.

### E. OLED Display
- Add a `Display` node attached to the chassis (visible from the top if desired, though the python script draws to it regardless).
- Name it `oled_display`. Set `width` 128 and `height` 64.

## 4. Running the Simulation

1. Set the robot's `controller` to `robot_maze_solver`.
2. Hit the Play/Run button in Webots.
3. The supervisor will instantly spawn a new random 5x5 maze, randomize the wall materials (watch out for the pitch black ones!), and place the speedbumps.
4. The robot will begin its noisy, multithreaded DFS exploration.
