# Vex-Library
## Brief
A VEX V5 pro library for holonomic and west-coast drive bases with integrated trajectory planning, PID class, and controller functions.

The implementation details for holonomic trajectory and velocity profile generation are explained in the following presentation (Japanese)

[特別活動レポート公開版](https://drive.google.com/file/d/1s2wVjT6lOR31UDFTtDkMZKHKzBM_V8rS/view?usp=sharing)

## Desmos Graphs
The Desmos graph to easily generate waypoints can be accessed through the following links.

The normal **Path Planner** version, where a starting and end point and velocity are can be visualized.

[Path Planner](https://www.desmos.com/calculator/zqpztqvvoi)

The **Path Planner Plus** version, where a starting, mid, and end point and velocity can be visualized. 
Path Planner Plus is ideal for long or complex paths that require an extra point for optimized object avoidance.

[Path Planner Plus](https://www.desmos.com/calculator/dg7kybaxyb)

## StaticProfile

is declared as follows:

```C++
StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8}
```
The parameters are as follow:

- initial velocity - compensate static friction or play in shaft (0 to 1)
- final velocity   - ensure robot reaches final destination (0 to 1)
- acceleration     - the greater this number, the faster the robot accelerates (greater than 0)
- deceleration     - the greater this number, the faster the robot decelerates (greater than 0)
- max velocity     - limit the maximum velocity of the robot (0 to 1)

## Differential Paths

### Differential Linear Path

```C++
DifferentialTrajectory traj {
  10,                                            // distance to move in inches (negative for reverse direction)
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8}     // static velocity profile
}
```

### Differential Two-Point Path

```C++
DifferentialTrajectory traj {
  Path {                                         // a two point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  false                                          // run this path in reverse? (don't have to include if false)
}
```

### Differential Three-Point Path

```C++
DifferentialTrajectory traj {
  PathPlus {                                     // a three point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // middle position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0},                                // middle velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  false                                          // run this path in reverse? (don't have to include if false)
}
```

## Holonomic Paths

### Holonomic Linear Path

```C++
HolonomicTrajectory traj {
  Vector{0, 0},                                  // inches to move in the x and y direction
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8}     // static velocity profile
}
```

### Holonomic Two-Point Path

```C++
HolonomicTrajectory traj {
  Path {                                         // a two point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8}     // static velocity profile 
}
```

### Holonomic Three-Point Path

```C++
HolonomicTrajectory traj {
  PathPlus {                                     // a three point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // middle position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0},                                // middle velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8}     // static velocity profile 
}
```

## Adding Holonomic Poses

When you want the robot to face a direction that is different from the direction of travel, use the holonomic pose feature. 

For example, if you want the robot to drive an S-curve while picking up game pieces, you may have to make to robot face different directions at different points through a single curve.

The holonomic pose feature allows the user to map a desierd robot angle to a certain point in the path. The point in the path is a number that ranges from 0 to 1, and a value of 0.5 for instance, would signify halfway through the path. Define an std::vector of as many poses as desired, and pass it to the function. If you decide to use this feature, ensure that the std::vector you have defined has an angle defined at the endpoints. This means that when constructing a two point path, points 0 and 1 are mapped to an angle, and when constructing a three point path, points 0 and 2 are mapped to an angle. Once this condition is met, feel free to make as many angle assignments in between.

The following are examples of acceptable paths:

```C++
HolonomicTrajectory traj {
  Path {                                         // a two point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  std::vector<HolonomicPose> {                   // std::vector of HolonomicPose's
    HolonomicPose {0, 90},                       // face 90 degrees when the robot is at the initial point
    HolonomicPose {0.3, 180},                    // face 180 degrees at 3/10 through the initial and final point
    HolonomicPose {0.8, 300},                    // face 300 degrees at 8/10 through the initial and final point
    HolonomicPose {1, 90}                        // face 90 degrees when the robot is at the final point
  }  
}
```

```C++
HolonomicTrajectory traj {
  PathPlus {                                     // a three point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // middle position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0},                                // middle velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  std::vector<HolonomicPose> {                   // std::vector of HolonomicPose's
    HolonomicPose {0, 90},                       // face 90 degrees at point 0
    HolonomicPose {0.3, 180},                    // face 180 degrees at 3/10 through the initial and middle point
    HolonomicPose {1, 300},                      // face 300 degrees when the robot is at the middle point
    HolonomicPose {0.4, 180},                    // face 180 degrees at 4/10 through the middle and final point
    HolonomicPose {2, 90}                        // face 90 degrees when the robot is at the final point
  }  
}
```

The following are examples of unacceptable paths:

```C++
HolonomicTrajectory traj {
  Path {                                         // a two point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  std::vector<HolonomicPose> {                   // std::vector of HolonomicPose's
    HolonomicPose {0, 90},                       // face 90 degrees when the robot is at the initial point
    HolonomicPose {0.3, 180},                    // face 180 degrees at 3/10 through the initial and final point
    HolonomicPose {0.8, 300}                     // face 300 degrees at 8/10 through the initial and final point
                                                 // the robot's pose at the final point is not stated (error)
  }  
}
```

```C++
HolonomicTrajectory traj {
  PathPlus {                                     // a three point path
    Vector{0, 0},                                // initial position
    Vector{0, 0},                                // middle position
    Vector{0, 0},                                // final position
    Vector{0, 0},                                // initial velocity
    Vector{0, 0},                                // middle velocity
    Vector{0, 0}                                 // final velocity
  },                                             
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},    // static velocity profile 
  std::vector<HolonomicPose> {                   // std::vector of HolonomicPose's
                                                 // the robot's pose at the initial point is not stated (error)
    HolonomicPose {0.3, 180},                    // face 180 degrees at 3/10 through the initial and middle point
                                                 // the robot's pose at the middle point is not stated (this is ok)
    HolonomicPose {0.4, 180},                    // face 180 degrees at 4/10 through the middle and final point
    HolonomicPose {2, 90}                        // face 90 degrees when the robot is at the final point
  }  
}
```

## Running a Trajectory

first make an instance of a trajectory and a drive base. A drive base can be declared as follows:

```C++
    HolonomicDrive drive;
```

or:

```C++
    DifferentialDrive drive;
```

and executing the trajectory as follows:

```C++
    drive.setPose(traj.initialPose);
    while (drive.follow(traj) != 1) {
      drive.localize();
      wait(10, msec);
    }
```
