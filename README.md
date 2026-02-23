# UAV-Path-Planning

## Introduction

The objective of this project is to design an intelligent algorithm for unmanned aerial vehicles (UAVs) that enables them to find a path with minimal total steering angle and relative altitude fluctuation, subject to specified dynamic constraints, obstacle avoidance requirements, and start-end point specifications. The proposed algorithm primarily incorporates principles from the Particle Swarm Optimisation (PSO) and Grey Wolf Optimisation (GWO) algorithms.

## Project Structure

```text
├── README.md                      # README
├── LICENSE                        # MIT
├── UAV_Path_Planning.pdf          # Final Report
├── src/                           # Source Code
   └── ...
```

## Getting Started

Simply run MATLAB files NMOPSO_PathPlanner_ver2.m and MOGWO_PathPlanner_ver2.m directly from src. As the obstacles are user-defined, no dataset downloads are required. The obstacle positions and height ranges can be adjusted within CreateModel.m.

## Results

Please refer to the instructions in the UAV_Path_Planning for the results.

## Credits & Acknowledgements

For the MOPSO section, the core optimization logic in this project is based on the SPSO implementation by **Thi Thuy Ngan Duong**. 

As required by the original license:
* The SPSO algorithm is based on the source code by **Yarpiz** (www.yarpiz.com) and the **SPSO** repository (https://github.com/duongpm/SPSO.git).
* The original copyright notice and disclaimer are preserved in the respective source files.

Our team modified certain hyperparameters and successfully reproduced the results. Furthermore, we revised the obstacle design by incorporating height ranges and correspondingly adjusted the form of the obstacle avoidance loss function, enabling the drone to plan routes with greater flexibility.

For the MOGWO section, we have taken Seyedali Mirjalili's work into consideration （https://www.sciencedirect.com/science/article/pii/S0965997813001853）. Our team has proposed an improved method enabling the grey wolf algorithm to be applied for optimising multi-objective problems.

## Contributers

This Completed with Qiran Xian and Chenxi Ji, Zhejiang University. Three contributions of the three authors of the project are equal.
