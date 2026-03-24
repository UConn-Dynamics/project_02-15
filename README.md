# **ME 5180: Advanced Dynamics | Project 02 | Group 15**

## Group Members:
[Christian DiPietrantonio](mailto:hwp25002@uconn.edu)

## Project Overview
![Dual slider kinematics project](https://raw.githubusercontent.com/cooperrc/me5180-project_02/refs/heads/main/dual-slider.svg)

In this project, a rigid bar is connected to two sliding pistons along
the diagonal tracks. As the pistons move along the tracks, the rigid bar rotates at a constant rate, $\dot{\theta}_3 = 2~rad/s$. The figure above has three _relative_ ccoordinate systems that move with the bodies:

1. $x_1-y_1-$ describes piston 1 position and orientation, $\theta_1$
2. $x_2-y_2-$ describes piston 2 position and orientation, $\theta_2$
3. $x_3-y_3-$ describes the rigid bar position and orientation, $\theta_3$

Each of the pistons are on tracks at $\pm 45^o$ and the rotating rigid
bar is 10 cm. The hinges are mounted to the center of the pistons
connecting the ends of the rigid bar. 
 
In this project, you need to 

1. determine constraint equations $C(\mathbf{q},~t)$
2. solve for the velocities, $\dot{q}$ and accelerations, $\ddot{q}$
3. visualize the motion of the system as the rigid bar goes through at least one full rotation

## Results
TBD

## Conclusions
TBD

## Derivation
TBD

## Reproducing Results
TBD

## Project Dependencies
TBD

## Project Structure
```
project_01-6/
|
├───── archive/ # old plots and files
|
├───── notes/   # hand calculations
|
├───── src/     # Source Code
|       |
|       ├───── ...
|
├───── main.jl                     # main driver
|
└───── README.md                   # project documentation
```