# Planar-Robot-Path-Planning-in-C

This robot path planning program takes command line inputs of initial and final configurations (x,y,phi; press enter after each), total time, and number of steps for desired trajectory.
Computations are for constant velocity planar RRR robots only (velocity vector computations and Jacobian as well as matrix dimensions (if applicable) will have to change for other cases.

Joint angles and rates for each step are output to the command line 
(note: N is taken to be points starting from the first step through the last step and not including the initial configuration; i.e. with initial there would be N+1 points).
There is also a commented print command of x, y, phi position values if desired.

Unreachable points are indicated with "Pose Unreachable" preceding the output line.

The code could be compiled and run on this online C compiler: https://www.onlinegdb.com/online_c_compiler
(Visual studio C++ 2019 has deprecated the scanf function so if that is used wiill need to replace scanf with cin from the C++ iostream library).
