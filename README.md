# Kidnapped Vehicle Localization with Particle Filter

## Project Introduction

The vehicle has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its 
initial location, and lots of (noisy) sensor and control data.

In this project I will implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial 
localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

### Demo: Vehicle Localization

![](https://github.com/Luzhongyue/Kidnapped-Vehicle/blob/master/Images/process.png)

* The blue car is the ground truth of the vehicle, including position and heading orientation. 
* The blue circle (with an black arrow inside) is the real-time estimation of the vehicle's location and heading orientation from the 
particle filter

## Code & Files

**1. Running the code**
Once you have this repository on your machine, cd into the repository's root directory and run the following commands from the command 
line:

`<./clean.sh>`     
`<./build.sh>`    
`<./run.sh>`

**2.My Files**
* Images: the folder contains images  
* data : the folder contains data, including map, control and observation data
* src : the folder contains the source code
* CMakeLists.txt : the cmake file
* clean.sh : cleans the project
* build.sh : builds the project
* run.sh : runs the project

If everything worked you should see something like the following output:

![](https://github.com/Luzhongyue/Kidnapped-Vehicle/blob/master/Images/result.png)

## Data

### Map

**map_data.txt**includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns

1.x position    
2.y position    
3.landmark id

### Control Data

**control_data.txt** contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two 
columns represent

1.vehicle speed (in meters per second)     
2.vehicle yaw rate (in radians per second)

### Obaervation Data

The observation directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes 
place.

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the 
vehicle. Each row in these files corresponds to a single landmark. The two columns represent:

1.x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE.     
2.y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

**Note: You must transform vehicle's coordinate system to the map coordinate system.**

## Overview of Particle Filter 

![](https://github.com/Luzhongyue/Kidnapped-Vehicle/blob/master/Images/system.png)

## Paper Reading

For further reading and to dive deeper into the concept of particle filtering and basic filtering I suggest these materials which you may 
enjoyed reading:

* See here http://users.isy.liu.se/rt/fredrik/reports/09TAESpftutorial.pdf where theory meets practice in particle filters
* http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
* Applying particle filters in robots http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
* A gentle introduction to particle filtering http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
* Particle filters and its applications http://ocw.alfaisal.edu/NR/rdonlyres/Aeronautics-and-Astronautics/16-412JSpring-2005/F9652688
-E118-442E-98CE-3013CBEB8F11/0/a5_hso_plnvl_mlr.pdf
* Get to understand the difference particle filter has with other filters here http://www.dsi.unifi.it/users/chisci/idfric/Nonlinear_
filtering_Chen.pdf
* Robot mapping with particle filters http://duch.mimuw.edu.pl/~kowaluk/GOBR/slam11-particle-filter.pdf
* https://en.wikipedia.org/wiki/Particle_filter
* [Particle/Kalman Filter for Efficient Robot Localization](https://docs.google.com/viewer?url=http%3A%2F%2Fresearch.ijcaonline.org%2Fvo
lume106%2Fnumber2%2Fpxc3899554.pdf)
* [Great content for Kalman and Particle filters](https://docs.google.com/viewer?url=http%3A%2F%2Fwww.sft.asso.fr%2FLocal%2Fsft%2Fdir%2
Fuser-3775%2Fdocuments%2Factes%2FMetti5_School%2FLectures%26Tutorials-Texts%2FText-T10-Orlande.pdf)
* See also [Marginalized Particle Filters for Mixed Linear/Nonlinear State-space Models](https://docs.google.com/viewer?url=http%3A%2F%
2Fusers.isy.liu.se%2Frt%2Ffredrik%2Freports%2F04SPschon.pdf)
* [Particle Filters for Positioning, Navigation and Tracking](https://docs.google.com/viewer?url=http%3A%2F%2Fusers.isy.liu.se%2Frt%2Ffr
edrik%2Freports%2F01SPpf4pos.pdf)


