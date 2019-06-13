# Danny Bynum Project Submission
Basic Particle Filter has been implemented and the simulation window indicats that the criteria has been passed.


## Project Description From Udacity
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
I worked in Windows Visual Studio and utilized the Linux Subsystem.  In order to copy the file back over from Windows to Linux Subsystem Location - and Compile and run I used the following command fairly often (from the build folder) :-):
```
cd .. && cp /mnt/c/users/bynum/documents/udacity/term2/DWB_BasicParticleFilter/src/particle_filter.cpp src/ && cd build && cmake .. && make && ./particle_filter
```

## Description of the files
I only had to modivy src/particle_filter.cpp ... but I had to read through particle_filter.h to understand the classes and structures I was using.  It wasn't too important to look at the "main.cpp" code that was provided if you understand the fundamental concepts of how the particle filter is supposed to be implemented - but the main.cpp drives everything including all of the calls to the functions I created in particle_filter.cpp

Udacity's comments:
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


# Udacity Description of the files associated with the project
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.