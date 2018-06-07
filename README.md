# Pegasus

Linux [![Build Status](https://travis-ci.org/Godlike/Pegasus.svg?branch=master)](https://travis-ci.org/Godlike/Pegasus)  
Windows [![Build status](https://ci.appveyor.com/api/projects/status/fnf0kri6wpyit1pi?svg=true)](https://ci.appveyor.com/project/ilia-glushchenko/pegasus)

## About
Pegasus is a cross-platform C++ game physics engine that is written with the data-oriented design in mind with minimum external dependencies (GLM and SPDLog). The aim of this project is to implement a robust, modern and simple to use API for game physics computations. The main motivation to write this engine was curiosity and desire to write physics code that utilizes a modern approach to software development, such as cache friendly code, high parallelization ability (CPU and GPU with CS), robust memory management with model C++ standard versions.

* [Building and Running Demo](https://github.com/Godlike/Pegasus/wiki/Building-and-Running-Demo)
* [Features](https://github.com/Godlike/Pegasus/wiki/Features)
* [Architecture Diagram](https://github.com/Godlike/Pegasus/wiki/Architecture-Diagram)

<a target="_blank" href="https://discord.gg/kQVvHQg">
<img src="https://discordapp.com/assets/bb408e0343ddedc0967f246f7e89cebf.svg" alt="Join discord chat." width="100"><br>
</a>

## Building

### Ubuntu:
_Tested on Ubuntu 16.04._

1. Install essential components:  
```sudo apt-get install build-essential git cmake gcc g++```

2. Install dependencies:  
```sudo apt-get install freeglut3 freeglut3-dev libglew1.5 libglew1.5-dev libglu1-mesa libglu1-mesa-dev libgl1-mesa-glx libgl1-mesa-dev libxtst-dev xvfb libxmu-dev libxi-dev libxinerama-dev```

3. Clone and initialize:  
```git clone git@github.com:Godlike/Pegasus.git && cd Pegasus```  
```git submodule init && git submodule update --init --recursive```
4. Build:  
```mkdir build && cd build && cmake .. && make && chmod +x ./output/PegasusPhysics```
5. Run the demo:  
```./output/PegasusPhysics```

### Windows:
_Tested on Windows 10 VS 2017._

1. [Download GitHub Windows client](https://desktop.github.com/)
2. Clone repository with the application from `git@github.com:Godlike/Pegasus.git`
3. [Download CMake GUI for Windows](https://cmake.org/)
4. Create a Visual Studio Project using CMake GUI
5. Open the solution in VS, build it and run ```PegasusDemo``` project.

### Mac OS:
_Tested on Mac OS High Sierra._
1. [Install CMake](https://cmake.org/install/)
2. [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
3. Clone and initialize:  
```git clone git@github.com:Godlike/Pegasus.git && cd Pegasus```  
```git submodule init && git submodule update --init --recursive```
4. Build:  
```mkdir build && cd build && cmake .. && make && chmod +x ./output/PegasusPhysics```
5. Run the demo:  
```./output/PegasusPhysics```

# Overview

## Collision primitives 
Plane

![](https://dl.dropboxusercontent.com/s/efxep0mm48hqb5n/PlaneStatic.gif)

Box

![](https://dl.dropboxusercontent.com/s/xveqti9k1knzzwl/BoxDynamic.gif)

Sphere

![](https://dl.dropboxusercontent.com/s/ka70wayu5a4q77z/SphereDynamic.gif)

## Collision detection
Collision detection is handled using custom implementations of Jacobi Eigenvalue, Quickhull Convex Hull, Half-Edge Data Structure, CSO, GJK and EPA algorithms.

## Collision resolution
Collision resolution is based on the constraint collision model and implemented using such techniques as Sequential Impules, Restitution Slop, and Warm Starting to help improve overall stabity of the system.

## Integration
Pegasus provides an API for creation of Dynamic and Static rigid bodies. Creating new simulation is as simple as creation new `Scene` object, and population of the new `Scene` could be peformd trought high-level OOP based API or trough low level Functional handle-based API, which provides developer with the flexibily when performas is needed.

![](https://dl.dropboxusercontent.com/s/3ppn652zd5m0frf/StaticBox.gif)

## Feature list
### Done
#### Collision detector
* Discrete collision detection
* Primitives: Plane, Ray, Sphere, Box
* Intersection, Contact normal, Penetration depth, Contact point calculation (GJK, EPA)
* Bounding sphere, AABB, OBB
* Ray tracing

#### Collision resolver
* Static rigid bodies 
* Dynamic rigid bodies
* Stable stacking
* Constraint-Based resolution

#### Integrator
* Static bodies
* Dynamic bodies
* Linear motion 
* Force generators
* Object groups
* Angular motion
* Friction

***

### Planned
#### Collision detector
* _Swept sphere based continuous collision detection_
* _Primitives_: _Triangle_, _Cone_, _Cylinder_, _8-DOP_, _Convex hull_
* _Sphere tree_, _AABB tree_, _OBB tree_, _8-DOP tree_, _Convex Hull tree_
* _Space partitioning tree_
* _Narrow phase collision detection_
* _Broad phase collision detection_
* _Compound geometries_

#### Collision resolver
* _Island based sleeping_
* _The time of impact_
* _Constraints_

#### Integrator
* _Kinematic bodies_
* _Impulse generators_
* _Soft Body Physics_
* _Liquids Physics_
* _Gases Physics_
* _Clothes Physics_
* _Ragdolls_
