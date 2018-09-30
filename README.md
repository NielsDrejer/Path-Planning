## Udacity Self-Driving Car Engineer Nanodegree, Term 3, Project 1: Path Planning

The goal of the project is to safely navigate a car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, and there is also a sparse map list of waypoints around the highway.

The car shall try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car shall avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car shall be able to make one complete loop around the 6946m highway. Finally the car shall not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Seed code for the project is provided here:

https://github.com/udacity/CarND-Path-Planning-Project

Also a simulator is provided for the car and highway.

## Notes about Installation

I am using a Mac for development.

The uWebSockets were already installed in connection with term 2, so I could directly execute the instructions:

1. mkdir build
2. cd build
3. cmake ..
4. make all

I had to modify the generated CMakeLists.txt to use the following line to enable make to find the linker:

link_directories(/usr/local/Cellar/libuv/1.20.3/lib)

The original line referred to the folder /usr/local/Cellar/libuv/1*/lib, and that does not work on my system.

After that everything worked as expected.

I used the Sublime text editor and make on the command line to build.

All the code I wrote is found in the file main.cpp between line 256 and 698. There are also some global variables and defines in lines 15-23.

Please note that I only submit the src files (not include the Eigen library), but not the build folder. Please place Eigen-3.3 under src and follow the 4 steps above to build my code.

## Rubric Points

### Compilation

#### Your code should compile  

Calling 'make all' on the command line compiles the code without warnings.

### Valid Trajectories

#### The car is able to drive 4.32 miles
Well for me it worked.

#### The car drives according to speed limit
The car never exceeds 49.5 mph.

#### Max acceleration and jerk are not exceeed
According to the simulator these values are within the limits at all times.

#### Car does not have collisions
In the current version my code appears to prevent collisions

#### The car stays in lane, except when doing lane changes
This is exactly the behavior of the car

#### The car is able to change lanes
The car change lanes when possible, according to a calculate cost for each lane, plus observations of whether other cars are within a safety distance.

### Code Model Description

The code I wrote is essentially divided into 4 logical parts.

Part 1 is found in lines 277-372 and does the analysis of the fusion data delivered from the simulator. Essentially this code does 2 things:

1. It determines the distance and velocity of the closest cars in front and behind our car in each of the 3 lanes. The code takes into account that another car can be 2 lanes at the same time (when it is doing a lane change). This data is used in the cost functions. It is worthwhile noticing that the velocity is converted to mile per hour in line 318 and 332, to allow easy comparison with the velocity of our car.

2. It determines whether other cars are inside a safety zone in front of and next to our car. The size of the safety zone is controlled by the variable MIN_DISTANCE, declared in line 20 to have the value 30 (meters). This data is used to decide when our car cannot perform a lane change. It is worthwhile noticing that we consider only cars in front in the lane of our own car, but also behind in the other lanes (lanes that our car can change to).

Part 2 is found in the lines 374-440, and calculates a cost associated with each lane. The cost of a lane consists of 3 components:

1. A cost depending on how close a car is in front of our car's position. I have chosen to generate a value between 0 and 1, the value decreases the further the other car is away. When the other car is further than 150 meters away the cost goes to 0.

2. A cost depending on how close a car is behind our car's position. As above the value is between 0 and 1, decreases the further the other car is away, and is 0 when the other car is further than 150 meters away.

3. A cost depending on the difference in velocity between our car and the closest car in front. Here we generate a value between -1 and 1 (negative if the other car is faster than our car, positive if our car is faster), and the value increase with the size of the difference, 30 mph generating the max value of -1 or 1.

The total cost per lane is simply the sum of these 3 components. There is no weighting (or each part is weighted with a value of 1).

Part 3 is located in the lines 456-577, and perform the first part of the path planning by determining which lane our car shall drive in, and which velocity we shall drive with.

The code between line 456 and 466 is essentially a poor mans handling of the wraparound of the value frenet s-coordinate that happens every 6946m of the highway. For the first 300 after start/wraparound point we perform no lane change, only adjustment of velocity. This handling is as I said a poor mans handling, but it appears to work well enough.

Following that there a 2 different scenarios handled:

1. Car in front of our car inside the safety distance.

2. No car in front of our car inside the safety distance.

In the first case (line 470-515) there is a per lane handling, in which we will decide to change lane if there is no other car inside the safety zone in that lane. We consider only the lane immediately next our own lane (no change of 2 lanes), and we use only the cost associated with the lanes in the case where we have the choice between 2 possible lanes (we are in the middle lane, and there is no other car in the safety zone of the other 2 lanes). In case our car is not in the middle lane and both other lanes are free, I prioritize lane a change to the left over a lane change to the right. This is the "correct" way to drive on the German autobahn's where I live, but as I understand it arbitrary on a US highway.  

For each lane change we apply a hysteresis value defined by the variable LANE_CHANGE_HYST in line 19. I use the value 150 corresponding to 3 seconds as the minimum time between subsequent lane changes. 3 seconds should be the maximum time it takes to complete a lane change. The point of this limitation is to prevent lane changes too often, and also to minimize the jerk which can become too high should the car decide to change 2 lanes at full speed.

If no lane change is possible, for whatever reason, the "ultima ratio" is to reduce speed. I either reduce the speed with 5 m/s, or with 10 m/s if the car in front is within a distance of 15 meters. This can happen if another car changes lane directly in front of our car (observed in the simulator a couple of times).

The second case (line 517-577) is when there is no car in front of our car in the lane we are driving. In this case essentially we look at the other lanes, and if no other car is inside the safe zone of that lane, we select a new lane by comparing the costs of possible lanes. Also here I apply the minimum time between lane changes described above, and I also use a hysteresis value for the cost comparison (COST_HYST declared in line 18), to prevent too many lane changes in case the costs are very similar.

This means a lane change is decided only if the safety zone of the other lane is also free, and that means that in any case (no matter which lane we decide to be in) we can also increase the reference velocity of our car to get as close a possible to the maximum allowed velocity of 50 mph.

Part 4 is found in lines 593 to 697. Here I do the second part of the path planning by generating new trajectories from the selected lane and reference velocity of our car. This code is essentially the same as what was provided in the project walkthrough video. I use the spline interpolation library as provided and discussed in the project walkthrough.

### Reflection

The project could be solved in much more comprehensive ways that what I chose to do. For example one could apply more, and more advanced cost functions. My solution is probably a minimum realisation of a path planner. I chose to stop at this point because my solution works (as far as I can tell), and I have applied the concept of cost functions, which I understand to the be a central aspect in the art of path planning.

Having said that there are so many things which could have been done better (time permitting). With no claim for completeness, here are some immediate thought I had:

1. I limit my car to change only 1 lane at a time. In real life lane changes over 2 lanes could be possible.

2. The wrap around of s-values and waypoints is  not handled properly. My simple handling appears to be good enough to prevent disasters, but a real handling would be better. Having said that wrap arounds like this should not happen very often in real life.

3. My algorithms assume that my car is always in a clearly defined lane. Actually this is not always the case, as my car does change lane every now and then.

4. The velocity of my car essentially either increases or decreases. This is a very uneconomical and uncomfortable way of driving, and a proper implementation should take care of this issue properly.

5. There is an infinite number of other cost functions which could have been used in addition. Side distance springs to mind, to penalise lane changes when there is another car the next lane but one. This would reduce risk of the other car doing a lane change at the same time as our car into the same lane.
