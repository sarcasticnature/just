# just
Jake's Ugly Simulation Testbed

![tests](https://github.com/sarcasticnature/just/actions/workflows/tests.yml/badge.svg)

## What is just?
just is an implementation of a robotics motion planning algorithm (possibly more than one in the future), with an accompanying simulation environment and visualization(s) to boot.

The focus is currently on methods related to the Vector Field Histogram (VFH) approach of motion planning for obstacle avoidance.
There are evolutions of this algorithm (VFH+, VFH\*, etc), which may be explored at a later date.

In addition to motion planning, this repository implements a mapping technique known as Histogramic In-Motion Mapping (HIMM), which was used in the original VFH paper.
There are no plans to implement SLAM at this time (although that could be an extension once the projects initial goals are complete), so it is assumed that the agents executing the VFH motion policy have 'perfect' absolute state estimation.

## Where did the inspiration for just come from?
The primary inspiration for this project was the paper "The Vector Field Histogram-Fast Obstacle Avoidance For Mobile Robots" from volume 7, number 3 of the (1991) IEEE Transactions on Robotics and Automation.
The mapping approach was taken from the paper "Histogramic in-motion mapping for mobile robot obstacle avoidance", which was cited by the VFH paper.

Note that some slight modifications/improvements have been made to the algorithms presented in the papers to better fit this use case.

## Why implement a (very) old approach?
The short (and honest) answer is that the state of the art in mobile robot motion planning, mapping, and control is quite involved at this point in time, making it a poor choice to implement as a side project done in one's free time.
VFH (and other older approach(es)) is comparably much easier to implement and reason about, while still offering compelling motion planning capabilities.

## What are the overall goals of just?
- Create and visualize a 2d simulation environment for an 'ideal' mobile robot
- Implement a mapping algorithm (HIMM)
- Implement a motion planning algorithm (VFH) to drive the agent around
- Thoroughly test the algorithms' implementation
- Extend the project's capabilities to things such as:
    * Multiple agents
    * Other motion planning algorithms (VFH+, VFH\*, etc)
    * SLAM
    * ?

## The story so far
A gif of an 'agent' (the blue square) navigating to a goal (green dot) while avoiding obstacles (white rectangles) running in simulation is shown below:

![use your imagination](/doc/demo.gif)

There are many things happening under the hood at this point, but perhaps the most interesting is the first part of the VFH algorithm: transforming the Histogram Grid into polar space.  A plot of the polar histogram at various time steps (from the simulation above) is shown below:

![use your imagination](/doc/plot.png)

These polar histograms are created from the Histogram Grid, a representation of which can be seen below (from the final timestep of the above simulation)

![use your imagination](/doc/map.png)

## What's the roadmap looking like?
This is a _very_ coarse estimation of the roadmap and is subject to change:

* [x] Create a basic demo of using box2d and raylib to simulate/visualize an """agent""" and its """environment""" (note the heavy quotes)
* [x] Implement a Histogram Grid for mapping
* [x] Implement adding perception data (percepts) to said grid (following the method described in the HIMM paper)
* [x] Create an idealistic 'sensor' that mimics a LIDAR/RADAR/Ultrasonic sensor array, for use in adding percepts to the grid
* [x] Implement VFH
    * [x] perform the first data transformation to a polar histogram grid
    * [x] perform the second transformation to produce a steering output
        * [x] steering angle
        * [x] steering speed
* [x] Create an 'agent' that
    * [x] moves in the world
    * [x] has a perception 'sensor'
    * [x] updates a histogram grid
    * [x] utilizes VFH to control its motion
    * [x] can be visualized
* [x] Create a configurable simulation environment (with obstacles) in which to place agents
* [x] Add logging functionality so that the results of a simulation can be replayed and analyzed after the fact
* [x] Create compelling visualizations of the system as a whole

Demos may be created along the way to show off progress.

Note that the underlying functionality of all the code used in the above should be unit tested using Doctest, with the exception of visualizations and demos (which can be 'tested' visually).
