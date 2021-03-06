# Path Generation

Paths are generated by creating 5 different desired points and creating a spline out of said points. The first two points in the set is the previous two positions in the car's path. The next three points are positions in the desired lane, with each point 30m apart.

The resulting spline is used to extract points that form the final path. Speed is maintained by estimating the length of the spline and using points along the spline with a distance that enables the ideal speed.

# Behavior Planning

Although we had been taught to use weighted cost functions to select the desired behavior, I chose instead to implement a simple set of conditional behaviors with priority. 

1. If not yet at desired lane, continue moving towards said lane.
2. If there's no car ahead, stay in the current lane at the ideal speed.
3. If it's safe to change to left lane, change to the left lane.
4. If it's safe to change to right lane, change to the right lane.
5. Slow down to a speed slightly less than the car ahead.

The above behaviors are depicted by two variables (desired_speed, desired_d). Collision avoidance is achieved by checking for cars ahead when moving forward and by checking cars to the side when switching lanes. 

Other cars' locations are "predicted" by calculating their future position using their current velocity and the time when which the current path ends.

# Avoiding Jerk

I have a function MaxSpeedWithoutJerk in planner.cpp that calculates the max / min velocity according to the max jerk / acceleration parameters.
