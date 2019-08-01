# Dial-A-Ride-Problem-with-Workload-Balancing
General Dial A Ride Problem (DARP) made with the pyomo package in python. 
http://www.pyomo.org/

Creates an LP file for a general Dial A Ride Problem with the pyomo package in python. 
Will also solve the problem if the user has an optimizer.

This DARP model includes parameters for workload balancing among the vehicles. 
Assumes all vehicles have a capacity of 1. Capacity can be increased but time windows will have to be tight in order to ensure
a proper, feasible solution.
