# Dial-A-Ride-Problem-with-Workload-Balancing
Several DARP lp file generators made with pyomo.

http://www.pyomo.org/

All DARP files create a LP file for a Dial A Ride Problem with the pyomo package in python. 
Will also solve the problem if the user has an optimizer.

These DARP models include time windows. It also has parameters for workload balancing among the vehicles, something not found in the literature and I researched as part of my Master's. 

Assumes all vehicles have a capacity of 1. Capacity can be increased but time windows will have to be tight in order to ensure
a proper, feasible solution. Ride time constraints can be added to ensure route feasibility as well. See Cordeau & Laporte for how to add ride time constraints.

DARP General is a general DARP model that will set up routes for k vehicles for n requests while minimizing the total travel time of all vehicles. This version includes workload balancing constraints - all vehicle workoads (travel time + service time) must be within two user specified parameters.

DARP MO is a multi-objective version of the DARP General. Workload balancing is included in the objective function with total travel time. No weights are present however.

DARP Day Of Changes is a specialized DARP designed to update routes from a previous schedule. Pickup times already scheduled are input as a new parameter called wi. The model strives to keep the difference between the previous pickup time and the new pickup smaller than a user specified parameter. It minimizes the number of changed pickup times. The vehicle performing the pickup and delivery can change however. Useful if small pickup time deviations are fine but large ones are not.
