# Chance-Constrained Vehicle Routing Problem (CCVRP)

The chance-constrainted vehicle routing problem (CCVRP) is a problem
concerned with finding the optimal route that a vehicle should
follow to serve a sequence of customers. The vehicle starts at a
depot with a set capacity of goods, and the demands of the
customers is not known until the vehicle leaves the depot. As with
any routing problem, the distances between customers are defined as
a graph.

In this particular implementation of the CCVRP, we assume that a
customer's demand cannot exceed the capacity of the vehicle, and
that the total demand is somewhere between `C` and `2C`, where `C`
is the capacity of the vehicle. Additionally, we do not allow the
vehicle to return to the depot more than once. 

The code for this version of the CCVRP is split into two sections.
The first file ,`tsp.jl`, implements only the TSP portion of the
problem, where we decide what the best tour is which starts and
ends at the depot and visits each customer exactly once. The second
file, `route.jl`, adds to this by also implementing the constraints
which determine which route will be the best with a return to the
depot.

## Implementation Notes

Firstly, the LP solver used is Gurobi, and the constraints,
variables, and objective function passed in are defined with JuMP. 
In order to use these programs, you will need to have Gurobi 
installed and configure JuMP so it knows where the home directory
is for the Gurobi optimizer.

Currently, the adjacency matrix for the system and the list of
possible demands is hard-coded. However, at some point the plan is
to design a way to input a graph and get out the adjacency matrix
and the list of possible demands from it.
