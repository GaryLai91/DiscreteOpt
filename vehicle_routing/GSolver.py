from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math


def length(point1, point2):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)


def compute_distance_matrix(customers):
    matrix = [[0] * len(customers) for i in range(len(customers))]
    for i in customers:
        for j in customers:
            if i.index != j.index:
                matrix[i.index][j.index] = length(i, j)
            else:
                matrix[i.index][j.index] = 0
    return matrix


def create_data_model(customers, vehicle_count, vehicle_capacity):
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = compute_distance_matrix(customers)
    data["demands"] = [c.demand for c in customers]
    data["vehicle_capacities"] = [vehicle_capacity for i in range(vehicle_count)]
    data["num_vehicles"] = vehicle_count
    data["depot"] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            plan_output += " {0} Load({1}) -> ".format(node_index, route_load)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += " {0} Load({1})\n".format(manager.IndexToNode(index), route_load)
        plan_output += "Distance of the route: {}m\n".format(route_distance)
        plan_output += "Load of the route: {}\n".format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print("Total distance of all routes: {}m".format(total_distance))
    print("Total load of all routes: {}".format(total_load))


def solve(customers, vehicle_count, vehicle_capacity):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(customers, vehicle_count, vehicle_capacity)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Setting first solution heuristic.
    """
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    """
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH
    )
    search_parameters.time_limit.seconds = 1800

    # Solve the problem.
    obj_value = 0
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
        obj_value = solution.ObjectiveValue()
        path = []
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            p = []
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                p.append(node_index)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
            p.append(manager.IndexToNode(index))
            path.append(p)

    return obj_value, path

