import math
import random
import matplotlib.pyplot as plt

# Parameters
num_nodes = 20
P = {chr(ord('A') + i): (random.randint(0, 10), random.randint(0, 10)) for i in range(num_nodes)}  # Set of nodes with (x, y) coordinates
L = [(random.choice(list(P.keys())), random.choice(list(P.keys()))) for _ in range(num_nodes * 2)]  # Set of links
V = ['Vehicle' + str(i) for i in range(1, 4)]  # Set of vehicles

# Ensure that starting and destination nodes for each vehicle are different and connected to other nodes
Sv = {}
Dv = {}
for vehicle in V:
    while True:
        start_node = random.choice(list(P.keys()))
        destination_nodes = [node for node in list(P.keys()) if node != start_node and (start_node, node) in L]
        if destination_nodes:
            destination_node = random.choice(destination_nodes)
            Sv[vehicle] = start_node
            Dv[vehicle] = destination_node
            break

B = 10000  # Large constant bigger than end of system time
Tvs = {vehicle: 0 for vehicle in V}  # Time when vehicle v enters the network
TRoad = 1  # Time to traverse any link with maximum road speed
TDelay = 0.1  # Delay added to the system time
INode = list(P.keys())  # Set of intersection nodes
CPercent = 10  # Percentage of added time delay based on the number of vehicles sharing the same link

# Decision Variables
Rv = {}  # Indicate whether link <i, j> is set on the route of vehicle v
Xv = {}  # System time when vehicle v traversed link <i, j>
TxD = {}  # System time when vehicle v arrives to destination
Nv = {}  # Number of vehicles sharing link <i, j> with vehicle v
Avv = {}  # Indicate vehicle v' is sharing link <i, j> with vehicle v
Alvv = {}  # Indicate if vehicle v' is passing through link <i, j> [To linearize the multi-vehicle constraint]
Agvv = {}  # Indicate if vehicle v' is arriving to link <i, j> [Require to linearize the multi-vehicle constraint]

# Initialize the decision variables
for vehicle in V:
    for link in L:
        Rv[(vehicle, link)] = 0
        Xv[(vehicle, link)] = B
        Nv[(vehicle, link)] = 0
        Avv[(vehicle, link)] = 0
        Alvv[(vehicle, link)] = 0
        Agvv[(vehicle, link)] = 0

# Initialize the decision variables
for vehicle in V:
    for link in L:
        Rv[(vehicle, link)] = 0
        Xv[(vehicle, link)] = B
        Nv[(vehicle, link)] = 0
        Avv[(vehicle, link)] = 0
        Alvv[(vehicle, link)] = 0
        Agvv[(vehicle, link)] = 0


def FindNeighbors(node, graph):
    """
    This function finds the neighboring nodes of the given node in the graph.

    Args:
    - node: The current node.
    - graph: The graph represented as an adjacency list.

    Returns:
    - neighbors: The neighboring nodes of the given node.
    """
    return graph[node]


def GetTrafficTime(node1, node2, traffic_data):
    """
    This function calculates the traffic time between two nodes based on the traffic data.

    Args:
    - node1: The first node.
    - node2: The second node.
    - traffic_data: The traffic data represented as a dictionary.

    Returns:
    - traffic_time: The traffic time between the two nodes.
    """
    # Example traffic data format: {('A', 'B'): 5, ('B', 'D'): 10, ...}
    return traffic_data.get((node1, node2), TRoad)  # Return default traffic time if link not present


def GetDistance(node1, node2, coordinates):
    """
    This function calculates the Euclidean distance between two nodes.

    Args:
    - node1: The first node.
    - node2: The second node.
    - coordinates: The coordinates of the nodes represented as a dictionary.

    Returns:
    - distance: The Euclidean distance between the two nodes.
    """
    x1, y1 = coordinates[node1]
    x2, y2 = coordinates[node2]
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def Normalized(value, scale=1.0):
    """
    This function normalizes the given value based on the scale.

    Args:
    - value: The value to be normalized.
    - scale: The scale for normalization.

    Returns:
    - normalized_value: The normalized value.
    """
    return value / scale


# BNART algorithm to find the best neighbor node for vehicle v
def BNART(graph, dead_end_nodes, current_vehicle_locations, destination, traversed_nodes, vehicle, traffic_data):
    node = traversed_nodes[-1]  # Current intersection node
    neighbors = FindNeighbors(node, graph)
    best_value = float('inf')
    best_neighbor = None

    for neighbor in neighbors:
        if neighbor not in traversed_nodes and neighbor not in dead_end_nodes:
            traffic_time = GetTrafficTime(neighbor, node, traffic_data)
            distance = GetDistance(neighbor, destination, P)
            norm_value = Normalized(traffic_time + distance)

            if norm_value < best_value:
                best_value = norm_value
                best_neighbor = neighbor

                # Update the decision variables
                Rv[(vehicle, (node, neighbor))] = 1
                Xv[(vehicle, (node, neighbor))] = Tvs[vehicle]
                if (vehicle, (node, neighbor)) not in Nv:
                    Nv[(vehicle, (node, neighbor))] = 0
                Nv[(vehicle, (node, neighbor))] += 1
                Avv[(vehicle, (node, neighbor))] = 1
                Alvv[(vehicle, (node, neighbor))] = 1
                Agvv[(vehicle, (node, neighbor))] = 1

    if best_neighbor is not None:
        current_vehicle_locations.append((node, best_neighbor))
        traversed_nodes.append(best_neighbor)
        Tvs[vehicle] += TRoad + TDelay * Nv[(vehicle, (node, best_neighbor))]

    return best_neighbor


def plot_graph(P, L):
    plt.figure(figsize=(8, 6))
    for link in L:
        x = [P[link[0]][0], P[link[1]][0]]
        y = [P[link[0]][1], P[link[1]][1]]
        plt.plot(x, y, 'k-', lw=1)
    for node, coords in P.items():
        plt.plot(coords[0], coords[1], 'bo', markersize=10)
        plt.text(coords[0], coords[1], node, fontsize=12, ha='right')


def plot_network(ax, P, L, traversed_nodes, vehicle, Sv, Dv, path_color, start_color, end_color):
    prev_node = Sv[vehicle]
    for node in traversed_nodes[vehicle][1:]:
        x = [P[prev_node][0], P[node][0]]
        y = [P[prev_node][1], P[node][1]]
        ax.plot(x, y, color=path_color, lw=2)
        prev_node = node
    start = Sv[vehicle]
    end = Dv[vehicle]
    ax.plot(P[start][0], P[start][1], 'o', markersize=10, color=start_color)
    ax.plot(P[end][0], P[end][1], 'o', markersize=10, color=end_color)

def plot_paths(P, L, traversed_nodes, Sv, Dv, path_colors, start_colors, end_colors):
    fig, ax = plt.subplots(figsize=(8, 6))
    for link in L:
        x = [P[link[0]][0], P[link[1]][0]]
        y = [P[link[0]][1], P[link[1]][1]]
        ax.plot(x, y, 'k-', lw=1)
    for node, coords in P.items():
        ax.plot(coords[0], coords[1], 'bo', markersize=10)
        ax.text(coords[0], coords[1], node, fontsize=12, ha='right')
    
    for i, vehicle in enumerate(V):
        plot_network(ax, P, L, traversed_nodes, vehicle, Sv, Dv, path_colors[i], start_colors[i], end_colors[i])

    ax.set_title("Network Graph with Vehicle Paths")
    ax.grid(True)
    plt.show()

# Print starting and destination points for each vehicle
for vehicle in V:
    print(f"{vehicle} starts at {Sv[vehicle]} and has a destination of {Dv[vehicle]}")

# Initialize simulation
graph = {node: [random.choice(list(P.keys())) for _ in range(random.randint(1, 5))] for node in P.keys()}
dead_end_nodes = []
current_vehicle_locations = []
traversed_nodes = {vehicle: [Sv[vehicle]] for vehicle in V}

# Example traffic data (time in minutes)
traffic_data = {(link[0], link[1]): random.randint(1, 15) for link in L}

# Colors for paths, start points, and end points
path_colors = ['b', 'g', 'r']
start_colors = ['c', 'm', 'y']
end_colors = ['k', 'purple', 'orange']

# Start simulation
print("\nSimulation Results:\n")
for time_step in range(10):
    for vehicle in V:
        if traversed_nodes[vehicle][-1] != Dv[vehicle]:
            best_neighbor = BNART(graph, dead_end_nodes, current_vehicle_locations, Dv[vehicle], traversed_nodes[vehicle], vehicle, traffic_data)
            print("Time Step {}: {} moved from {} to {}".format(time_step, vehicle, traversed_nodes[vehicle][-2], traversed_nodes[vehicle][-1]))
    print("\n" + "=" * 30 + "\n")

# Plot paths of all vehicles on a single graph
plot_paths(P, L, traversed_nodes, Sv, Dv, path_colors, start_colors, end_colors)
