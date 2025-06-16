import random
import heapq
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tkinter import Tk, Label, Button, Entry, Frame, messagebox

# ----------------------------
# Graph representation
# ----------------------------
traffic_graph = {
    1: {2: 0, 3: 0},
    2: {1: 0, 3: 0},
    3: {1: 0, 2: 0}
}

car_positions = []
green_lane = 0  # Initialize green_lane

# ----------------------------
# Dijkstra's algorithm
# ----------------------------
def dijkstra(graph, start):
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    while pq:
        current_distance, current_node = heapq.heappop(pq)
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    return min(distances, key=distances.get)

# ----------------------------
# Update traffic weights randomly
# ----------------------------
def update_traffic_graph(graph):
    for lane in graph:
        for neighbor in graph[lane]:
            graph[lane][neighbor] = random.randint(1, 20)

# ----------------------------
# Initialize random car positions
# ----------------------------
def initialize_cars():
    global car_positions
    car_positions = [
        (random.choice(list(traffic_graph.keys())),
         random.choice(list(traffic_graph.keys())))
        for _ in range(5)
    ]

# ----------------------------
# Animate car movement on graph
# ----------------------------
def animate(i):
    plt.clf()
    G = nx.DiGraph()
    for node, neighbors in traffic_graph.items():
        for neighbor, weight in neighbors.items():
            G.add_edge(node, neighbor, weight=weight)

    pos = nx.circular_layout(G)
    edge_labels = nx.get_edge_attributes(G, 'weight')

    global green_lane
    edge_colors = []
    for u, v in G.edges():
        if u == green_lane or v == green_lane:
            edge_colors.append('green')
        else:
            edge_colors.append('red')

    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=10, edge_color=edge_colors)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    global car_positions
    for idx, (current, target) in enumerate(car_positions):
        next_target = random.choice(list(traffic_graph[current].keys()))
        car_positions[idx] = (target, next_target)
        car_x, car_y = pos[current]
        plt.scatter(car_x, car_y, color='red', s=100)

    plt.title("Traffic Graph with Cars")

# ----------------------------
# Control traffic lights
# ----------------------------
def control_traffic(lane, status_label):
    global green_lane
    green_lane = lane
    green_time = 10  # seconds
    status_label.config(text=f"Green In Lane {lane} Active\nRed In Other Lanes Active")
    root.after(green_time * 1000, lambda: status_label.config(text=f"Lane {lane} Green Time: {green_time} seconds"))

# ----------------------------
# Start the simulation
# ----------------------------
def start_simulation(emergency_lane_entry, status_label):
    try:
        emergency_lane = int(emergency_lane_entry.get())
    except ValueError:
        messagebox.showerror("Invalid input", "Enter a valid lane number (1, 2, 3 or 0 for none).")
        return

    update_traffic_graph(traffic_graph)
    initialize_cars()

    if emergency_lane in [1, 2, 3]:
        optimal_lane = emergency_lane
    else:
        optimal_lane = dijkstra(traffic_graph, start=1)

    status_label.config(text=f"{'Emergency' if emergency_lane in [1, 2, 3] else 'Optimal'} Lane: {optimal_lane}")
    control_traffic(optimal_lane, status_label)

    ani = FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()

# ----------------------------
# Tkinter GUI setup
# ----------------------------
root = Tk()
root.title("Traffic Control Simulation")
root.geometry("800x600")

control_frame = Frame(root)
control_frame.pack(pady=20)

Label(control_frame, text="Enter Emergency Lane (1, 2, 3 or 0 for none):").grid(row=0, column=0, padx=10)
emergency_lane_entry = Entry(control_frame)
emergency_lane_entry.grid(row=0, column=1, padx=10)

Button(control_frame, text="Start Simulation",
       command=lambda: start_simulation(emergency_lane_entry, status_label)).grid(row=1, columnspan=2, pady=10)

status_label = Label(root, text="Traffic Light Status", font=("Arial", 14))
status_label.pack(pady=20)

root.mainloop()
