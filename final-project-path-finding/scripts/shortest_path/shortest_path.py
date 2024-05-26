import csv
import sys

def read_csv_to_matrix(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        matrix = list(reader)
    adj_matrix = []
    for row in matrix:
        adj_row = []
        for cell in row:
            if cell == '[]':
                adj_row.append(float('inf'))  # No path available
            else:
                # Convert the string "[100, 90]" to a list [100, 90]
                cell = cell[1:-1]
                data = float(cell.split(',')[0])
                print(data)
                distance = data
                adj_row.append(distance)
        adj_matrix.append(adj_row)
    print(adj_matrix)
    return adj_matrix

def dijkstra(adj_matrix, source):
    """Applies Dijkstra's algorithm to find the shortest path from the source to all other vertices."""
    n = len(adj_matrix)
    dist = [float('inf')] * n
    dist[source] = 0
    visited = [False] * n
    predecessor = [-1] * n

    for _ in range(n):
        # Find the vertex with the minimum distance from the set of vertices not yet processed.
        u = min((v for v in range(n) if not visited[v]), key=lambda v: dist[v])
        visited[u] = True
        
        # Update distance of the adjacent vertices of the picked vertex.
        for v in range(n):
            if adj_matrix[u][v] > 0 and not visited[v] and dist[v] > dist[u] + adj_matrix[u][v]:
                dist[v] = dist[u] + adj_matrix[u][v]
                predecessor[v] = u

    return dist, predecessor

def convert_to_index(node_name):
    if len(node_name) == 2:
        return 26 + ord(node_name.lower()[1])-97
    elif len(node_name) == 1:
        return ord(node_name.lower())-97

def convert_to_letter(num):
    if num <= 26:
        return chr(num+97)
    else:
        num = num - 26
        return "A" + chr(num+97)

def shortest_path(predecessor, source, dest):
    """Reconstructs the shortest path from source to dest using the predecessor array."""
    path = []
    step = dest
    while step != -1:
        path.append(step)
        step = predecessor[step]
        if step == source:
            path.append(step)
            break
    return path[::-1]

# Usage
filename = 'adjacency.csv'
source_vertex = convert_to_index('B')  # change as needed
destination_vertex = convert_to_index('Q')  # change as needed

adj_matrix = read_csv_to_matrix(filename)
distances, predecessors = dijkstra(adj_matrix, source_vertex)
path = shortest_path(predecessors, source_vertex, destination_vertex)

print("Shortest distances:", distances)
print("Shortest path from", source_vertex, "to", destination_vertex, ":", path)
str_path = []
for i in path:
    str_path.append(convert_to_letter(i))
print(str_path)
