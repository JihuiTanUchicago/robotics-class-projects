import csv 
import math
def read_csv_to_matrix(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        matrix = list(reader)
    dist_matrix = []
    orient_matrix = []
    for row in matrix:
        dist_row = []
        orient_row = []
        for cell in row:
            if cell == '[]':
                dist_row.append(float('inf'))  # No path available
                orient_row.append(float('inf')) # No angle available
            else:
                # Convert the string "[100, 90]" to a list [100, 90]
                cell = cell[1:-1]
                data = cell.split(',')
                distance = float(data[0])
                orientation = float(data[1])
                dist_row.append(distance)
                orient_row.append(orientation)
        dist_matrix.append(dist_row)
        orient_matrix.append(orient_row)

    # Define the CSV file name
    dist_file = 'dist.csv'
    orient_file = 'orient.csv'

    # Write the 2D array to the CSV file
    with open(dist_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(dist_matrix)
    # Write the 2D array to the CSV file
    with open(orient_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(orient_matrix)

    coordinates = [[-1, -1] for _ in range(39)]
    print(coordinates)
    # AG is the origin, which is 32 (0-index)
    coordinates[32] = [0,0] 
    candidates = [[32,32]]
    while candidates != []:
        ref_node = candidates[0][0]
        target_node = candidates[0][1]
        ref_coordinate = coordinates[ref_node]
        candidates = candidates[1:]
        print(candidates)
        distance = dist_matrix[ref_node][target_node]
        print(f"distance {distance}")
        if orient_matrix[ref_node][target_node] == 0.0:
            target_coordinate = [ref_coordinate[0],ref_coordinate[1] + distance]
        elif orient_matrix[ref_node][target_node] == 90.0:
            target_coordinate = [ref_coordinate[0] + distance, ref_coordinate[1]]
        elif orient_matrix[ref_node][target_node] == 180.0:
            target_coordinate = [ref_coordinate[0], ref_coordinate[1] - distance]
        else:
            target_coordinate = [ref_coordinate[0] - distance, ref_coordinate[1]]
        coordinates[target_node] = target_coordinate
        print(f"target_node = {target_node}; coordinate = {coordinates[target_node]}")
        
        
        for i in range(0, 39):
            if dist_matrix[target_node][i] != float('inf') and coordinates[i] == [-1,-1]:     
                candidates.append([target_node, i])
    for i in range(39):
        coordinates[i] = [round(coordinates[i][0], 3), round(coordinates[i][1], 3)]
    print(coordinates)
    with open('coordinates.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(coordinates)

    print(f"2D array has been written")



def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Read the coordinates from the CSV file
input_csv = 'coordinates.csv'
coordinates = []

with open(input_csv, mode='r') as file:
    reader = csv.reader(file)
    for row in reader:
        x, y = map(float, row)  # Ensure that coordinates are read as floats
        coordinates.append((x, y))

# Calculate the Euclidean distance matrix
num_nodes = len(coordinates)
distance_matrix = [[0] * num_nodes for _ in range(num_nodes)]

for i in range(num_nodes):
    for j in range(num_nodes):
        if i != j:
            x1, y1 = coordinates[i]
            x2, y2 = coordinates[j]
            distance_matrix[i][j] = calculate_euclidean_distance(x1, y1, x2, y2)

# Write the distance matrix to a new CSV file
output_csv = 'euclidean_dist_matrix.csv'

with open(output_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(distance_matrix)
print(coordinates[3])
print(f"Distance matrix has been written to {output_csv}")


read_csv_to_matrix('adjacency.csv')

