import csv
import heapq

# Function to read CSV file into a 2D list
def read_csv_to_matrix(filename):
    matrix = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            matrix.append(list(map(float, row)))
    return matrix

# Function to implement A* algorithm
def a_star_algorithm(dist_matrix, heuristics_matrix, start_node, goal_node):
    num_nodes = len(dist_matrix)
    open_set = []
    heapq.heappush(open_set, (0, start_node))
    
    came_from = {start_node: None}
    g_score = {node: float('inf') for node in range(num_nodes)}
    g_score[start_node] = 0
    
    f_score = {node: float('inf') for node in range(num_nodes)}
    f_score[start_node] = heuristics_matrix[start_node][goal_node]
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal_node:
            return reconstruct_path(came_from, current)
        
        for neighbor in range(num_nodes):
            if dist_matrix[current][neighbor] != float('inf'):
                tentative_g_score = g_score[current] + dist_matrix[current][neighbor]
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristics_matrix[neighbor][goal_node]
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None

# Function to reconstruct the path
def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        if current is not None:
            total_path.append(current)
    total_path.reverse()
    return total_path

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

# Main function
def main():
    dist_matrix = read_csv_to_matrix('dist.csv')
    heuristics_matrix = read_csv_to_matrix('heuristics.csv')
    orient_matrix = read_csv_to_matrix('orient.csv')  # Assuming you have an orientation matrix

    start_node = 0  # Define your start node
    goal_node = 38  # Define your goal node

    path = a_star_algorithm(dist_matrix, heuristics_matrix, start_node, goal_node)
    
    if path is not None:
        print("path found")
        letter_path = []
        for i in path:
            letter_path.append(convert_to_letter(i))
        print(letter_path)
    else:
        print("No path found")

if __name__ == "__main__":
    main()

