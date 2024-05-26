import csv
import string

num_nodes = 39
adjacency_matrix = [[-1] * num_nodes for _ in range(num_nodes)]

def convert_to_index(node_name):
    if len(node_name) == 2:
        print("2 case activated")
        return 26 + ord(node_name.lower()[1])-97
    elif len(node_name) == 1:
        print("1 case activated")
        return ord(node_name.lower())-97

with open('data.csv', newline='') as file:
    rowreader = csv.reader(file, delimiter=',')
    for row in rowreader:
        start, end = row[0].split('->')
        distance = float(row[1])
        start_idx = convert_to_index(start)
        end_idx = convert_to_index(end)
        print(f'start index is {start_idx} and end index is {end_idx}')
        adjacency_matrix[start_idx][end_idx] = distance
        adjacency_matrix[end_idx][start_idx] = distance

with open('adjacency_matrix.csv', 'w+') as file:
    writer = csv.writer(file, delimiter='\t')
    writer.writerows(adjacency_matrix)
