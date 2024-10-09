import csv

with open('experiments/h1_walk_2024-07-29_09-10-40/experiment_data.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))
    print(len(data))