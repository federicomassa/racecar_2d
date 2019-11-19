import csv

traj = []

with open('test_traj.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        traj.append([None,None,None,None])
        i = 0
        for e in row:
            traj[len(traj)-1][i] = float(row[e])
            i+=1

print(traj)