from sim2d import *
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import matplotlib.path as mpath

def clean_triangles(inside_points, outside_points, triangles):
    clean_triangles = []

    for t in triangles:
        # Remove triangles formed only of inside or outside points
        if t[0] < len(inside_points) and t[1] < len(inside_points) and t[2] < len(inside_points):
            continue
        if t[0] >= len(inside_points) and t[1] >= len(inside_points) and t[2] >= len(inside_points):
            continue
            
        clean_triangles.append([t[0], t[1], t[2]])

    return np.array(clean_triangles)


sim = Sim2D()
sim.frequency = 25
sim.set_track('track.json')

points = []
# for i in range(len(sim.ins_line)):
#     points.append(sim.ins_line[i][0], sim.ins_line[i][1]])
# for i in range(len(sim.out_line)):
#     points[len(sim.ins_line)+i] = [sim.out_line[i][0], sim.out_line[i][1]]

for x in sim.ins_line:
    points.append([x[0], x[1]])
for x in sim.out_line:
    points.append([x[0], x[1]])

points = np.array(points)

t = Delaunay(points)

t_clean = clean_triangles(sim.ins_line, sim.out_line, t.simplices.copy())
center_points = []
points_clean = points[t_clean]
for p in points_clean:
    x = (p[0][0] + p[1][0] + p[2][0])/3.0
    y = (p[0][1] + p[1][1] + p[2][1])/3.0
    center_points.append([x,y])

center_points = np.array(center_points)

#plt.triplot(points[:,0], points[:,1], t_clean)
plt.plot(points[:,0], points[:,1], 'o')
plt.plot(center_points[:,0], center_points[:,1],'x')
plt.show()


#sim.set_trajectory(0, 'test_traj.csv')

# while not sim.done:
#     sim.tick()
