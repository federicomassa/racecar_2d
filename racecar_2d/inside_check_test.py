from sim2d import *
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import matplotlib.path as mpath

import time

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

def is_inside(point, triangle):
    t1 = triangle[0]
    t2 = triangle[1]
    t3 = triangle[2]
    c1 = (t2[0]-t1[0])*(point[1]-t1[1])-(t2[1]-t1[1])*(point[0]-t1[0])
    c2 = (t3[0]-t2[0])*(point[1]-t2[1])-(t3[1]-t2[1])*(point[0]-t2[0])
    c3 = (t1[0]-t3[0])*(point[1]-t3[1])-(t1[1]-t3[1])*(point[0]-t3[0])
    if (c1<0 and c2<0 and c3<0) or (c1>0 and c2>0 and c3>0):
        return True
    else:
        return False

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

# plt.triplot(points[:,0], points[:,1], t_clean)
# plt.plot(points[:,0], points[:,1], 'o')
# #plt.plot(center_points[:,0], center_points[:,1],'x')
# plt.show()


#sim.set_trajectory(0, 'test_traj.csv')

init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0))
sim.add_player('Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])

last_clicked_point = None
while not sim.done:
    sim.update_player(0, (1,0.1))
    sim.tick()

    if sim.clicked_point == None and last_clicked_point != None:
        is_inside_track = False

        for triangle in points_clean:
            if is_inside(last_clicked_point, triangle):
                is_inside_track = True
                break
        
        print("Clicked point inside? {}".format(is_inside_track))
    
    last_clicked_point = sim.clicked_point
