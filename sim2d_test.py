from racecar_2d import *
import numpy
import pdb
pdb.set_trace()

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('track.json')

print("DELAUNAY: {}".format(len(sim.delaunay_triangles)))

init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0))
init_pose.append((sim.race_line[10][0], sim.race_line[10][1], 0.0, 0.0))
init_pose.append((sim.race_line[20][0], sim.race_line[20][1], 0.0, 0.0))

sim.add_player('Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
sim.players[0].add_sensor('laser', SensorLaser(sim.players[0], sim, (-np.pi/6.0, 0.0, np.pi/6.0), 20.0, 0.1))

_, index_hint = sim.is_inside_track(init_pose[0][0:2])

while not sim.done:
    sim.update_player(0, (1,0.1))
    readings = sim.players[0].simulate('laser', index_hint=index_hint, interval=100)

    for r in readings:
        # Readings are (laser_angle, distance)
        current_angle = sim.players[0].current_state[2]
        point1 = sim.players[0].current_state[0:2]
        point2 = (point1[0] + np.cos(r[0]+current_angle)*r[1], point1[1] + np.sin(r[0]+current_angle)*r[1])

        sim.draw_path([point1, point2])

    # sim.test_laser(index_hint, 100)
    # sweep = sim.get_sweeping(sim.delaunay_triangles, index_hint, 100)
    # for i in sweep:
    #     t = sim.delaunay_triangles[i]
    #     sim.draw_point(t[0])
    #     sim.draw_point(t[1])
    #     sim.draw_point(t[2])

    # first_player_state = sim.players[0].current_state
    # print("x: {:.2f}, y: {:.2f}, theta: {:.2f}, v: {:.2f}".format(first_player_state[0], first_player_state[1], first_player_state[2], first_player_state[3]))

    sim.tick()
    #input("Press to continue.")

    point = (sim.players[0].current_state[0], sim.players[0].current_state[1])
    is_inside, hint = sim.is_inside_track(point)

    if is_inside:
        index_hint = hint
    else:
        index_hint = -1
    

    #print(sim.get_track_coordinates(point))