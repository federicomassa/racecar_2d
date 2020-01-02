from racecar_2d import *
import numpy
import pdb
pdb.set_trace()

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('/home/luca/Downloads/Baltimore.json', dump_file='Baltimore.dump', one_point_every=10)
#sim.set_track('track.json', dump_file='track.dump')


init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0, 0.0))

sim.add_player('../images/Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
player = sim.players[0]
player.add_sensor('laser', SensorLaser(player, sim, (-np.pi/6.0, 0.0, np.pi/6.0), 40.0, 0.1))

inside, last_good_index_hint = sim.is_inside_track(player.current_state[0:2])
if not inside:
    raise Exception("Player must be initialized inside the track")

# Set to manual mode by default (change with M key while running)
sim.is_manual = True
while not sim.done:
    # Autonomous mode: give acceleration and steering = (0.0,0.0) to player 0 (the first and only in this case)
    sim.update_player(0, (0.0,0.0)) 

    # This is the current state after update
    (x,y,theta,u,v) = player.current_state[0:5]

    # Verify if the vehicle is inside the track, return an index_hint that is used to make the sensor simulation more efficient. Technically, this is the index of the Delaunay triangle the vehicle is in
    inside, index_hint = sim.is_inside_track(player.current_state[0:2], last_good_index_hint, 20)

    # If you want to simulate sensors, do like the following. Otherwise, you can comment out this part, the vehicle pose has already been updated in update_player
    if inside:
        last_good_index_hint = index_hint
        readings = player.simulate('laser', index_hint=index_hint, interval=40)
        [s, d] = sim.get_track_coordinates(player.current_state[0:2])
        state = np.array([readings[0][1], readings[1][1], readings[2][1], s, d, x, y, theta, u, v]).reshape((-1, 10))
        laser_points = []

        for data in readings:
            laser_angle = data[0]
            laser_range = data[1]

            # Do not draw point if measurement is out of range
            if laser_range + player.sensors['laser'].resolution >= player.sensors['laser'].range:
                continue

            laser_x = x + numpy.cos(theta + laser_angle)*laser_range
            laser_y = y + numpy.sin(theta + laser_angle)*laser_range

            laser_points.append((laser_x, laser_y))

            sim.draw_path([[x,y], [laser_x, laser_y]])
    else:
        print("OUTSIDE")

    # Render to display, catch user input, etc, sleep if in real-time mode, etc...
    sim.tick()