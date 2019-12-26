from racecar_2d import *
import numpy

sim = Sim2D(render=True, use_ros=True)
sim.frequency = 25
sim.set_track('track.json')


init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0, 0.0))
init_pose.append((sim.race_line[20][0], sim.race_line[20][1], 0.0, 0.0, 0.0))

sim.add_player('../images/Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
sim.add_player('../images/Acura_NSX_yellow.png', 4.4, unicycle_model, init_pose[1])

player = sim.players[0]
player.add_sensor('laser', SensorLaser(player, sim, (-np.pi/6.0, 0.0, np.pi/6.0), 20.0, 0.1))

inside, last_good_index_hint = sim.is_inside_track(player.current_state[0:2])
if not inside:
    raise Exception("Player must be initialized inside the track")

# Set to manual mode by default (change with M key while running)
sim.is_manual = True
while not sim.done:
    # Autonomous mode: give acceleration and steering = (0.0,0.0) to player 0 (the first and only in this case)
    sim.update_player(0, (0.0,0.0)) 
    sim.update_player(1, (0.0,0.0)) 
    
    # This is the current state after update
    (x,y,theta,v) = (player.current_state[0], player.current_state[1], player.current_state[2], player.current_state[3])

    # Verify if the vehicle is inside the track, return an index_hint that is used to make the sensor simulation more efficient. Technically, this is the index of the Delaunay triangle the vehicle is in
    inside, index_hint = sim.is_inside_track(player.current_state[0:2], last_good_index_hint, 20)

    # If you want to simulate sensors, do like the following. Otherwise, you can comment out this part, the vehicle pose has already been updated in update_player
    if inside:
        last_good_index_hint = index_hint
        readings = player.simulate('laser', index_hint=index_hint, interval=20)
        [s, d] = sim.get_track_coordinates(player.current_state[0:2])
        state = np.array([readings[0][1], readings[1][1], readings[2][1], s, d, player.current_state[0], player.current_state[1], player.current_state[2], player.current_state[3]]).reshape((-1, 9))

    # Render to display, catch user input, etc, sleep if in real-time mode, etc...
    sim.tick()