from racecar_2d import *
import numpy

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('track.json')


init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0))

sim.add_player('../images/Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
player = sim.players[0]
player.add_sensor('laser', SensorLaser(player, sim, (-np.pi/6.0, 0.0, np.pi/6.0), 20.0, 0.1))

inside, last_good_index_hint = sim.is_inside_track(player.current_state[0:2])
if not inside:
    raise Exception("Player must be initialized inside the track")

sim.is_manual = True
while not sim.done:
    sim.update_player(0, (0.0,0.0))
    inside, index_hint = sim.is_inside_track(player.current_state[0:2], last_good_index_hint, 20)
    if inside:
        last_good_index_hint = index_hint
        readings = player.simulate('laser', index_hint=index_hint, interval=20)
        [s, d] = sim.get_track_coordinates(player.current_state[0:2])
        state = np.array([readings[0][1], readings[1][1], readings[2][1], s, d, player.current_state[3]]).reshape((-1, 6))
        print(state)

    sim.tick()

    