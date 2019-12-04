from racecar_2d import *
import numpy

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('track.json')

init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0, 0.0))
init_pose.append((sim.race_line[10][0], sim.race_line[10][1], 0.0, 0.0))
init_pose.append((sim.race_line[20][0], sim.race_line[20][1], 0.0, 0.0))

sim.add_player('Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])

while not sim.done:
    sim.update_player(0, (1,0.1))
    
    # first_player_state = sim.players[0].current_state
    # print("x: {:.2f}, y: {:.2f}, theta: {:.2f}, v: {:.2f}".format(first_player_state[0], first_player_state[1], first_player_state[2], first_player_state[3]))

    sim.tick()

    point = (sim.players[0].current_state[0], sim.players[0].current_state[1])
    if not sim.is_inside_track(point):
        print("OUTSIDE!")
    else:
        print("INSIDE!")