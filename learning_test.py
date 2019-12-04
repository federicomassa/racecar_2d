from racecar_2d import *
from ddpg import *
import numpy

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('track.json')

sim.add_player('Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
rand_index = np.random.randint(0, len(sim.race_line))
sim.reset(0, rand_index)

ddpg = DDPG(6, 2)

counter = 0
while not sim.done:
    if counter % 120 == 0:
        rand_index = np.random.randint(0, len(sim.race_line))
        sim.reset(rand_index)

    sim.update_player(0, (1,0.1))
    
    # first_player_state = sim.players[0].current_state
    # print("x: {:.2f}, y: {:.2f}, theta: {:.2f}, v: {:.2f}".format(first_player_state[0], first_player_state[1], first_player_state[2], first_player_state[3]))

    sim.tick()

    counter += 1