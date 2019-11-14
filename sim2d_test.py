from sim2d import Sim2D

sim = Sim2D()
sim.set_track('track.json')

while not sim.done:
    sim.update((3, 0.16))
    sim.sleep()
