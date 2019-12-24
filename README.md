# Racecar 2D

This is a simple python racing car simulator using the 2D pygame library.

## Features
- Synchronous
- Interfaceable with ROS [TODO add bridge to the library]
- Multi-vehicle
- Tracks are loaded from standard JSON used for Roborace (inside, outside and racing line) [TODO: use center line as a reference optionally]
- State: (x,y,theta,v)
- Local coordinates: (s,d) --> curvilinear abscissa and transversal distance along the racing line
- Control modes: 
    - Manual --> keyboard arrows
    - Autonomous --> control are acceleration and steering
    - Kinematic trajectory following --> follow a given trajectory (x,y,v) regardless of its dynamic feasibility

## Requirements

Tested on Python 3.6.6 but it should work on Python 2.7 also. [CHECK]

It is **very** recommended to use virtual environments to keep the installation environments isolated:

**On Windows:**
- Recommended: install [Anaconda](https://www.anaconda.com/distribution/)
- `pip install virtualenv`
- `virtualenv <environment_name>` --> Creates a folder named <environment_name>
- `<environment_name>/Scripts/activate` --> Activates the virtual environment
- `deactivate` --> If you want to deactivate the current environment

**On Linux:**
- `pip install virtualenv`
- `virtualenv <environment_name>` --> Creates a folder named <environment_name>
- `source ./<environment_name>/bin/activate` --> Activates the virtual environment
- `deactivate` --> If you want to deactivate the current environment
 


## Build

If using virtual environments, activate the one of your choice (see Requirements)

```
git clone https://gitlab.com/roboteam-italia/simulation/racecar_2d
cd racecar_2d
pip install .
```

## Usage

See tests/sim2d_test.py for a basic example. Launch it with 

`python tests/sim2d_test.py`

### Initialization

**Import:**
- `from racecar_2d import *`

**Construction:**
- `sim = Sim2D()` with optional arguments: 
    - render=True/False: if the simulator should display the simulation on screen (default=True)
    - sort_triangles=True/False: abilitate if you want to use the information of the vehicle being inside or outside the track (default=True)
    - real_time=True/False: disabilitate if you don't want the simulator to run at a given frequency and trigger the updates manually (default=True)

**Setup:**
- `sim.frequency = 25`: set simulator frequency (in Hz, only for real time mode). This is both the rendering frequency and the state update frequency
- `sim.set_track(<track_name>.json)`: set the json track. Right now racing line is required in JSON as a reference for the curvilinear abscissa
- `sim.add_player(image_path, vehicle_length, dynamic_model, initial_state)`: add a player, which will be visualized with an image found in **<image_path>**, of length **<vehicle_length>** (meters), model **<dynamic_model>**, initial state (x,y,theta,v) = **<initial_state>**. <dynamic_model> is a function: see sim2d.py, unicycle_model for an example. This assign a player index starting from 0, in order of player creation.
- `sim.players[<index>].add_sensor('sensor_name', <Sensor>(...))`: add a sensor named 'sensor_name' of type <Sensor> to the <index>-th player. See **Sensors** below.

and, only if in **kinematic trajectory following mode**, one of the following:

`sim.set_trajectory(player_index, array of (x,y,v) points)`

OR

`sim.set_csv_trajectory(player_index, csv_file)`: CSV file containing a list of x,y,v points

### Agents update
This is a synchronous simulator, which means that you have to trigger each frame update manually. This is typically done like this (again, see tests/sim2d_test.py as a reference):

**Autonomous mode**
```python
while not sim.done:
    sim.update_player(player_index, (acceleration_value,steering_value))
    sim.tick()
```

**Kinematic trajectory following mode**
```python
while not sim.done:
    sim.update_trajectory(player_index)
    sim.tick()
```

### UI and Rendering
If render is enabled (see Construction), tick() function renders the simulator to display. If you want, you can draw several stuff on the display, using draw_path and draw_point functions (see sim2d.py)

While active, the simulator reacts to the following user inputs:
- [0-9]: focus camera on player with correspondent index
- M: set manual mode and control car with arrow keys
- F: detach camera from vehicle, move freely in the map while holding mouse 1 button

### Sensors
You can always access the following data of the vehicle:

```python
- (x,y,theta,v) = sim.players[player_index].current_state[0:4]
- (s,d) = sim.get_track_coordinates(x,y) : local vehicle coordinates (curvilinear abscissa and transversal distance from racing line)
```

If you want, you can add a **laser sensor** to the <player_index>-th player by initializing the sensor this way (write this before entering the simulator loop (see tests/sim2d_test.py):

```
sim.players[<index>].add_sensor('laser', SensorLaser(sim.players[player_index], sim, (-3.14/6.0, 0.0, 3.14/6.0), 20.0, 0.1))
```

Here, we have declared a laser sensor with 3 rays, at -30°, 0°, 30° with respect to the fron of the car (positive counter-clockwise), range of 20 meters and 0.1 meters of resolution.
The actual simulation is called within the simulator loop by calling 

```
readings = sim.players[player_index].simulate('laser', index_hint=index_hint, interval=20)
```

Here, <index_hint> is used for efficiency reasons and represents an estimate of the index of the Delaunay triangle the vehicle is in, while <interval> is the number of Delaunay triangles to check around the hint.
In practice, index_hint can be obtained from the is_inside_track function. Again, see tests/sim_2d.py for an example of this. 

Readings contains, for each laser ray (3 in this case), a pair (angle, range), that indicates the angle of the current laser (so -30°, 0° or 30° in our example), and the range is the distance from the vehicle to the obstacle.

### ROS interface

TODO