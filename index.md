---
class: wide
---

<!-- # Chemistry-Inspired Pattern Formation with Robotic Swarms -->
<!-- Dynamic Gibbs random field applied to swarm morphogenesis abstracting chemical binding mechanism. -->

## Abstract
<p align="justify">Self-organized emergent patterns can be widely seen in particle interactions producing complex structures such as chemical elements and molecules. Inspired by these interactions, this work presents a novel stochastic approach that allows a swarm of heterogeneous robots to create emergent patterns in a completely decentralized fashion and relying only on local information. Our approach consists of modeling the swarm configuration as a dynamic Gibbs Random Field (GRF) and setting constraints on the neighborhood system inspired by chemistry rules that dictate binding polarity between particles. Using the GRF model, we determine velocities for each robot, resulting in behaviors that lead to the emergence of patterns or shapes. Simulated experiments show the versatility of the approach in producing a variety of patterns, and experiments with a group of physical robots show the feasibility in potential applications.</p>

## Media

<div class="video-container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/y7ls4djT3W4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
  </div>

## Publication
- Accepted for publication in the 2022 IEEE RA-L and presented at IROS
- [Pre-print](https://arxiv.org/abs/2206.03388)

## Dependecies

- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- [Gazebo 11](http://gazebosim.org/download)
- [Hero Common](https://github.com/verlab/hero_common)

## Installation

-   Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/rezeck/gibbs_swarm_pattern_formation.git
$ git clone https://github.com/verlab/hero_common.git # for gazebo simulations and real robots
```

-   Fixing package dependencies:

```sh
$ cd ~/catkin_ws
$ rosstack profile && rospack profile
$ rosdep install --from-paths src/gibbs_swarm_pattern_formation --ignore-src -r -y
```

## Usage
### Numeric Simulations
Use the configuration files to set the simulator parameter. For example,
```sh
$ cat sim.yaml
# Simulation Visualization Parameter
gui: true                  # Show GUI Simulation
show_velocity: true        # Show velocity vector
show_sensing: false        # Show sensing range
show_id: false             # Show robots ids
show_obstacles: false      # Show obstacles detection
show_bounding: true        # Show bounding
# Environment Parameter
world: 10                  # World size (10 x 10)
seed: 1                    # Random Seed
iterations: 20000          # Maximum number of iterations
sensing: 0.8               # Maximum sensing range
safezone: 0.8              # Safezone range
velocity: 1.0              # Max velocity
dt:  0.02                  # Sim steps
babystep: false            # Step by step simulation (use keyboard to step the simulation)
log: true                  # Enable logging
```

Use another configuration file to set the swarm properties. For example,
```sh
$ cat oxocarbons.yaml
# Heterogeneity
robots:
  hydrogen:                 # Hygrogen-like robot
    type: 0                 # Type of the robot (should assume order by charge)
    mass: 1.0               # Atomic mass
    radius: 53.0            # Atomic radius
    charge: 5.0             # Eletrical charge
    amount: 0               # Number of robots
    bound: 1                # Maximum number of bond
    orbitals: [1, 1, 1, 1]  # Number of robots per orbitals, e.g, [hydrogen, oxygen, nitrogen, carbon] *order by type

  oxygen:                   # Oxygen-like robot
    type: 1                 # Type of the robot (should assume order by charge)
    mass: 16.0              # Atomic mass
    radius: 60.0            # Atomic radius
    charge: 15.00           # Eletrical charge
    amount: 130             # Number of robots
    bound: 2                # Maximum number of bond
    orbitals: [0, 2, 0, 1]  # Number of robots per orbitals, e.g, [hydrogen, oxygen, nitrogen, carbon] *order by type

  nitrogen:                 # Nitrogen-like robot
    type: 2                 # Type of the robot (should assume order by charge)
    mass: 14.0              # Atomic mass
    radius: 65.0            # Atomic radius
    charge: 10.00           # Eletrical charge
    amount: 0               # Number of robots
    bound: 3                # Maximum number of bond
    orbitals: [3, 0, 0, 1]  # Number of robots per orbitals, e.g, [hydrogen, oxygen, nitrogen, carbon] *order by type

  carbon:                   # Carbon-like robot
    type: 3                 # Type of the robot (should assume order by charge)
    mass: 12.0              # Atomic mass
    radius: 70.0            # Atomic radius
    charge: 10.00           # Eletrical charge
    amount: 50              # Number of robots
    bound: 2                # Maximum number of bond
    orbitals: [0, 2, 0, 0]  # Number of robots per orbitals, e.g, [hydrogen, oxygen, nitrogen, carbon] *order by type
```

Finally, create a launch file that runs a simulation. For example,
```sh
$ cat oxocarbon.launch
<launch>
    <group ns="$(anon grf_controller)">
        <rosparam command="load" file="$(find gibbs_swarm_pattern_formation)/config/sim.yaml" />
        <!-- Select a swarm -->
        <rosparam command="load" file="$(find gibbs_swarm_pattern_formation)/config/oxocarbon.yaml" />
        <node name="grf_controller" pkg="gibbs_swarm_pattern_formation" type="gibbs_swarm_pattern_formation_node" output="screen" required="true"></node>
    </group>

</launch>   
```

Run using the command:
```sh
$ roslaunch gibbs_swarm_pattern_formation oxocarbon.launch
```

### Gazebo Simulations
Here it is necessary to have the hero common package installed. 
After installing the package, run the following commands to run the algorithm for forming chains.
```sh
$ roslaunch hero_gazebo gazebo_bringup.launch
$ roslaunch hero_gazebo env_spawn_bridge.launch
$ rosrun gibbs_swarm_pattern_formation gibbs_swarm_pattern_formation_gazebo
```

### Real Robots
Here it is necessary to have the hero common package installed and also the physical robots. 
Run these commands:
```sh
$ roslaunch hero_bringup hero_bringup.launch
$ rosrun gibbs_swarm_pattern_formation gibbs_swarm_pattern_formation_real
```
