
# FCND Motion Planning
Udacity Flying Car Nanodegree Motion Planning

This is the second project on [Udacity's Flying Car Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787). It consists of planning and executing a trajectory of a drone in an urban environment. Built on top of the event-based strategy utilized on the first project, the complexity of path planning in a 3D environment is explored. The code communicates with [Udacity FCND Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) using [Udacidrone](https://udacity.github.io/udacidrone/) API.

# Prerequisites
To run this project, you need to have the following software installed:

- [Miniconda](https://conda.io/miniconda.html) with Python 3.6. 
- [Udacity FCND Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) the latest the better.

# Project description

The following are the main code used on the project:

- [motion_planning_colinearty.py](./motion_planning_colinearty.py): This version extends the provided implementation with the following features:
  - The global home location is read from the [colliders.csv](./colliders.csv) file.
  - The calculated path is pruned with a collinearity function to eliminate unnecessary waypoints.
- [motion_planning_medial_axis](./motion_planning_medial_axis.py):this version uses a medial_axis graph instead of a grid
- [motion_planning_veronoi](./motion_planning_veronoi.py):this version uses a veronoi graph.
- [motion_planning_veronoi_out](./motion_planning_veronoi_out.py):this is the same as motion_planning_veronoi but the planning process happen before the drone started car it took to much time to search throught the graph and sometime after finish searching the simulateur disconnect.
- [motion_planning_probabilistic](./motion_planning_probabilistic.py):this version it implement the probabilistic roadmap.
- [motion_planning_probabilistic_out](./motion_planning_probabilistic_out.py):this is a probabilistic roadmap but the planning process happen before the drone started
- [planning_utils.py](./planning_utils.py): It provides support for the features mention above, also extends the A* search algorithm to include diagonals actions and other features like A* search for graph and create voronoi graph.


# Run the Project

To run the code you need to change to the repo directory and create the conda environment with the following command:

```
conda env create -f environment.yml
```

**Note**: This environment configuration is provided by Udacity at the [FCND Term 1 Starter Kit repo](https://github.com/udacity/FCND-Term1-Starter-Kit).

Activate your environment with the following command:

```
source activate fcnd
```

Start the drone simulator

Select the **Motion Planning** project

Now is time to run the code, for the A* grid implementation:
```
python motion_planning_colinearty.py 
```

For the graph implementation:

```
python motion_planning_medial_axis.py 
python motion_planning_veronoi.py
python motion_planning_veronoi_out.py
python motion_planning_probabilistic.py
python motion_planning_probabilistic_out.py

```

