# PRM-AStar-PathPlanning
## Introduction
This is a path-planning project, in which the goal is to create a Probability Roadmap (PRM) in an enclosed 2D environment, perform A* Search to find optimal path to goal, and post-process the path to reduce the overall cost.

## How to Use

### Installation and Setup
    git clone https://github.com/JHoeLee/PRM-AStar-PathPlanning.git
    cd PRM-AStar-PathPlanning
    pip install -r requirements.txt

### Run the code
    python PathPlanning.py --num_samples 3000 --connection_radius 0.5 --seed 12345

## Framework
### Step 1: Generate Random Samples in Environment
Random points in the environment were selected as nodes for the roadmap. Points within the obstacles are avoided. **Default num_samples = 3000.**

![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/19904b97-e1ce-4225-9ec7-8b697f86ab4a)

### Step 2: Build Roadmap
Nearest neighbours of each node within a specified radius (**default connection_radius = 0.5**) are acquired with sklearn.neighbors. Nodes and its neighbours are joined together if they do not intersect with obstacles to form a complete roadmap.

![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/228c4de2-fffe-4cee-95f6-b2c6e5d43474)

### Step 3: Perform A* Search
Heuristic cost is the distance between each node and the goal. Node with the lowest f-score(distance so far(g-score) + heuristic) is selected as current node. For each neighbour, if tentative g_score is lower than current g-score of neighbour, the neighbour is pushed into the open set with its updated f-score. When the current node is the goal node, the optimal path is reconstructed. **Default seed = 4**, change its value to see other environments.

![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/99269129-f76d-4194-9bde-8fce10f98294)


If no path is found, the roadmap and its nodes will be displayed.

![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/72e63589-2604-4def-8bdb-59ee92478783)

### Step 4: Post-Processing of Path
The path is improved through 2 steps. First, unnecessary nodes are removed to ensure a straight route when needed. Then, random points in both directions are selected from a node and potentially joined, creating smoother curves at sharp path edges. This reduces total distance without sacrificing speed. **Orange - before post-processing, Green - after post-processing**. The cost before and after this process is shown for comparison.

![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/71f02251-4c3a-41ad-9317-492371680d8c)
![image](https://github.com/JHoeLee/PRM-AStar-PathPlanning/assets/111511618/7dbf5a59-9859-4716-9d42-b441d39cf72b)



 
