# 3D Motion Planning using A* and RRT Algorithms

Girish Krishnan | [GitHub](https://github.com/Girish-Krishnan) | [LinkedIn](https://www.linkedin.com/in/girk/)

This is a project for the **graduate-level course ECE 276B: Planning and Learning in Robotics** at UC San Diego. Since this is a course project, the code and report will not be publicly available. If you are interested in the implementation details, please reach out to me at gikrishnan@ucsd.edu.

---

![RRT Trees](./report_imgs/tree_4.png)


The objective of motion planning is to determine a feasible path for a robot or agent from a start position to a goal position while avoiding obstacles. Solving this problem is crucial for applications in robotic navigation, autonomous vehicles, and virtual simulations for games and training environments for reinforcement learning. The complexity of this task arises from the need to account for obstacles and other constrainsts within the environment, since the resulting path must be collision-free. In this project, we explore search-based and sampling-based techniques for solving the motion planning problem in three-dimensional (3D) environments. The algorithms I used are the __A\*__ algorithm and the __Rapidly-exploring Random Tree (RRT)__ algorithm. The A\* algorithm is a search-based algorithm that uses heuristics to efficiently find the shortest path in a discretized space. A* provides optimality and completeness, provided that the heuristic used is admissibale and consistent. The RRT algorithm is a sampling-based algorithm that builds a tree by randomly sampling points in the space and extending the tree toward these points. By implementing and comparing the results of these algorithms, this project highlights the strengths and weaknesses of these algorithms in different types of 3D environments.

## Problem Statement

The agent under consideration is a point robot. For a given map, the deterministic shortest path problem in continuous 3D space can be formally described as follows:

1. **Configuration Space**:
   - Let $\mathcal{C} \subset \mathbb{R}^3$ be the configuration space. This represents the space in which the robot can move, and is defined by the position of the robot $(x, y, z)$ in 3D space. Every map is bounded by a cuboid starting at $(x_{min}, y_{min}, z_{min})$ and ending at $(x_{max}, y_{max}, z_{max})$. Therefore, the configuration space is defined as:
     $$\mathcal{C} = \{ (x, y, z) \in \mathbb{R}^3 \; | \; x_{min} \leq x \leq x_{max}, \; y_{min} \leq y \leq y_{max}, \; z_{min} \leq z \leq z_{max} \}$$
   - The configuration space is divided into:
     - Obstacle space: $\mathcal{C}_{obs} \subset \mathcal{C}$, representing the regions occupied by obstacles. $\mathcal{C}_{obs} = \{ \mathbf{x} \in \mathcal{C} \; | \; \text{obstacle at position } \mathbf{x} \}$.
     - Free space: $\mathcal{C}_{free} = \mathcal{C} \setminus \mathcal{C}_{obs}$, representing the regions free of obstacles. $\mathcal{C}_{free} = \{ \mathbf{x} \in \mathcal{C} \; | \; \text{no obstacle at position } \mathbf{x} \}$.

2. **Start and Goal States**:
   - The start state $\mathbf{x}_s \in \mathcal{C}_{free}$ is the initial position of the robot.
   - The goal state $\mathbf{x}_\tau \in \mathcal{C}_{free}$ is the target position the robot must reach.

3. **Path**:
   - A path is defined as a continuous function $\rho: [0, 1] \rightarrow \mathcal{C}$.
   - The set of all possible paths is denoted by $\mathcal{P}$.

4. **Feasible Path**:
   - A feasible path $\rho$ is a continuous function $\rho: [0, 1] \rightarrow \mathcal{C}_{free}$ such that:
     $$\rho(0) = \mathbf{x}_s \quad \text{and} \quad \rho(1) = \mathbf{x}_\tau$$
   - The set of all feasible paths from $\mathbf{x}_s$ to $\mathbf{x}_\tau$ is denoted by $\mathcal{P}_{s,\tau}$.

5. **Cost Function**:
   - The cost function $J: \mathcal{P} \rightarrow \mathbb{R}_{\geq 0}$ assigns a non-negative real number to each path, representing its cost. In general, this could be the length of the path, time taken, energy consumed, etc. In this specific problem, the cost function is the length of the path.

6. **Motion Planning Problem**:
   - Given the free space $\mathcal{C}_{free}$, the obstacle space $\mathcal{C}_{obs}$, the start state $\mathbf{x}_s$, the goal state $\mathbf{x}_\tau$, and the cost function $J$, the objective is to find a feasible path $\rho^*$ such that:
     $$\rho^* = \arg \min_{\rho \in \mathcal{P}_{s,\tau}} J(\rho)$$
   - If no such path exists, the algorithm should report failure.

Mathematically, the problem can be summarized as finding:
$$\rho^* = \arg \min_{\rho \in \mathcal{P}_{s,\tau}} J(\rho)$$
subject to:
$$\rho(0) = \mathbf{x}_s, \quad \rho(1) = \mathbf{x}_\tau, \quad \rho(t) \in \mathcal{C}_{free} \; \forall t \in [0, 1]$$

This formulation captures the essence of the deterministic shortest path problem in continuous 3D space, considering obstacles and ensuring the path is feasible and optimal according to the given cost function. It is important to note that in this project, we plan a geometric path in 3D Euclidean space that may ignore the motion model of the robot. Hence, to execute the path, we may need a controller that takes the actual motion model of the robot into account. However, in this project, we focus only on the motion planning part.

## Technical Approach

This section describes the search-based planning algorithm that I implemented from scratch, which is A*. This section will also describe the sampling-based algorithm and the different parameters specified.

### Search-Based Planning Algorithm: A*

#### Discretization of the Environment

To apply the A* algorithm in continuous 3D space, we first need to discretize the environment. This involves defining a grid over the 3D space where each cell represents a potential position the robot can occupy.

- Let \( \mathbf{x}_s \) and \( \mathbf{x}_\tau \) be the start and goal states, respectively.
- Define a grid size \( g \) that determines the resolution of the discretized space. Each cell in the grid has dimensions \( g \times g \times g \).
- **Conversion from Continuous Space to Grid Cells**:
   $$\mathbf{c} = \left\lfloor \frac{\mathbf{x} - \mathbf{m}}{g} \right\rfloor$$
    where:
   - \( \mathbf{x} \) is the continuous coordinate.
   - \( \mathbf{m}  = (x_{min}, y_{min}, z_{min})\) is the minimum corner (origin) of the environment.
 - **Conversion from Grid Cells to Continuous Space**:
   \[
   \mathbf{x} = (\mathbf{c} + 0.5) \cdot g + \mathbf{m}
   \]
    where \( \mathbf{c} \) is the grid cell coordinate.

The environment is then represented as a set of discrete states \( \mathcal{S} \) where each state \( s \in \mathcal{S} \) corresponds to a unique cell in the grid. The free space in the discretized environment is \( \mathcal{S}_{free} \subseteq \mathcal{S} \).

#### A* Algorithm

The A* algorithm operates on the discretized space to find an optimal path from \( \mathbf{x}_s \) to \( \mathbf{x}_\tau \). The key components of the A* algorithm are:

1. **Graph Representation**:
   - The graph \( G = (V, E) \) consists of vertices \( V \) representing states in \( \mathcal{S}_{free} \) and edges \( E \) representing possible transitions between states.
   - We use a 26-connected grid approach in 3D space, where each state \( s \in V \) has up to 26 neighbors. These neighbors are defined by the Cartesian product of the sets \(\{-g, 0, g\}\) for each dimension, excluding the zero vector. Mathematically, the set of possible transitions (directions) \( \mathbf{d} \) is:
     \[
     \mathbf{d} = \{ (dx, dy, dz) \mid dx, dy, dz \in \{-g, 0, g\} \setminus (0, 0, 0) \}
     \]
     where \( g \) is the grid resolution.
   - Each edge \( e \in E \) has an associated cost, which is the Euclidean distance between the connected states \( s \) and \( s' \):
     \[
     c(s, s') = \| s - s' \|_2
     \]

2. **Cost Functions**:
   - The cost-from-start \( g(s) \) is the accumulated cost from the start state \( \mathbf{x}_s \) to the current state \( s \).
   - The heuristic cost \( h(s) \) is an estimate of the cost from the current state \( s \) to the goal state \( \mathbf{x}_\tau \). The Heuristic used is the Euclidean distance, which is a consistent and admissible heuristic because it satisfies the triangle inequality, and is an underestimate of the true cost when using a 26-connected grid.
     \[
     h(s) = \| s - \mathbf{x}_\tau \|_2
     \]
   - The total cost function \( f(s) \) combines the cost-to-come and the heuristic cost:
     \[
     f(s) = g(s) + \epsilon \cdot h(s)
     \]
     where \( \epsilon \geq 1 \) is a parameter that controls the balance between exploration and exploitation. In my implementation, I set \( \epsilon = 1 \).

3. **Algorithm Steps**:
   - **Initialization**: Initialize the open list \( \text{OPEN} \) with the start state \( \mathbf{x}_s \) and set \( g(\mathbf{x}_s) = 0 \). Initialize the closed list \( \text{CLOSED} \) as empty.
   - **Main Loop**:
     1. Extract the state \( s \) from \( \text{OPEN} \) with the lowest \( f \)-value.
     2. If \( s \) is the goal state \( \mathbf{x}_\tau \), reconstruct the path by tracing back from \( s \) to \( \mathbf{x}_s \) using parent pointers.
     3. Otherwise, expand \( s \) by generating its neighbors. For each neighbor \( s' \):
        - If \( s' \) is in \( \text{CLOSED} \), skip it.
        - Compute the tentative cost-to-come \( g_{\text{tent}}(s') = g(s) + c(s, s') \), where \( c(s, s') \) is the cost of transitioning from \( s \) to \( s' \).
        - If \( s' \) is not in \( \text{OPEN} \) or \( g_{\text{tent}}(s') < g(s') \):
          - Set \( g(s') = g_{\text{tent}}(s') \).
          - Set the parent of \( s' \) to \( s \).
          - Update \( f(s') = g(s') + \epsilon \cdot h(s') \) and add \( s' \) to \( \text{OPEN} \).
     4. Add \( s \) to \( \text{CLOSED} \).

#### Collision Checking in Continuous Space

Collision checking is performed in continuous space to ensure accuracy. For each potential move from state \( s \) to \( s' \), the algorithm checks if the line segment connecting \( s \) and \( s' \) intersects any obstacles.

1. **Line Segment Intersection with AABB**:
   - The environment consists of axis-aligned bounding boxes (AABBs) representing obstacles.
   - For a line segment defined by points \( \mathbf{p}_1 \) and \( \mathbf{p}_2 \), check if it intersects any AABB.

2. **Intersection Check**:
   - The intersection check involves testing the line segment against each face of the AABB.
   - For each face, solve the intersection equation:
     \[
     \mathbf{p}(t) = \mathbf{p}_1 + t(\mathbf{p}_2 - \mathbf{p}_1)
     \]
     where \( 0 \leq t \leq 1 \) and \(\mathbf{p}(t)\) represents a point on the line segment.
   - Check if the intersection point lies within the bounds of the AABB face.

#### Properties of the A* Algorithm

- **Optimality**: The A* algorithm is optimal if the heuristic \( h(s) \) is admissible (never overestimates the true cost) and consistent (satisfies the triangle inequality). Mathematically:
  - A heuristic \( h \) is **admissible** if:
    \[
    h(s) \leq \text{dist}(s, \tau) \quad \forall s \in V
    \]
  - A heuristic \( h \) is **consistent** if:
    \[
    h(s) \leq c(s, s') + h(s') \quad \forall s, s' \in V
    \]
  - In our implementation, the Euclidean distance heuristic is both admissible and consistent:
    \[
    h(s) = \| s - \tau \|_2
    \]
  - Furthermore, we set \( \epsilon = 1 \), ensuring that the resulting path is optimal.

- **Completeness**: The A* algorithm is complete, meaning it will find a path if one exists, provided the search space \( V \) is finite and the heuristic is admissible. If there exists at least one finite cost path from \( s \) to \( \tau \), then the A* algorithm will terminate with \( g(\tau) = \text{dist}(s, \tau) \), the shortest path length from \( s \) to \( \tau \).

- **Memory Efficiency**: The memory requirements of A* can be significant, as it needs to store all explored states and their associated costs. Specifically, the memory complexity of A* is \( O(|V|) \), where \( |V| \) is the number of states in the search space. This can be limiting in large environments, as each state must be stored in memory.

- **Time Efficiency**: The time complexity of A* depends on the efficiency of the priority queue operations and the number of nodes expanded. The main operations include inserting nodes into the priority queue, updating their priorities, and extracting the node with the minimum \( f \)-value. Using a binary heap, the time complexity is:
  \[
  O(|E| \log |V| + |V| \log |V|) = O((|E| + |V|) \log |V|)
  \]
  where \( |E| \) is the number of edges and \( |V| \) is the number of nodes. 

#### Summary of Key Ideas that enable the planner to scale to large maps and compute efficient solutions

1. **Heuristic Function**:
   - The heuristic function \( h(s) \) provides an estimate of the cost from the current state \( s \) to the goal state \( \tau \). By guiding the search towards the goal, the heuristic reduces the number of states that need to be explored.
   - A well-designed heuristic that closely approximates the true cost-to-go can significantly reduce the search space, making the algorithm more efficient.

2. **Priority Queue**:
   - A* uses a priority queue to manage the open list, which contains states that need to be expanded. The priority queue ensures that the state with the lowest estimated cost-to-go (i.e., \( f(s) = g(s) + \epsilon h(s) \)) is expanded first.
   - Efficient priority queue operations, such as insertion, update, and extraction of the minimum, are crucial for maintaining performance. Using a binary heap, the time complexity for these operations is \( O(\log |V|) \).

3. **Early Termination**:
   - The algorithm terminates as soon as the goal state \( \tau \) is reached and its cost-from-start \( g(\tau) \) is finalized. This prevents unnecessary exploration of other states, saving time and computational resources.

4. **Consistent Heuristic**:
   - Using a consistent heuristic ensures that the \( g \)-values of expanded states are non-decreasing. This property allows the algorithm to avoid re-expanding states, reducing the number of necessary operations and thus improving efficiency.

5. **Epsilon-Greedy Approach**:
   - By setting \( \epsilon = 1 \), we balance the exploration of the state space with the exploitation of the heuristic. This ensures that the search is both guided and thorough, allowing for the discovery of the optimal path while avoiding excessive computation.

6. **26-Connected Grid**:
   - The use of a 26-connected grid in 3D space allows for a finer resolution of possible transitions, providing a more detailed representation of the environment. This enhances the planner's ability to find feasible paths around obstacles.

7. **Collision Checking in Continuous Space**:
   - Collision checking is performed in continuous space to ensure accuracy. This prevents the planner from selecting paths that would appear feasible in a discretized space but would result in collisions when executed in the real world.

### Sampling-Based Planning Algorithm: RRT
 
#### Rapidly-exploring Random Tree (RRT)

The RRT algorithm is a sampling-based approach to motion planning, which constructs a tree incrementally by randomly sampling points in the configuration space. The tree rapidly explores large areas of the space, making it well-suited for high-dimensional and complex environments.

#### Algorithm Description

1. **Initialization**:
   - Start with an initial tree \( T \) rooted at the start state \( \mathbf{x}_s \).
   - Define the configuration space \( \mathcal{C} \) and the free space \( \mathcal{C}_{free} \subset \mathcal{C} \).

2. **Sampling**:
   - Randomly sample a point \( \mathbf{x}_{rand} \) in the configuration space \( \mathcal{C} \). This point is chosen uniformly at random.

3. **Nearest Neighbor**:
   - Find the nearest vertex \( \mathbf{x}_{near} \) in the tree \( T \) to the sampled point \( \mathbf{x}_{rand} \). This is done by minimizing the Euclidean distance:
     \[
     \mathbf{x}_{near} = \arg \min_{\mathbf{x} \in T} \| \mathbf{x} - \mathbf{x}_{rand} \|
     \]

4. **Steering**:
   - Move from \( \mathbf{x}_{near} \) towards \( \mathbf{x}_{rand} \) by a step size \( q \). The new point \( \mathbf{x}_{new} \) is given by:
     \[
     \mathbf{x}_{new} = \mathbf{x}_{near} + q \cdot \frac{\mathbf{x}_{rand} - \mathbf{x}_{near}}{\| \mathbf{x}_{rand} - \mathbf{x}_{near} \|}
     \]

5. **Collision Checking**:
   - Check if the path from \( \mathbf{x}_{near} \) to \( \mathbf{x}_{new} \) is collision-free. This involves verifying that the line segment connecting \( \mathbf{x}_{near} \) and \( \mathbf{x}_{new} \) does not intersect any obstacles.

6. **Tree Expansion**:
   - If the path is collision-free, add \( \mathbf{x}_{new} \) to the tree \( T \) with an edge from \( \mathbf{x}_{near} \) to \( \mathbf{x}_{new} \).

7. **Goal Checking**:
   - With a probability \( p \), check if \( \mathbf{x}_{new} \) is within a certain distance \( r \) from the goal state \( \mathbf{x}_\tau \). If it is, connect \( \mathbf{x}_{new} \) to \( \mathbf{x}_\tau \) and terminate the search.

8. **Iterate**:
   - Repeat the sampling, nearest neighbor, steering, collision checking, and tree expansion steps until the maximum number of samples \( N \) is reached or a path to the goal is found.

The RRT algorithm can be summarized as follows:

\[
\begin{aligned}
&\text{Initialize:} \\
&\quad T \leftarrow \{ \mathbf{x}_s \} \\
&\text{while } \text{not reached goal and } |T| < N \text{ do} \\
&\quad \mathbf{x}_{rand} \leftarrow \text{Sample}(\mathcal{C}) \\
&\quad \mathbf{x}_{near} \leftarrow \text{NearestNeighbor}(T, \mathbf{x}_{rand}) \\
&\quad \mathbf{x}_{new} \leftarrow \text{Steer}(\mathbf{x}_{near}, \mathbf{x}_{rand}, q) \\
&\quad \text{if } \text{CollisionFree}(\mathbf{x}_{near}, \mathbf{x}_{new}) \text{ then} \\
&\qquad T \leftarrow T \cup \{ \mathbf{x}_{new} \} \\
&\qquad \text{if } \text{withinDistance}(\mathbf{x}_{new}, \mathbf{x}_\tau, r) \text{ then} \\
&\quad \quad \text{Connect } \mathbf{x}_{new} \text{ to } \mathbf{x}_\tau \\
&\quad \quad \text{end if} \\
&\quad \text{end if} \\
&\text{end while}
\end{aligned}
\]

#### Parameters and Their Selection

- **Step Size (\( q \))**:
  - Determines how far the tree extends towards the sampled point in each iteration.
  - A smaller step size allows for finer resolution and better handling of narrow passages but increases the number of iterations needed.

- **Sampling Radius (\( r \))**:
  - Defines the distance within which the algorithm checks for a connection to the goal state.
  - A larger radius increases the chances of connecting to the goal but may lead to suboptimal paths.

- **Maximum Samples (\( N \))**:
  - The maximum number of samples to be taken before the algorithm terminates.
  - Larger values increase the probability of finding a path but also increase computation time.

- **Goal Bias (\( p \))**:
  - The probability of directly sampling the goal state to accelerate the search.
  - Higher goal bias can speed up the search in structured environments but may lead to less thorough exploration in unstructured environments.

#### Mathematical Justification

- **Exploration and Convergence**:
  - The random sampling ensures that the tree explores the entire free space \( \mathcal{C}_{free} \).
  - As the number of samples \( N \) increases, the probability of finding a feasible path approaches one.

- **Path Optimality**:
  - While basic RRT is not guaranteed to find the optimal path, the RRT* variant includes a re-wiring step to improve path quality.
  - RRT* ensures asymptotic optimality, meaning the solution improves with the number of samples.

#### Implementation Details

I used the `rrt_algorithms` package from [https://github.com/motion-planning/rrt-algorithms/tree/develop](https://github.com/motion-planning/rrt-algorithms/tree/develop) to implement the RRT algorithm. The choice of parameters used varied depending on the map under testing. This is because some maps (such as the maze) required a smaller step size to navigate through narrow passages, while other maps (such as the single cube) could use a larger step size for faster exploration.

| Map Name | Step Size (\( q \)) | Sampling Radius (\( r \)) | Maximum Samples (\( N \)) | Goal Bias (\( p \)) |
|----------|---------------------|--------------------------|--------------------------|-------------------|
| Single Cube     | 0.2                 | 0.05                      | 10000                     | 0.1               |
| Maze | 0.2               | 0.05                      | 100000                     | 0.1               |
| Flappy Bird | 0.2               | 0.05                      | 10000                     | 0.1               |
| Monza | 0.2            | 0.05                      | 500000                     | 0.1               |
| Window | 0.2             | 0.05                      | 10000                     | 0.1               |
| Tower | 0.2           | 0.05                      | 10000                     | 0.1               |
| Room | 0.2           | 0.05                      | 10000                     | 0.1               |

The step size, sampling radius, and goal bias were kept constant across all maps, while the maximum number of samples varied based on the complexity of the map. For the Monza map, which requires a fairly constrained path, a larger number of samples was used to ensure thorough exploration.
 
## Results

This section compares the performance of A* and RRT algorithms. 


### Efficiency Comparison and Quality of Computed Paths

Here is a summary of the running times and path lengths for each map:

 Test Case        | Algorithm | Path Length | Running Time (sec) |
|------------------|-----------|-------------|--------------------|
| Single Cube      | A*        | 7           | 0.0125             |
|                  | RRT       | 8           | 0.0089             |
| Maze             | A*        | 79          | 17.9174            |
|                  | RRT       | 119         | 37.6400            |
| Flappy Bird      | A*        | 25          | 4.0500             |
|                  | RRT       | 37          | 0.5529             |
| Monza            | A*        | 77          | 2.6216             |
|                  | RRT       | 106         | 60.0000            |
| Window           | A*        | 26          | 9.8044             |
|                  | RRT       | 31          | 0.2070             |
| Tower            | A*        | 32          | 7.1700             |
|                  | RRT       | 42          | 1.5987             |
| Room             | A*        | 12          | 0.5876             |
|                  | RRT       | 21          | 0.2084             |

### Images of Generated Paths: Qualitative Comparison

This section presents the paths generated by the A* and RRT algorithms for each map. The paths are visualized in 3D space, showing the start state, goal state, obstacles, and the path connecting the two states. The images provide a qualitative comparison of the paths generated by the two algorithms in different environments.  

For each environment, I have put two or more images of exactly the same setting, but with different camera angles. This is to provide a clearer view of the path generated by the algorithms.

The red path represents the path generated by the A* algorithm, while the blue path represents the path generated by the RRT algorithm.

#### Single Cube

||||
|:---:|:---:|:---:|
|![Single Cube](./report_imgs/single_cube_1.png)|![Single Cube](./report_imgs/single_cube_2.png)|![Single Cube](./report_imgs/single_cube_3.png)|

#### Maze

|||
|:---:|:---:|
|![Maze](./report_imgs/maze_1.png)|![Maze](./report_imgs/maze_2.png)|

#### Flappy Bird

|||
|:---:|:---:|
|![Flappy Bird](./report_imgs/flappy_bird_1.png)|![Flappy Bird](./report_imgs/flappy_bird_2.png)|

#### Monza

|||
|:---:|:---:|
|![Monza](./report_imgs/monza_1.png)|![Monza](./report_imgs/monza_2.png)|

#### Window

||||
|:---:|:---:|:---:|
|![Window](./report_imgs/window_1.png)|![Window](./report_imgs/window_2.png)|![Window](./report_imgs/window_3.png)|

#### Tower

||||
|:---:|:---:|:---:|
|![Tower](./report_imgs/tower_1.png)|![Tower](./report_imgs/tower_2.png)|![Tower](./report_imgs/tower_3.png)|

#### Room

|||
|:---:|:---:|
|![Room](./report_imgs/room_1.png)|![Room](./report_imgs/room_2.png)|

### Discussion of Quality of Paths

For each environment, it is clear that the path from A* is more efficient as it has a shorter length compared to the path from RRT. Furthermore, the path from A* tends to be more direct and smoother, while the path from RRT is noisier due to the random exploration. This is expected since A* is a search-based algorithm that guarantees optimality, while RRT is a sampling-based algorithm that may not find the optimal path. In terms of computational efficiency, however, there are differences.

### Discussion of the Number of Considered Nodes (A*) and Number of Samples (RRT)

The table below shows the number of nodes considered by the A* and number of points sampled by RRT algorithms for each map:

| Test Case        | Algorithm | Number of Nodes Considered (A*) or Samples (RRT) |
|------------------|-----------|----------------------------|
| Single Cube      | A*        | 12                         |
|                  | RRT       | 73                         |
| Maze             | A*        | 8938                       |
|                  | RRT       | 84317                      |
| Flappy Bird      | A*        | 3456                       |
|                  | RRT       | 3031                       |
| Monza            | A*        | 3056                       |
|                  | RRT       | 309952                     |
| Window           | A*        | 5550                       |
|                  | RRT       | 1277                       |
| Tower            | A*        | 2446                       |
|                  | RRT       | 7055                       |
| Room             | A*        | 286                        |
|                  | RRT       | 1312                       |


Again, we can see that in maps with a regular structure and constricted pathways (e.g. maze and Monza), the A* algorithm considers fewer nodes than the RRT algorithm. This is consistent with the running time results, where A* is faster in these environments.

### Discussion of the Effect of Different Parameters

#### Step Size (\( q \))

A smaller step size in the RRT algorithm results in finer resolution and more detailed exploration of the space. This can be beneficial in environments with narrow passages or complex structures, as it allows the algorithm to navigate through tight spaces. However, a smaller step size may require more iterations to find a path, leading to longer running times. In contrast, a larger step size can speed up exploration but may miss potential paths through narrow regions.

#### Sampling Radius (\( r \))

The sampling radius in RRT determines the distance within which the algorithm checks for a connection to the goal state. A larger radius increases the chances of connecting to the goal quickly but may result in suboptimal paths. In contrast, a smaller radius requires more exploration to find a path to the goal, which can be computationally expensive.

#### Maximum Samples (\( N \))

The maximum number of samples in RRT controls the number of iterations the algorithm performs before terminating. A larger number of samples increases the probability of finding a path but also increases computation time. In complex environments, a higher number of samples may be necessary to thoroughly explore the space and find a feasible path. An example of this is the Monza map, where a larger number of samples was required to navigate through the constricted region.

#### Goal Bias (\( p \))

The goal bias in RRT determines the probability of directly sampling the goal state to accelerate the search. A higher goal bias can speed up the search in structured environments with clear paths to the goal. However, in unstructured environments with obstacles, a high goal bias may lead to less thorough exploration and suboptimal paths. Therefore, the goal bias should be chosen based on the complexity of the environment and the desired trade-off between exploration and exploitation.

#### $\epsilon$ in the weighted A* algorithm

The parameter $\epsilon$ in the weighted A* algorithm controls the balance between exploration and exploitation. A higher value of $\epsilon$ prioritizes exploration, leading to a more thorough search of the state space. However, this may result in longer running times and suboptimal paths. For simplicity, I set $\epsilon = 1$ in my implementation, since this provides an optimality guarantee.

### Visualization of Trees Computed by RRT

![RRT Trees](./report_imgs/tree_1.png)

![RRT Trees](./report_imgs/tree_2.png)

![RRT Trees](./report_imgs/tree_3.png)

![RRT Trees](./report_imgs/tree_4.png)

![RRT Trees](./report_imgs/tree_5.png)

### Discussion of any Interesting Details on the Performance of the Two Algorithms

For all paths except the Maze and Monza, the RRT algorithm is faster than the A* algorithm. This is because the RRT algorithm rapidly explores the configuration space by randomly sampling points, making it more efficient in unstructured environments. 

In contrast, A* requires a systematic search through the grid space, which can be computationally intensive. A* works better in the maze environment, where the regular grid-like structure allows for efficient exploration. The same applies with the Monza environment, where the agent must move through a very constricted region in order to reach the goal. In the case of the Monza environment, it is very unlikely for a random sample to bypass the constricted region, which is why the RRT algorithm takes a longer time to find a path.

The noise in the RRT path is due to the random sampling and steering process, which can lead to suboptimal paths. This can be resolved by using more sophisticated variants of RRT, such as RRT*, which includes a re-wiring step to improve path quality. However, the basic RRT algorithm is still effective for rapid exploration and path generation in complex environments.

## Conclusion

In conclusion, the A* and RRT algorithms provide complementary approaches to motion planning in 3D environments. A* is a search-based algorithm that guarantees optimality and completeness, making it suitable for finding the shortest path in a discretized space. On the other hand, RRT is a sampling-based algorithm that rapidly explores the configuration space, making it well-suited for high-dimensional and complex environments. By implementing and comparing these algorithms on various maps, we have demonstrated their strengths and weaknesses in different scenarios. A* is particularly effective in structured environments with known grid resolutions, while RRT excels in unstructured environments where random sampling can efficiently explore the space. The choice of algorithm depends on the specific requirements of the task, such as optimality, exploration speed, and computational resources. Therefore, this project provides valuable insights into motion planning techniques and their applications in real-world scenarios.