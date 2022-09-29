# A-star-method


For A* search algorithm, because the robot is allowed to move left, right, up, down, and diagonal, we use
Chebysev distance as the heuristic function. Formally, the Chebysev distance between two positions, (x1,y1)
and (x2,y2), on the grid is defined as follows:

𝐷𝐶ℎ𝑒𝑏𝑦𝑠𝑒𝑣 = 𝑚𝑎𝑥(⌈𝑥2 − 𝑥1⌉, ⌈𝑦2 − 𝑦1⌉)


States: positions (Cartesian coordinates) of the robot on the grid.

Initial state: a start position of the robot (e.g., (1,1)).

Actions: left, right, up, down, and diagonal -> 8 possible movements.

Goal test: check if the robot is in the goal (e.g., (8,8)).

Path cost: Each standard movement cost is 1 but if the robot enters the barrier point, the cost is 200. Hence, the path cost is the sum of all movement costs.

Heuristic (for A* search): Chebysev distance between the current position of the robot and the goal.

The results for UCS search:
![UCS search](https://user-images.githubusercontent.com/53122798/192971488-568112f3-5b35-4fe2-a3cb-fef6027908fe.png)

The results for A* search:
![A* search](https://user-images.githubusercontent.com/53122798/192971624-946b09b2-75b8-4a41-bc40-e98d15a8d449.png)

