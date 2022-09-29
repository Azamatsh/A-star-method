#Author: Nguyen Anh Tu, Dinh-Mao Bui
#Topic: finding a shortest path for a robot vacuum in an 8x8 grid board

def heuristic(pos, goal):
    # Compute the Chebyshev distance from current position to the goal. 
    # Use the Chebyshev distance as a heuristic which allows our robot 
    # to move one square either adjacent or diagonal.
    """
    Args: 
        pos: coordinate of current node
        goal: coordinate of goal node
    Return:
        Chebyshev distance from current position to the goal
    """
    return max(abs(pos[0] - goal[0]),abs(pos[1] - goal[1]))

def get_neighbors(pos):
    # Get the positions of neighbors of current node on the grid board
    """
    Args: 
        pos: coordinate of current node
    Return:
        neighbors: list of coordinates of neighbors
    """
    neighbors = []
    for ix, iy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
        nx = pos[0] + ix
        ny = pos[1] + iy
        if nx < 1 or nx > 8 or ny < 1 or ny > 8:
            continue
        neighbors.append((nx,ny))
    return neighbors

def move_cost(next_node, barrier):
    # Compute the movement cost when moving to next node. 
    # If the next node is the point on the barrier, the cost is extremely high (i.e., 200). 
    # Otherwise, just set to the standard cost = 1
    """
    Args: 
        next_node: coordinate of next node
        barrier: list of coordinates of the points belonging to the barrier
    Return:
        movement cost
    """
    for bar_pos in barrier:
        if next_node in bar_pos:
            return 200
    return 1 



def UCSearch(start, goal, barrier):
    # Implementation of Unoform-Cost Search (2 points)
    """
    Args: 
        start: coordinate of start node
        goal: coordinate of goal node
        barrier: list of coordinates of the points belonging to the barrier
    Return:
        path: list of coordinates of the points belonging to optimal path
        expandedNodes: set of coordinates of expanded nodes
        cost: path cost at the goal state
    """
    ######################################################################################
    
    # Firstly, create a dictonary that stores the path cost so far to reach each node from 
    # the start position. For example, G ={(3, 3): 2} depicts that path cost of (3,3) is 2.
    
    G = {} 
    
    #Initialize value at starting position
    G[start] = 0
    
    # Create two sets. One set (i.e. expandedNodes) is used to store the expanded nodes, 
    # and another set (i.e. fringe) is to store nodes on the fringe. With this data structure,
    # you can use add() (e.g. fringe.add(node)) or remove() (e.g. fringe.remove(node)) functions
    # to update nodes on the given set.
    
    expandedNodes = set() 
    fringe = set([start])
    
    # create a dictonary that keeps track of parents of the opened nodes on the path to the goal.
    # For example, parent = {(1, 2): (1, 1)} depicts that (1,1) is a parent of (1,2).
    # This data structure can help to retrace the path when we reach to the goal.
    parent = {} 
    
    while len(fringe) > 0:
        #Pick the node in the fringe with the lowest G score
        curNode = None # initialize the current node.
        curGscore = None # initialize the G score of current node
        
        for pos in fringe:
            if curNode is None or G[pos] < curGscore:
                curGscore = G[pos]
                curNode = pos
        fringe.remove(curNode)
        
        #Check if our robot has reached the goal
        if curNode == goal:
            #Retrace our path backward
            path = [curNode]
            while curNode in parent:
                curNode = parent[curNode]
                path.append(curNode)
            path.reverse()
            return path, expandedNodes, G[goal] #Done!

        ############################################################################################
        ##                                  START OF YOUR CODE                                    ##
        ############################################################################################
        ## TODO:                                                                                   #
        ## Complete the remaining part from here. Specifically, if the current node is not the goal#
        ## state, then mark the current node as expanded (or processed, or visited). Then, go and  #
        ## check each neighbor of the current node using get_neighbors() function. Subsequently,   #
        ## calculate the path cost of the neigbor (using move_cost() function). Put the neighbor   #
        ## into the fringe and update the following: the value of its path cost to the set G; its  #
        ## parent's corrdinate (parent is the current node). Note that, in case any neighbor was   #
        ## already in fringe, we will only update the value of path cost and its parent if the     #
        ## current path cost is less than the previous path cost of the same neighbor.             #
        ############################################################################################

        # This part was done first because it's easier than A*
        else:
            # curNode is already visited that why we can place it to expandedNodes
            expandedNodes.add(curNode)

            # after that we go through all curNode neighbors
            for x in get_neighbors(curNode):
                # once element was visited it is not necessary to
                # visit the same element again
                if x not in expandedNodes:
                    # add new element to the fringe
                    if x not in fringe:
                        fringe.add(x)
                        parent[x] = curNode
                        G[x] = move_cost(x, barrier) + curGscore
                    # we CAN update element's data if it's G is
                    # big in comparison with new one G
                    elif move_cost(x, barrier) + curGscore < G[x]:
                        parent[x] = curNode
                        G[x] = move_cost(x, barrier) + curGscore
            # For new iteration I clean data of curNode
            curNode = None

        ###########################################################################################
        ##                                   END OF YOUR CODE                                    ##
        ###########################################################################################
            
    raise RuntimeError("UCS failed to find a solution")
    


def aStarSearch(start, goal, barrier):
    # Implementation of A Star Search (3 points)
    """
    Args: 
        start: coordinate of start node
        goal: coordinate of goal node
        barrier: list of coordinates of the points belonging to the barrier
    Return:
        path: list of coordinates of the points belonging to optimal path
        expandedNodes: set of coordinates of expanded nodes
        cost: total estimated path cost at the goal state
    """
    ######################################################################################
    
    # Firstly, create two dictonaries that store the path cost (g) and total estimated cost (f)
    # to reach each node from the start position. 
    # For example, F ={(3, 3): 7} depicts that total cost of (3,3) is 7.
    
    G = {} 
    F = {} 
    
    #Initialize values at starting position
    G[start] = 0
    F[start] = heuristic(start, goal)
    
    # Create two sets. One set (i.e. expandedNodes) is used to store the expanded nodes, 
    # and another set (i.e. fringe) is to store nodes on the fringe. With this data structure,
    # you can use add() (e.g. fringe.add(node)) or remove() (e.g. fringe.remove(node)) functions
    # to update nodes on the given set.
    
    expandedNodes = set() 
    fringe = set([start]) 
    
    # create a dictonary that keeps track of parents of the opened nodes on the path to the goal.
    # For example, parent ={(1, 2): (1, 1)} depicts that (1,1) is a parent of (1,2).
    # This data structure can help to retrace the path when we reach to the goal.
    parent = {} 
    
    while len(fringe) > 0:
        #Pick the node in the fringe with the lowest F score
        curNode = None # initialize the current node.
        curFscore = None # initialize the F score of current node
        
        for pos in fringe:
            if curNode is None or F[pos] < curFscore:
                curFscore = F[pos]
                curNode = pos
        fringe.remove(curNode)
        
        #Check if our robot has reached the goal
        if curNode == goal:
            #Retrace our path backward
            path = [curNode]
            while curNode in parent:
                curNode = parent[curNode]
                path.append(curNode)
            path.reverse()
            return path, expandedNodes, F[goal] #Done!
        
        ############################################################################################
        ##                                  START OF YOUR CODE                                    ##
        ############################################################################################
        ## TODO:                                                                                   #
        ## Complete the remaining part from here. Specifically, if the current node is not the goal#
        ## state, then mark the current node as expanded (or processed, or visited). Then, go and  #
        ## check each neighbor of the current node using get_neighbors() function. Subsequently,   #
        ## calculate the path cost and total cost of the neigbor (using move_cost() function). Put # 
        ## the neighbor into the fringe and update the following: value of its path cost to set G; #
        ## value of its total cost to set F (using heuristic() function); its parent's corrdinate  #
        ## (the current node is the parent). Note that, in case the neighbor was already in fringe,# 
        ## we will only update the values and its parent if the current path cost is less than the #
        ## previous path cost of this neighbor node.                                               #
        ############################################################################################

        # This part was done after UCS, most comments are the same
        else:
            # curNode is already visited that why we can place it to expandedNodes
            expandedNodes.add(curNode)

            # after that we go through all curNode neighbors
            for x in get_neighbors(curNode):
                # once element was visited it is not necessary to
                # visit the same element again
                if x not in expandedNodes:
                    # add new element to the fringe
                    # F - total cost = g + h, can calculate only after G was derived
                    # G - path cost
                    if x not in fringe:
                        fringe.add(x)
                        parent[x] = curNode
                        G[x] = move_cost(x, barrier) + G[curNode]
                        F[x] = G[x] + heuristic(x, goal)
                    # we CAN update element's data if it's G is
                    # big in comparison with new one G
                    elif move_cost(x, barrier) + G[curNode] + heuristic(x, goal) < F[x]:
                        parent[x] = curNode
                        G[x] = move_cost(x, barrier) + G[curNode]
                        F[x] = G[x] + heuristic(x, goal)
            curNode = None
        
            
        ###########################################################################################
        ##                                   END OF YOUR CODE                                    ##
        ###########################################################################################
    raise RuntimeError("A* failed to find a solution")