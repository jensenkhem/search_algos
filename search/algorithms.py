import heapq

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains a static variable:
    map_width containing the width of the map. Although this variable is a property of the map and not of 
    the state, the property is used to compute the hash value of the state, which is used in the CLOSED list. 

    Each state has the values of x, y, g, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for A*, Bi-A*, and MM. For A* and Bi-A* the cost should be the f-value of the node, while
    for MM the cost should be the p-value of the node. 
    """
    map_width = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, cost):
        """
        Sets the g-value of the state
        """
        self._g = cost

    def get_cost(self):
        """
        Returns the g-value of the state
        """
        return self._cost
        
    def set_cost(self, cost):
        """
        Sets the g-value of the state
        """
        self._cost = cost

# h-function
def octile_distance(start, goal):
    deltax = abs(goal.get_x()-start.get_x())
    deltay = abs(goal.get_y()-start.get_y()) 
    return 1.5*min(deltax, deltay) + abs(deltax - deltay)

def aStar(start, goal, map): 
    """
    A*  Algorithm implementation
    Arguments:
        start: State - The initial state on the map
        goal: State - The goal state on the map
        map: Map - The state space
    Returns:
        solution_cost: int - The optimal solution cost from start to goal.
                             Returns -1 if no solution.
        expanded_nodes: int - The amount of nodes expanded during the algorithm
    """
    # Open list - heap
    open_list = []
    # Closed list - dict
    closed_list = {}
    # Solution Cost
    solution_cost = -1
    # Expanded nodes
    expanded_nodes = 0

    # First, take the start state and push into open list
    start.set_cost(start.get_g() + octile_distance(start, goal))
    heapq.heappush(open_list, start)
    closed_list[start.state_hash()] = start

    # Iterate until the open list is empty
    while(open_list):
        # Pop from open list
        state = heapq.heappop(open_list)
        # Check for goal
        if state == goal:
            solution_cost = state.get_g()
            return solution_cost, expanded_nodes

        # Expand
        successors = map.successors(state)
        expanded_nodes += 1

        # Iterate over expanded states
        for successor in successors:

              # Calculate the new cost and the state hash
            successor.set_cost(successor.get_g() + octile_distance(successor, goal))
            successor_hash = successor.state_hash()

            if successor_hash not in closed_list:
                # Push successor into open and closed list
                heapq.heappush(open_list, successor)
                closed_list[successor_hash] = successor

            if (successor_hash in closed_list) and successor.get_g() < closed_list[successor_hash].get_g():
                # Update the closed/open lists with the new min-value
                # and 're-heapify'
                successor_index = open_list.index(successor)
                open_list[successor_index].set_g(successor.get_g())
                open_list[successor_index].set_cost(successor.get_g() + octile_distance(successor, goal))
                closed_list[successor_hash] = successor
                heapq.heapify(open_list)
    
    return solution_cost, expanded_nodes
                   
def biaStar(start, goal, map): 
    """
    BI-A* Algorithm implementation
    Arguments:
        start: State - The initial state on the map
        goal: State - The goal state on the map
        map: Map - The state space
    Returns:
        solution_cost: int - The optimal solution cost from start to goal.
                             Returns -1 if no solution.
        expanded_nodes: int - The amount of nodes expanded during the algorithm
    """

    # Open lists - heaps
    open_list_f = []
    open_list_b = []
    # Closed lists - dicts
    closed_list_f = {}
    closed_list_b = {}
    # Solution Cost
    solution_cost = float('inf')
    # Expanded nodes
    expanded_nodes = 0

    # First, take the start and goal states and push into respective open/closed lists
    start.set_cost(start.get_g() + octile_distance(start, goal))
    goal.set_cost(goal.get_g() + octile_distance(goal, start))

    heapq.heappush(open_list_f, start)
    heapq.heappush(open_list_b, goal)
    closed_list_f[start.state_hash()] = start
    closed_list_b[goal.state_hash()] = goal

    # Iterate until either open list is empty
    while(open_list_f and open_list_b):
        cheapest_open_f_cost = open_list_f[0].get_cost() 
        cheapest_open_b_cost = open_list_b[0].get_cost()
        # Check for goal
        if solution_cost <= min(cheapest_open_f_cost, cheapest_open_b_cost):
            return solution_cost, expanded_nodes
        if cheapest_open_f_cost < cheapest_open_b_cost:
            # Pop from forward open list
            state = heapq.heappop(open_list_f)

            # Expand forward
            successors = map.successors(state)
            expanded_nodes += 1
        
            # Iterate over expanded states
            for successor in successors:
                # Get successor vars
                successor_hash = successor.state_hash()
                successor.set_cost(successor.get_g() + octile_distance(successor, goal))
                
                # Check for improved solution
                if successor_hash in closed_list_b:
                    solution_cost = min(solution_cost, closed_list_b[successor_hash].get_g() + successor.get_g())

                if (successor_hash in closed_list_f) and successor.get_g() < closed_list_f[successor_hash].get_g():
                    # Update the closed/open lists with the new min-value
                    # and 're-heapify'
                    index = open_list_f.index(successor)
                    open_list_f[index].set_g(successor.get_g())
                    open_list_f[index].set_cost(successor.get_g() + octile_distance(successor, goal))
                    closed_list_f[successor_hash] = successor
                    heapq.heapify(open_list_f)

                if successor_hash not in closed_list_f:
                    # Push successor into open and closed list
                    heapq.heappush(open_list_f, successor)
                    closed_list_f[successor_hash] = successor  
        else:
            # Pop from backward open list
            state = heapq.heappop(open_list_b)

            # Expand forward
            successors = map.successors(state)
            expanded_nodes += 1
        
            # Iterate over expanded states
            for successor in successors:
                # Get successor vars
                successor_hash = successor.state_hash()    
                successor.set_cost(successor.get_g() + octile_distance(successor, start))
                
                # Check for improved solution
                if successor_hash in closed_list_f:
                    solution_cost = min(solution_cost, closed_list_f[successor_hash].get_g() + successor.get_g())

                if (successor_hash in closed_list_b) and successor.get_g() < closed_list_b[successor_hash].get_g():
                    # Update the closed/open lists with the new min-value
                    # and 're-heapify'
                    index = open_list_b.index(successor)
                    open_list_b[index].set_g(successor.get_g())
                    open_list_b[index].set_cost(successor.get_g() + octile_distance(successor, start))
                    closed_list_b[successor_hash] = successor
                    heapq.heapify(open_list_b)

                if successor_hash not in closed_list_b:
                    # Push successor into open and closed list
                    heapq.heappush(open_list_b, successor)
                    closed_list_b[successor_hash] = successor  

    return -1, expanded_nodes

def mm(start, goal, map): 
    """
    MM Algorithm implementation
    Arguments:
        start: State - The initial state on the map
        goal: State - The goal state on the map
        map: Map - The state space
    Returns:
        solution_cost: int - The optimal solution cost from start to goal.
                             Returns -1 if no solution.
        expanded_nodes: int - The amount of nodes expanded during the algorithm
    """

    # Open lists - heaps
    open_list_f = []
    open_list_b = []
    # Closed lists - dicts
    closed_list_f = {}
    closed_list_b = {}
    # Solution Cost
    solution_cost = float('inf')
    # Expanded nodes
    expanded_nodes = 0

    # First, take the start and goal states and push into respective open/closed lists
    start.set_cost(max(start.get_g() + octile_distance(start, goal), 2 * start.get_g()))
    goal.set_cost(max(goal.get_g() + octile_distance(goal, start), 2 * goal.get_g()))

    heapq.heappush(open_list_f, start)
    heapq.heappush(open_list_b, goal)
    closed_list_f[start.state_hash()] = start
    closed_list_b[goal.state_hash()] = goal

    # Iterate until either open list is empty
    while(open_list_f and open_list_b):
        cheapest_open_f_cost = open_list_f[0].get_cost() 
        cheapest_open_b_cost = open_list_b[0].get_cost()
        # Check for goal
        if solution_cost <= min(cheapest_open_f_cost, cheapest_open_b_cost):
            return solution_cost, expanded_nodes
        if cheapest_open_f_cost < cheapest_open_b_cost:
            # Pop from forward open list
            state = heapq.heappop(open_list_f)

            # Expand forward
            successors = map.successors(state)
            expanded_nodes += 1
        
            # Iterate over expanded states
            for successor in successors:
                # Get successor vars
                successor_hash = successor.state_hash()               
                successor.set_cost(max(successor.get_g() + octile_distance(successor, goal), 2 * successor.get_g()))
                
                # Check for improved solution
                if successor_hash in closed_list_b:
                    solution_cost = min(solution_cost, closed_list_b[successor_hash].get_g() + successor.get_g())

                if (successor_hash in closed_list_f) and successor.get_g() < closed_list_f[successor_hash].get_g():
                    # Update the closed/open lists with the new min-value
                    # and 're-heapify'
                    index = open_list_f.index(successor)
                    open_list_f[index].set_g(successor.get_g())
                    open_list_f[index].set_cost(max(successor.get_g() + octile_distance(successor, goal), 2 * successor.get_g()))
                    closed_list_f[successor_hash] = successor
                    heapq.heapify(open_list_f)

                if successor_hash not in closed_list_f:
                    # Push successor into open and closed list
                    heapq.heappush(open_list_f, successor)
                    closed_list_f[successor_hash] = successor  
        else:
            # Pop from backward open list
            state = heapq.heappop(open_list_b)

            # Expand forward
            successors = map.successors(state)
            expanded_nodes += 1
        
            # Iterate over expanded states
            for successor in successors:
                # Get successor vars
                successor_hash = successor.state_hash()    
                # Update the count of expanded nodes
                successor.set_cost(max(successor.get_g() + octile_distance(successor, start), 2 * successor.get_g()))
                
                # Check for improved solution
                if successor_hash in closed_list_f:
                    solution_cost = min(solution_cost, closed_list_f[successor_hash].get_g() + successor.get_g())

                if (successor_hash in closed_list_b) and successor.get_g() < closed_list_b[successor_hash].get_g():
                    # Update the closed/open lists with the new min-value
                    # and 're-heapify'
                    index = open_list_b.index(successor)
                    open_list_b[index].set_g(successor.get_g())
                    open_list_b[index].set_cost(max(successor.get_g() + octile_distance(successor, start), 2 * successor.get_g()))
                    closed_list_b[successor_hash] = successor
                    heapq.heapify(open_list_b)
                if successor_hash not in closed_list_b:
                    # Push successor into open and closed list
                    heapq.heappush(open_list_b, successor)
                    closed_list_b[successor_hash] = successor  

    return -1, expanded_nodes
    



