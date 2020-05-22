# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 10] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)
                
goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def find_child(node):
	childs_list = []
	for i in range(len(action)):
		c = (node[2] + action[i]) % 4
		x = forward[c][0] + node[0]
		y = forward[c][1] + node[1]
		
		
		if(x <= len(grid[0]) and x >=0 and y <= len(grid) and y >= 0):
			if(grid[x][y] != 1):
				childs_list.append((x, y, c));
				
		
	return childs_list;	

def cost_neigh(neighbour,node):
	if(node[2] - neighbour[2] == 0):
		return cost[1];
	if(node[2] == 0 or node[2] == 3):
		if(neighbour[2] == 1 or neighbour[2] == 0):
			return cost[2];   # Left
		else:
			return cost[0];    # Right

	if(node[2] - neighbour[2] > 0 ):
		return cost[0];
	else:
		return cost[2];
		
		
def get_direction(next_node,node):
	if(node[2] - next_node[2] == 0):
		return 'F';
	if(node[2] == 0 or node[2] == 3):
		if(next_node[2] == 1 or next_node[2] == 0):
			return 'L';   # Left
		else:
			return 'R';    # Right

	if(node[2] - next_node[2] > 0 ):
		return 'R';
	else:
		return 'L';		
			
			
			
			
	
	
	
	
def compute_path(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)   
	
	
	   
    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up    
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))], 
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]
    #print(parent)
    # The path of the car
		
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]
    
		
		
    x = start[0]
    y = start[1]
    theta = start[2]
    h = heuristic[x][y]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))

    # your code: implement A*

    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    while open_set:
        node = open_set.pop()
        closed_set.add(node[0])
        if(node[0] == goal):
            break;
        # Finding child for each node.
        neighbours = find_child(node[0])
        
        for neighbour in neighbours:
            #print(neighbour)
            #print(node[0]) 
            g = node[1].g + cost_neigh(neighbour,node[0]) 
            x = neighbour[0];
            y = neighbour[1];
            theta = neighbour[2]
            h = heuristic[x][y];
            f = g + h;
             
            if(neighbour not in open_set or neighbour not in closed_set):
                open_set.put(neighbour,Value(f,g))
                a = parent[theta][x][y]
                if(a == ' ' or f < a[1].g + cost_neigh(neighbour,a[0]) + h):                
                    parent[theta][x][y] = node	                
								 
									  
            elif(neighbour in open_set and f < open_set.get(neighbour).f ):
                open_set.put(neighbour,Value(f,g))
                if(a == ' ' or f < a[1].g + cost_neigh(neighbour,a[0]) + h):
                    parent[theta][x][y] = node						
				
				
    g = goal
    d = []
    while(g!=start):
        #print(g)
        d.append(g)
        g = parent[g[2]][g[0]][g[1]][0]
        
         
        
    
    d.append(g)   		
    d.reverse()
    #print(d)
    
    direction_list = []
    for i in range(0,len(d) - 1):
        direction = get_direction(d[i+1],d[i])
        direction_list.append(direction)
      
    #print(direction_list);
    direction_list.append("*");
				    
    for i in range(0,len(d)):
        x=d[i][0];
        y=d[i][1];
        path[x][y] = direction_list[i];
			

    
		
		    				 
		
    return path, closed_set
if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")    
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""