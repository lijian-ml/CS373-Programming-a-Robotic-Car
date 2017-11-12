# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    value = [[99 if grid[i][j]==1 else 100 for j in range(len(grid[i]))] for i in range(len(grid))]
    value[goal[0]][goal[1]] = 0
    n = 0
    while 1:
	    for i in range(len(value)):
	    	for j in range(len(value[i])):
	    		if value[i][j] == n:
	    			for k in range(len(delta)):
	    				if 0<=i+delta[k][0]<len(value) and 0<=j+delta[k][1]<len(value[0]):
		    				if value[i+delta[k][0]][j+delta[k][1]] == 100:
		    					value[i+delta[k][0]][j+delta[k][1]] = n+1
		if i == len(value)-1:
			n += 1
	    if [[0 if value[i][j]!=100 else 100 for j in range(len(value[i]))] for i in range(len(value))] == [[0 for j in range(len(grid[i]))] for i in range(len(grid))]:
	    	value_max = max([max([-1 if value[i][j]==99 else value[i][j] for j in range(len(value[i]))]) for i in range(len(value))])
	    	value_ = [[' ' for j in range(len(value[i]))] for i in range(len(value))]
	    	while 1:
		    	for i in range(len(value)):
		    		for j in range(len(value[i])):
		    			if value[i][j] == value_max:
		    				for k in range(len(delta)):
		    					if 0<=i+delta[k][0]<len(value) and 0<=j+delta[k][1]<len(value[0]):
		    						if value[i+delta[k][0]][j+delta[k][1]] == value_max-1:
		    							value_[i][j] = delta_name[k]
		    	value_max -= 1
	    		if value_max == 0:
	    			return value_
print (compute_value(grid,goal,cost))