# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
	all_list = []
	drop_list = []
	every_list = [init]
	n = 0
	nn = 1
	grid1 = [[-1 if q==1  else q for q in e] for e in grid]
	while 1:
		every_list1 = []
		for i in range(len(every_list)):
			for e in [[every_list[i][0]-1,every_list[i][1]],[every_list[i][0],every_list[i][1]-1],[every_list[i][0]+1,every_list[i][1]],[every_list[i][0],every_list[i][1]+1]]:
				if 0<=e[0]<5 and 0<=e[1]<6 and grid[e[0]][e[1]] == 0:
					if e not in drop_list and e not in every_list and e not in every_list1:
						every_list1.append(e)
						grid1[e[0]][e[1]] = nn
						nn += 1
		drop_list.extend(every_list)
		all_list.append(every_list)
		every_list = every_list1
		n += 1
		if goal in every_list:
			grid2 = [[-1 if (q==0 or q > grid1[goal[0]][goal[1]]) else q for q in e] for e in grid1]
			grid2[init[0]][init[1]] = 0
			move = goal
			grid[goal[0]][goal[1]] = '*'
			for i in range(len(all_list)):
				nnn = len(all_list) - i -1
				for e in all_list[nnn]:
					ee_list = [[e[0]-1,e[1]],[e[0],e[1]-1],[e[0]+1,e[1]],[e[0],e[1]+1]]
					for j in range(4):
						if ee_list[j] == move:
							grid[e[0]][e[1]] = delta_name[j]
							move = [e[0],e[1]]
			#return ([n, goal[0], goal[1]])
			#return grid2
			return grid
		if every_list == []:
			grid2 = [[-1 if q==0 else q for q in e] for e in grid1]
			grid2[init[0]][init[1]] = 0
			#return 'fail'
			return grid2
print (search(grid,init,goal,cost))
