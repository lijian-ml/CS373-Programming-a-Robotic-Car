# --------------
# USER INSTRUCTIONS
#
# Write a function called stochastic_value that 
# returns two grids. The first grid, value, should 
# contain the computed value of each cell as shown 
# in the video. The second grid, policy, should 
# contain the optimum policy for each cell.
#
# --------------
# GRADING NOTES
#
# We will be calling your stochastic_value function
# with several different grids and different values
# of success_prob, collision_cost, and cost_step.
# In order to be marked correct, your function must
# RETURN (it does not have to print) two grids,
# value and policy.
#
# When grading your value grid, we will compare the
# value of each cell with the true value according
# to this model. If your answer for each cell
# is sufficiently close to the correct answer
# (within 0.001), you will be marked as correct.

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>'] # Use these when creating your policy grid.

# ---------------------------------------------
#  Modify the function stochastic_value below
# ---------------------------------------------

def stochastic_value(grid,goal,cost_step,collision_cost,success_prob):
    failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
    value = [[collision_cost if grid[row][col]==1 else 0 for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]

    ii = 1
    value_sum2 = 0
    while 1:
        value_sum1 = 0
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                aaa = []
                for e in delta:
                    if 0<=i+e[0]<len(grid) and 0<=j+e[1]<len(grid[0]) and grid[i][j] != 1 :
                        aaa.append([[i,j],[i+e[0], j+e[1]]])
                aaa_value =  value[i][j] if value[i][j]>0 else 110
                for k in range(len(aaa)):
                    nnn = delta.index([aaa[k][1][0]-i,aaa[k][1][1]-j])
                    bbb = [[i+delta[(nnn-1)%4][0], j+delta[(nnn-1)%4][1]], [i+delta[nnn][0], j+delta[nnn][1]], [i+delta[(nnn+1)%4][0], j+delta[(nnn+1)%4][1]]]
                    bbb = [value[q[0]][q[1]] if 0<=q[0]<len(grid) and 0<=q[1]<len(grid[0]) else collision_cost for q in bbb]
                    bbb = failure_prob*(bbb[0]+bbb[2]) + success_prob*(bbb[1]) + ((abs(aaa[k][1][0]-goal[0])+abs(aaa[k][1][1]-goal[1]))-(abs(i-goal[0])+abs(j-goal[1])))
                    if bbb<aaa_value:
                        aaa_value = bbb
                        policy[i][j] = delta_name[nnn]
        policy[goal[0]][goal[1]] = '*'

        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if policy[i][j] in delta_name:
                    nnn = delta_name.index(policy[i][j])
                    bbb = [[i+delta[(nnn-1)%4][0], j+delta[(nnn-1)%4][1]], [i+delta[nnn][0], j+delta[nnn][1]], [i+delta[(nnn+1)%4][0], j+delta[(nnn+1)%4][1]]]
                    bbb = [value[q[0]][q[1]] if 0<=q[0]<=len(grid) and 0<=q[1]<len(grid[0]) else collision_cost for q in bbb]
                    bbb = failure_prob*(bbb[0]+bbb[2]) + success_prob*(bbb[1]) + cost_step
                    value[i][j] = bbb
                    value_sum1 += bbb
        if abs(value_sum1 - value_sum2)<0.0001:
          print  ii 
          break
        value_sum2 = value_sum1 
        ii += 1
        
    return value, policy

# ---------------------------------------------
#  Use the code below to test your solution
# ---------------------------------------------

grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]
goal = [0, len(grid[0])-1] # Goal is in top right corner
cost_step = 1
collision_cost = 100
success_prob = 0.5

value,policy = stochastic_value(grid,goal,cost_step,collision_cost,success_prob)
for row in value:
    print row
for row in policy:
    print row

# Expected outputs:
#
# [57.9029, 40.2784, 26.0665,  0.0000]
# [47.0547, 36.5722, 29.9937, 27.2698]
# [53.1715, 42.0228, 37.7755, 45.0916]
# [77.5858, 100.00, 100.00, 73.5458]
#
# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
