# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------
def value(mean, init, cost):
  valus = [init[2]] + [forward.index([mean[i+1][j]-mean[i][j] for j in range(2)]) for i in range(len(mean)-1)]
  valus = [(valus[i+1]-valus[i])%4 for i in range(len(valus)-1)]
  valus = [cost[2] if e==1 else e for e in valus]
  valus = [cost[1] if e==0 else e for e in valus]
  valus = [cost[0] if e==3 else e for e in valus]
  return sum(valus)


# ----------------------------------------
# modify code below
# ----------------------------------------
#forward[(init[3]+i)%4]
def optimum_policy2D(grid,init,goal,cost):
  means = [] #quan bu lu jing
  mean = []  #dan ci lu jing
  mean_1 = [] #fen cha ji ben lu jing
  step = [[init[0], init[1]]]  #xia yi bu ji he
  last_step = [] #shang bu
  values = [] #ji suan xiao hao zhi
  min_value = 10000
  q = 0  #mei ci fen cha you ji gu
  while step != []:
    k = len(step)
    if value(mean, init, cost) < min_value:    #xiao hao bu chao guo xian you fang an
      for i in range(len(forward)):
        if 0<=step[-1][0]+forward[i][0]<len(grid) and 0<=step[-1][1]+forward[i][1]<len(grid[0]) and \
        grid[step[-1][0]+forward[i][0]][step[-1][1]+forward[i][1]]!=1 and \
        [step[-1][0]+forward[i][0],step[-1][1]+forward[i][1]] != last_step and \
        mean.count([step[-1][0]+forward[i][0],step[-1][1]+forward[i][1]])<2:     #bu chao jie #bu shi qiang #bu shi shang bu #bu neng chong fu 3 ci 
          if len(step) == 1:
            step = [[step[-1][0]+forward[i][0],step[-1][1]+forward[i][1]], step[0]]
          else:
            step = step[:-1] + [[step[-1][0]+forward[i][0],step[-1][1]+forward[i][1]]] +[step[-1]]

      #fen cha chu ji ben lu jing + fen cha shu
      q = len(step)-k
      if q > 1:
        last_step = step[-1]
        step.pop()
        mean.append(last_step)
        aaa = [[e for e in mean], q]
        mean_1.append(aaa)
      #cha lu ji suan shang bu
      if q == 1:
        last_step = step[-1]
        step.pop()
        mean.append(last_step)
      #cha lu bu tong, huan cha lu ji xu 
      if q == 0:
        for i in range(len(mean_1)):
          if mean_1[len(mean_1)-1-i][-1] > 1:
             mean_1[len(mean_1)-1-i][-1] -= 1
             mean_1 = mean_1[:len(mean_1)-i]
             aaa = [e for e in mean_1[-1][0]]
             mean = aaa
             last_step = mean[-1]
             step.pop()
             break
        if len(step) == k+q:
          step.pop()

      #shi fou dao da mu di di
      if goal in step:
        means.append(mean+[goal])
        if q>1:
          mean_1.pop()
        for i in range(len(mean_1)):
          if mean_1[len(mean_1)-1-i][-1] > 1:
             mean_1[len(mean_1)-1-i][-1] -= 1
             mean_1 = mean_1[:len(mean_1)-i]
             aaa = [e for e in mean_1[-1][0]]
             mean = aaa
             step.remove(goal)
             last_step = mean[-1]
             break
        if len(step) == k+q:
          step.pop()


      #tian jia mei ge fang fa de xiao hao zhi
      if len(means) != len(values):   
        values.append(value(means[-1], init, cost))
        min_value = min(values) 
    else:
      for i in range(len(mean_1)):
        if mean_1[len(mean_1)-1-i][-1] > 1:
           mean_1[len(mean_1)-1-i][-1] -= 1
           mean_1 = mean_1[:len(mean_1)-i]
           aaa = [e for e in mean_1[-1][0]]
           mean = aaa
           last_step = mean[-1]
           step.pop()
           break
      if len(step) == k:
        step.pop()   

  return means[values.index(min_value)],min_value,len(means)

print (optimum_policy2D(grid,init,goal,cost))
