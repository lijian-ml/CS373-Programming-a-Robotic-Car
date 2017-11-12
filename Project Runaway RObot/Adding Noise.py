# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
import time

def setup_kalman_filter():
    """
    Setup 7D Kalman Filter for this problem
    [x, y, v, a, theta, w, w']
    """
    # measurement function: reflect the fact that we observe x and y but not the two velocities and two accelerations
    H = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.]])
    # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    R = matrix([[0.1, 0.0],
                [0.0, 0.1]])
    # 6d identity matrix
    I = matrix([[1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0]]) 

    return H, R, I
# This is the function you have to write. The argument 'measurement' is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    #stepsize
    dt = 1.0
    H, R, I = setup_kalman_filter()
    u = matrix([[0.], [0.], [0.], [0.], [0.]]) # external motion
    
    if OTHER == None:
        xy_estimate = measurement
        # initial uncertainty: 0 for positions x and y, 1000 for the two velocities, 1000 for the two acceleration
        P = matrix([[1000., 0.,    0.,    0.,    0.],
                    [0., 1000.,    0.,    0.,    0.],
                    [0.,    0., 1000.,    0.,    0.],
                    [0.,    0.,    0., 1000.,    0.],
                    [0.,    0.,    0.,    0., 1000.]])
         
        x = matrix([[0.], [0.], [0.], [0.], [0.]]) 
        OTHER = [[measurement], P, x]
        return xy_estimate, OTHER
    elif len(OTHER[0])==1:
        OTHER[0].append(measurement)
        xy_estimate = measurement
        return xy_estimate, OTHER
    elif len(OTHER[0])==2:
        OTHER[0].append(measurement)     
        parameters = [[0 for i in range(3)] for i in range(5)]
        parameters[0] = [OTHER[0][i][0] for i in range(len(OTHER[0]))]
        parameters[1] = [OTHER[0][i][1] for i in range(len(OTHER[0]))]
        parameters[2] = [0] + [sqrt((parameters[0][i+1]-parameters[0][i])**2+(parameters[1][i+1]-parameters[1][i])**2)/dt for i in range(len(parameters[0])-1)]
        parameters[3] = [0] + [atan2(parameters[1][i+1]-parameters[1][i], parameters[0][i+1]-parameters[0][i]) for i in range(len(parameters[0])-1)]
        parameters[4] = [0,0] + [(parameters[3][i]-parameters[3][i-1])/dt for i in range(2,len(parameters[0]))]

        v = parameters[2][-1]
        theta = parameters[3][-1]
        w = parameters[4][-1]

        OTHER[2] = matrix([[measurement[0]],[measurement[1]],[v],[theta],[w]])
        x = matrix([[measurement[0]+v*dt*cos(theta+w*dt)],
                    [measurement[1]+v*dt*sin(theta+w*dt)],
                    [v],
                    [theta+w*dt],
                    [w]])
        
        xy_estimate = (x.value[0][0], x.value[1][0])
        return xy_estimate, OTHER
    
    OTHER[0].append(measurement)
    P = OTHER[1]
    # prediction
    x = OTHER[2]
    v = x.value[2][0]
    theta = x.value[3][0]
    w = x.value[4][0]

    # next state function: 7d
    F = matrix([[1,  0, dt*cos(theta+w*dt), -v*dt*sin(theta+w*dt),  -v*dt**2*sin(theta+w*dt)],
                [0,  1, dt*sin(theta+w*dt),  v*dt*cos(theta+w*dt),   v*dt**2*cos(theta+w*dt)],
                [0,  0,        1,                    0,                                    0],
                [0,  0,        0,                    1,                                   dt],
                [0,  0,        0,                    0,                                    1]])
    X1 = matrix([[x.value[0][0]+x.value[2][0]*dt*cos(x.value[3][0]+x.value[4][0]*dt)],
                 [x.value[1][0]+x.value[2][0]*dt*sin(x.value[3][0]+x.value[4][0]*dt)],
                 [x.value[2][0]],
                 [x.value[3][0]+x.value[4][0]*dt],
                 [x.value[4][0]]])
    x = X1
    P = F * P * F.transpose()

    
    # measurement update
    Z = matrix([[measurement[0], measurement[1]]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P
 
    X = matrix([[x.value[0][0]+x.value[2][0]*dt*cos(x.value[3][0]+x.value[4][0]*dt)],
                [x.value[1][0]+x.value[2][0]*dt*sin(x.value[3][0]+x.value[4][0]*dt)],
                [x.value[2][0]],
                [x.value[3][0]+x.value[4][0]*dt],
                [x.value[4][0]]])
    
    OTHER[1] = P
    OTHER[2] = x
    xy_estimate = (X.value[0][0], X.value[1][0])
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
        #print xy_estimate, measurement
    return xy_estimate, OTHER 

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 100:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        print (error)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 100:
            print "Sorry, it took you too many steps to localize the target."
    return localized

def demo_grading2(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()

    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)

        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
        time.sleep(5)
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)
