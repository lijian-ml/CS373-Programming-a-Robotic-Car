# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

def setup_kalman_filter(R_value=0.1):
    """
    Setup 5D Kalman Filter for this problem
    [x, y, v, theta, w]
    """
    # measurement function: reflect the fact that we observe x and y but not the two velocities and two accelerations
    H = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.]])
    # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    R = matrix([[R_value, 0.0],
                [0.0, R_value]])
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

def estimate_next_pos(measurement, OTHER):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    #stepsize
    dt = 1.0
    r_value=2.5
    H, R, I = setup_kalman_filter(R_value=r_value)
    u = matrix([[0.], [0.], [0.], [0.], [0.]]) # external motion

    if OTHER ==None :
        OTHER = [[measurement], [0], [0]]
        xy_estimate = measurement
        return xy_estimate, OTHER 
    elif len(OTHER[0]) < 5 :
        OTHER[0].append(measurement)
        xy_estimate = measurement
        return xy_estimate, OTHER 
    elif len(OTHER[0]) == 5:
        OTHER[0].append(measurement)
        OTHER = [OTHER[0], 0]
        xy_estimate = measurement
        return xy_estimate, OTHER 


    OTHER[0].append(measurement)
    recrify_1 = [e for e in OTHER[0]]
    for i in range(5):
        recrify_1 = [recrify_1[0]] + [((recrify_1[i-1][0]+1.0*recrify_1[i][0]+recrify_1[i+1][0])/3.0,(recrify_1[i-1][1]+2.0*recrify_1[i][1]+recrify_1[i+1][1])/4.0) for i in range(1, len(recrify_1)-1)] + [recrify_1[-1]] 
    OTHER[1] = recrify_1[:-1]


    parameters = [[0 for i in range(len(OTHER[1]))] for i in range(5)]
    parameters[0] = [OTHER[1][i][0] for i in range(len(OTHER[1]))]
    parameters[1] = [OTHER[1][i][1] for i in range(len(OTHER[1]))]
    parameters[2] = [0] + [sqrt((parameters[0][i+1]-parameters[0][i])**2+(parameters[1][i+1]-parameters[1][i])**2)/dt for i in range(len(parameters[0])-1)]
    parameters[3] = [0] + [atan2(parameters[1][i+1]-parameters[1][i], parameters[0][i+1]-parameters[0][i]) for i in range(len(parameters[0])-1)]
    parameters[4] = [0,0] + [(parameters[3][i]-parameters[3][i-1])/dt for i in range(2,len(parameters[0]))]

    v = sum(parameters[2])/(len(parameters[2])-1)
    theta = parameters[3][2]
    w =  parameters[4][2]#sum(parameters[4])/(len(parameters[4])-2)
    x = matrix([[parameters[0][2]],
                [parameters[1][2]],
                [v], 
                [theta],
                [w]])
    # initial uncertainty: 0 for positions x and y, 1000 for the two velocities, 1000 for the two acceleration
    P = matrix([[1000., 0.,    0.,    0.,    0.],
                [0., 1000.,    0.,    0.,    0.],
                [0.,    0., 1000.,    0.,    0.],
                [0.,    0.,    0., 1000.,    0.],
                [0.,    0.,    0.,    0., 1000.]])
    #calculate P matrix
    for i in range(3, len(OTHER[1])):
        x_ = x.value[0][0]
        y_ = x.value[1][0]
        v = x.value[2][0]
        theta = x.value[3][0]
        w =  x.value[4][0]
        # prediction
        x = matrix([[x_+v*dt*cos(theta+w*dt)],
                    [y_+v*dt*sin(theta+w*dt)],
                    [v], 
                    [theta+w*dt],
                    [w]])
        # next state function: 5d
        F = matrix([[1,  0, dt*cos(theta+w*dt), -v*dt*sin(theta+w*dt),  -v*dt**2*sin(theta+w*dt)],
                    [0,  1, dt*sin(theta+w*dt),  v*dt*cos(theta+w*dt),   v*dt**2*cos(theta+w*dt)],
                    [0,  0,        1,                    0,                                    0],
                    [0,  0,        0,                    1,                                   dt],
                    [0,  0,        0,                    0,                                    1]])

        P = F * P * F.transpose()
        # measurement update
        Z = matrix([[parameters[0][i], parameters[1][i]]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
    #recrify the point at this point and predict next point
    for i in range(2):
        x_ = x.value[0][0]
        y_ = x.value[1][0]
        v = x.value[2][0]
        theta = x.value[3][0]
        w =  x.value[4][0]
        # prediction
        x = matrix([[x_+v*dt*cos(theta+w*dt)],
                    [y_+v*dt*sin(theta+w*dt)],
                    [v], 
                    [theta+w*dt],
                    [w]])
    
    xy_estimate = (x.value[0][0], x.value[1][0])

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
    xy_estimate, OTHER = estimate_next_pos(target_measurement, OTHER)
    measurements = xy_estimate
    hunter_positions = [hunter_position]
    hunter_headings = [hunter_heading]

    heading_to_target = get_heading(hunter_position, measurements)
    heading_difference = angle_trunc(heading_to_target - hunter_heading)
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.97 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 200:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True
        print separation
        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 200:
            print "It took too many steps to catch the target."
    return caught

def demo_grading2(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 200:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 200:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

demo_grading2(hunter, target, next_move)





