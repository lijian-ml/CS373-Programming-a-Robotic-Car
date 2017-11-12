from matrix import *

class kalman:

    def __init__(self, x=None, z=None, F=None, P=None):
        """
        This function is called when you create a filter function. 
        x -> state
        z -> measurement
        F -> state from last state to next state
        P -> x last covariances
        """
        self.n = len(x.value)
        # measurement function: reflect the fact that we observe x and y 
        self.H = matrix([[1.0 if i==j else 0.0 for j in range(self.n)] for i in range(2)])
        # measurement noise uncertainty: use 2x2 matrix with 0.1 as main diagonal
        self.R = matrix([[2.0, 0.0],
                    [0.0, 2.0]])
        # n d identity matrix
        self.I = matrix([[1.0 if i==j else 0.0 for j in range(self.n)] for i in range(self.n)]) 
        # external control motion  
        self.u = matrix([[0.] for i in range(self.n)])

        # F = F if F != None else I
        # P = P if P != None else I
        # z = z if z != None else matrix([[0,0]])

    def kalman_filter(self, x, z, F, P):
        # prediction
        x = (F * x) + self.u  # x -> last state
        P = F * P * F.transpose()

        # measurement update
        Z = matrix([z])
        y = Z.transpose() - (self.H * x)
        S = self.H * P * self.H.transpose() + self.R
        K = P * self.H.transpose() * S.inverse()
        x = x + (K * y)
        P = (self.I - (K * H)) * P
        return x, P


    def extended_kalman_filter(self, x, z, F, P):
        """
        Applies extended kalman filter on system

        z -> measurement
        x -> predict state
        u -> control vector
        P -> covariances
        F -> Function that returns F matrix for given 'x'
        H -> Measurement matrix
        R -> Measurement covariance
        """

        # prediction
        x = x + self.u
        P = F * P * F.transpose()

        # measurement update
        Z = matrix([z])
        y = Z.transpose() - (self.H * x)
        S = self.H * P * self.H.transpose() + self.R
        K = P * self.H.transpose() * S.inverse()
        x = x + (K * y)
        P = (self.I - (K * self.H)) * P

        return x, P
