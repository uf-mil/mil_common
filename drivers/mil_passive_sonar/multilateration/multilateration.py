#!/usr/bin/python
from __future__ import division
import numpy as np
from scipy import optimize
from itertools import combinations

from sub8_msgs.srv import Sonar, SonarResponse

def quadratic(a, b, c):
    '''
    Solves a quadratic equation of the form ax^2 + bx + c = 0
 
    returns - list of solutions to the equation if there are any
    '''
    discriminant = b*b - 4*a*c
    if discriminant >= 0:
        first_times_a = (-b+math.copysign(math.sqrt(discriminant), -b))/2
        return [first_times_a/a, c/first_times_a]
    else:
        return []

def ls_line_intersection3d(start, end):
    '''
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf
    '''
    if len(start) != len(end):
        raise RuntimeError('Dimension mismatch')
    if len(start) < 2:
        raise RuntimeError('Insufficient line count')
    dir_vecs = end - start
    lengths = np.linalg.norm(dir_vecs).reshape((-1,1))
    dir_vecs = dir_vecs / lengths
    nx = dir_vecs[:, 0]
    ny = dir_vecs[:, 1]
    nz = dir_vecs[:, 2]
    XX = nx * nx - 1
    YY = ny * ny - 1
    ZZ = nz * nz - 1
    XY = nx * ny - 1
    XZ = nx * nz - 1
    YZ = ny * nz - 1
    AX = start[:, 0]
    AY = start[:, 1]
    AZ = start[:, 2]
    S = np.array([[np.sum(XX), np.sum(XY), np.sum(XZ)],
                  [np.sum(XY), np.sum(YY), np.sum(YZ)],
                  [np.sum(XZ), np.sum(YZ), np.sum(ZZ)]])
    CX = np.sum(AX * XX + AY * XY + AZ * XZ)
    CY = np.sum(AX * XY + AY * YY + AZ * YZ)
    CZ = np.sum(AX * XZ + AY * YZ + AZ * ZZ)
    C = np.stack((CX, CY, CZ))
    return np.linalg.lstsq(S, C)[0]

def ls_line_intersection2d(start, end):
    """
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2
    """
    if len(start) != len(end):
        raise RuntimeError('Dimension mismatch')
    if len(start) < 2:
        raise RuntimeError('Insufficient line count')
    dir_vecs = end - start
    lengths = np.linalg.norm(dir_vecs).reshape((-1,1))
    dir_vecs = dir_vecs / lengths
    Rl_90 = np.array([[0, -1], [1, 0]])  # rotates right 90deg
    perp_unit_vecs = Rl_90.dot(dir_vecs.T).T
    A_sum = np.zeros((2, 2))
    Ap_sum = np.zeros((2, 1))

    for x, y in zip(start, perp_unit_vecs):
        A = y.reshape(2, 1).dot(y.reshape(1, 2))
        Ap = A.dot(x.reshape(2, 1))
        A_sum += A
        Ap_sum += Ap

    return np.linalg.lstsq(A, Ap_sum)[0]

def max_delta_t(receiver0, receiver1, c):
    '''
    Returns the absolute value of the maximum possible difference in time of arrival (DTOA)
    of a signal originating from a single source to two different receivers

    receiver0 - position of receiver0 (list or numpy array of coordinates)
    receiver1 - position of receiver1 (list or numpy array of coordinates)
    c - speed of the signal in the medium

    returns: positive float (speed with the same distance and time units as the arguments)

    Note: Units of distance must match for both the position and speed arguments
    '''
    dist = np.linalg.norm(receiver1 - receiver0)
    return dist / c

def get_time_delta(t, non_ref, ref):
    '''
    Given two signals that are identiacal except for a time delay and some noise,
    this will return the time delay of the non_ref signal with respect to ref

    t - 1D numpy array of time values for the elements of non_ref and ref
    non_ref - 1D numpy array
    ref - 1D numpy array

    returns: scalar (of the same type and units as an element of t)
    delta_t - time delay of non_ref w.r.t. ref
    cross_corr - cross-correlation of non_ref with ref
    t_corr - time delay values corresponding to the cross-correlation
    '''
    if len(non_ref) != len(ref):
        raise RuntimeError('Signals must be of the same length')
    if len(non_ref) != len(t):
        raise RuntimeError('Vector of time values must have the same size as the signals')
    cross_corr = np.correlate(non_ref, ref, mode='full')
    t_corr = np.linspace(-t[-1], t[-1], len(cross_corr))
    max_idx = cross_corr.argmax()
    delta_t = t_corr[max_idx]
    return delta_t, cross_corr, t_corr

class Multilaterator(object):
    '''
    This class uses difference in time of arrival (DTOA) measurements to calculate the relative
    heading or postion of a signal source with respect to a receiver array.

    There as of now, there are 3 different solvers that you can use. You can select using the 'method'
    argument.
    bancroft - analytical solution, fast but not very numerically stable
    LS - optimization routine, fast and robust to noise
    LS1 - optimization routine, slow, but may be more accurate

    If the distance between receivers is orders of magnitude below the distance between the source
    and the array, the resulting position estimate should be in the right heading direction but with
    unreliable range. Otherwise, it may be possible to fully estimate the relative position of the
    pinger.

    Note: All distances are in millimeters and all times in microseconds
    '''
    def __init__(self, receiver_locations, c, method):  # speed in millimeters/microsecond
        '''
        receiver_locations - N x 3 numpy array with the locations of receivers in units of millimeters.
            The location of the reference hydrophone is considered the origin of the receiver array's
            reference frame and should not be included
        c - speed at whic the pulse propagates through the medium (millimeters / microsecond)
        method - kind of solver used to estimate the position or heading to the pulse
        '''
        self.receiver_locations = receiver_locations
        self.n = len(receiver_locations)
        self.pairs = list(combinations(range(self.n), 2))
        self.c = c
        self.method = method
        self.solvers = {'bancroft' : self.estimate_pos_bancroft,
                        'LS'       : lambda dtoa: self.estimate_pos_LS(dtoa, self.cost_LS),
                        'LS1'      : lambda dtoa: self.estimate_pos_LS1(dtoa, self.cost_LS)}
        print "\x1b[32mSpeed of Sound (c):", self.c, "millimeter/microsecond\x1b[0m"

    def get_pulse_location(self, dtoa, method=None):
        '''
        Returns a 3-element list with the  coordinates  of the estimated position of a point source
        transmitter in the frame of the receiver array.

        timestamps - list of n-1 time dtoas, all with respect to a reference receiver
        '''
        if method == None:
            method = self.method
        if not len(self.receiver_locations) == len(dtoa):
            raise RuntimeError('Number of non-reference receivers and dtoa measurents don\'t match')
        return self.solvers[method](dtoa)

    def estimate_pos_bancroft(self, dtoa):
        '''
        Uses the Bancroft Algorithm to solve for the position of a source base on dtoa measurements
        '''
        N = len(dtoa)
        if N < 4:
            raise RuntimeError('At least 4 dtoa measurements are needed')
        
        L = lambda a, b: a[0]*b[0] + a[1]*b[1] + a[2]*b[2] - a[3]*b[3]
        
        def get_B(delta):
            B = np.zeros((N, 4))
            for i in xrange(N):
                B[i] = np.concatenate([self.receiver_locations[i]/(self.c), [-dtoa[i]]]) + delta
            return B
        
        delta = min([.1*np.random.randn(4) for i in xrange(10)], key=lambda delta: np.linalg.cond(get_B(delta)))
        # delta = np.zeros(4) # gives very good heading for noisy timestamps, although range is completely unreliable

        B = get_B(delta)
        a = np.array([0.5 * L(B[i], B[i]) for i in xrange(N)])
        e = np.ones(N)
        
        Bpe = np.linalg.lstsq(B, e)[0]
        Bpa = np.linalg.lstsq(B, a)[0]
        
        Lambdas = quadratic(
            L(Bpe, Bpe),
            2*(L(Bpa, Bpe) - 1),
            L(Bpa, Bpa))
        if not Lambdas: 
            return [0, 0, 0]
        
        res = []
        for Lambda in Lambdas:
            u = Bpa + Lambda * Bpe
            position = u[:3] - delta[:3]
            time = u[3] + delta[3]
            if any(dtoa[i] < time for i in xrange(N)): continue
            res.append(position*self.c)
        if len(res) == 1:
            source = res[0]
        elif len(res) == 2:
            source = [x for x in res if x[2] < 0]   # Assume that the source is below us
            if not source: 
                source = res[0]
            else:
                source = source[0]
        else:
            source = [0, 0, 0]
        return source

    def estimate_pos_LS(self, dtoa, cost_func):
        '''
        Uses the a minimization routine to solve for the position of a source base on dtoa measurements
        '''
        self.dtoa = dtoa
        init_guess = np.random.normal(0,100,3)
        opt = {'disp': 0}
        opt_method = 'Powell'
        result = optimize.minimize(cost_func, init_guess, method=opt_method, options=opt, tol=1e-15)
        if(result.success):
            source = [result.x[0], result.x[1], result.x[2]]
        else:
            source = [0, 0, 0]
        return source

    def cost_LS(self, potential_pulse):
        """
        Slightly less accurate than the one above in terms of heading but much faster.
        """
        cost = 0
        t = self.dtoa
        x = potential_pulse[0]
        y = potential_pulse[1]
        z = potential_pulse[2]
        d0 = np.sqrt((x)**2 + (y)**2 + (z)**2)
        for i in range(self.n - 1):
            xi = self.receiver_locations[i, 0]
            yi = self.receiver_locations[i, 1]
            zi = self.receiver_locations[i, 2]
            di = np.linalg.norm([xi - x, yi - y, zi -z])
            receiver_i_cost = (di - d0 - self.c * t[i])**2
            cost = cost + receiver_i_cost
        return cost

    def cost_LS1(self, potential_pulse):
        '''
        Generates cost proportional to the difference in observed and theoretical dtoa measurements
        between every unique pair of receivers.
        '''
        cost = 0
        t = self.dtoa
        c = self.c
        x = np.array(potential_pulse)
        rcv = np.concatenate((np.array([[0, 0, 0]]), self.receiver_locations))

        for pair in self.pairs:
            cost += (np.linalg.norm(x-rcv[pair[0]]) - np.linalg.norm(x-rcv[pair[1]]) 
                     - c*(t[pair[0]] - t[pair[1]])) ** 2
        return cost


class ReceiverArraySim(object):
    """
    Simulates an array of receivers that listens to point sources and returns the DTOA.
    (difference in time of arrival)
    """
    def __init__(self, receiver_locations, c):
        '''
        receiver_locations - (N-1 x 3) numpy array. Positions of the non_reference receivers
            (in millimeters) w.r.t the reference receiver.  Do not include reference receiver, assumed
            to be at the origin.
        c - speed of propagation of a signal in the medium (millimeters / microsecond)
        '''
        self.c = c
        self.receiver_locations = receiver_locations
        self.n = len(receiver_locations) + 1

    def listen(self, pulse):
        '''
        Returns n-1 dtoa measurements for each of the non_reference receivers with respect to the
        reference.
        '''
        dtoa = []
        for idx in range(self.n - 1):
            src_range = np.sqrt(sum(np.square(pulse.position() - self.receiver_locations[idx])))
            dtoa += [src_range / self.c]
        return dtoa

class Pulse(object):
    """
    Represents an omnidirectional signal or impulse emmited from a point source

    Units should be in millimeters
    """
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def position(self):
        return np.array([self.x, self.y, self.z])

    def __repr__(self):
        return "Pulse:\t" + "x: " + str(self.x) + " y: " + str(self.y) + " z: " \
            + str(self.z) + " (mm)"

