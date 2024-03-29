import math
import numpy as np
from scipy import special


class StrokeGenerator:
    """Generates strokes from trajectory input."""

    def __init__(self, eps=1e-15, sensitivity=1e-10):
        self.eps = eps  # a really small value, used to avoid divide by zero
        self.sensitivity = sensitivity  # a small value used for filtering

    @staticmethod
    def sigma(Ac):
        """
        Sigma: log response time (the log response times, the response times
        of the neuromuscular systems expressed on a logarithmic time scale)

        Args:
            Ac ([np.array(float)]): [shape parameter that defines "skewdness"
                                     of the lognormal curve]
        """
        return(np.sqrt(-np.log(1.0 - Ac)))

    @staticmethod
    def mu(sigma, T):
        """
        Mu: stroke delay (the logtime delays, the time delays of the
        neuromuscular systems expressed on a logarithmic time scale)

        Args:
            sigma ([np.array(float)]): [log response time of the stroke]
            T ([float]): [period of the trajectory]
        """
        return(3.*sigma - np.log((-1.0 + np.exp(6.*sigma)) / T))

    def D_theta(self, t_points, delta):
        """
        D: stroke amplitude, distance between target points
        theta: angle between target points

        Args:
            t_points ([np.array(float)]): [target points]
            delta ([np.array(float)]): [central angle of the circular arc
                                        shape of curve]
        """
        t_disp = np.diff(t_points, axis=1)
        D = np.sqrt(t_disp[0, :]**2 + t_disp[1, :]**2)
        theta = np.arctan2(t_disp[1, :], t_disp[0, :])
        D_adj = D
        # From Berio code:
        # shift D with lognormal (using delta parameter)
        if delta.any() > self.sensitivity:
            h = (delta/2) / np.sin(delta/2)
            h[np.abs(np.sin(delta)) < self.sensitivity] = 1.
            D_adj = D*h
        # in Berio17, this is "theta + ..."
        theta_0 = theta - (math.pi + delta)/2
        return(D, D_adj, theta, theta_0)

    @staticmethod
    def t0_t(dt, sigma, mu, T, delta_t, t_offset=0.):
        """
        t0: the time of ocurrence of the command that the lognormal impulse
            responds to.
                "D(t)i describes the direction and amplitude of each stroke
                (towards a virtual target) and describes the lognormal
                impulse response of each stroke to a centrally generated
                command occurring at time t0i."
        t: the time distribution for the trajectory

        Args:
            dt ([float]): the timestep of the trajectory
            sigma ([np.array(float)]): [log response time of the stroke]
            mu (np.array(float)]): [stroke delay]
            T ([float]): [period of the trajectory]
            delta_t ([np.array(float)]): [a relative time offset with respect
                                          to the previous stroke time
                                          occurrence and duration]
        """
        # Method from Berio code, similar to Berio 17eurographics:
        # add small time offset at the start to guarantee that
        # the first stroke starts with zero velocity
        n = delta_t.shape[0]
        t1 = np.zeros(n) + t_offset
        # eqtn: t0 = t0_(i-1) + delta_t*T - e^(mu - sigma)"""
        t1[1:] = delta_t[1:] * T
        t1 = np.cumsum(t1)
        # Add onsets in order to shift lognormal to start
        t0 = t1 - np.exp(mu[0] - sigma[0]*3)
        endtime = t0[-1] + np.exp(mu[-1] + sigma[-1]*3)
        t = np.arange(0.0, endtime, dt)
        return(t0, t)

    @staticmethod
    def time(dt, sigma, mu, t0):
        """
        t: the time distribution for the trajectory

        Args:
            dt ([float]): the timestep of the trajectory
            sigma ([np.array(float)]): [log response time of the stroke]
            mu (np.array(float)]): [stroke delay]
            T ([float]): [period of the trajectory]
            delta_t ([np.array(float)]): [a relative time offset with respect
                                          to the previous stroke time
                                          occurrence and duration]
        """
        endtime = t0[-1] + np.exp(mu[-1] + sigma[-1]*3)
        t = np.arange(0.0, endtime, dt)
        return(t)

    def weight(self, t, t0, sigma, mu):
        """
        weight: lognormal interpolation between target points

        Args:
            t ([np.array(float)]): [time domain of trajectory]
            t0 ([np.array(float)]): [time of the start of the lognormal]
            sigma ([np.array(float)]): [log response time of the stroke]
            mu ([np.array(float)]): [stroke delay]
        """
        t_stroke = np.maximum((t - t0), self.eps)
        weight = 0.5*(1 + special.erf((np.log(t_stroke) - mu) /
                                      (sigma*math.sqrt(2))))
        return(weight)

    def displacement(self, t_points, weight, D, theta, theta_0, delta):
        """
        displacement: displacements between interpolated points, generated
                      between target points

        Args:
            t_points ([np.array(float)]): [timestep between generated points]
            weight ([np.array(float)]): [lognormal interpolation between
                                         target points]
            D ([np.array(float)]): [stroke amplitude]
            theta ([np.array(float)]): [angle between target points]
            delta ([np.array(float)]): [central angle of the circular arc
                                        shape of curve]
        """

        P0 = t_points.T[0]
        P0 = P0.reshape(-1, 1)
        P0_offset = np.ones((2, len(weight)))*P0
        if abs(delta) > self.sensitivity:
            # in Berio17: trig_fn(th0 - delta*weight) - ...
            d = D*np.vstack([(np.cos(theta_0 + delta*weight) -
                              np.cos(theta_0)) / (2*np.sin(delta/2)),
                             (np.sin(theta_0 + delta*weight) -
                              np.sin(theta_0)) / (2*np.sin(delta/2))])
            displacement = d + P0_offset
        else:
            d = D * np.vstack([np.cos(theta)*weight,
                               np.sin(theta)*weight])
            displacement = d + P0_offset
        return(displacement)

    @staticmethod
    def velocity(dt, points):
        """
        velocity: lognormal profiles, speed in trajectory

        Args:
            dt ([float]): the timestep of the trajectory
            points ([np.array(float)]): the generated points of the
              trajectory
        """
        velocity_profiles = []
        for i in range(len(points)):
            velocity_prof = [math.dist(points[i][x], points[i][x-1])/dt
                             for x in range(1, len(points[i]))]
            time = [round(dt * x, 2) if i == 0
                    else (round(dt * x, 2))  # + T[-1])
                    for x in range(1, len(points[i]))]
            velocity_profiles.append([velocity_prof, time])
        return(velocity_profiles)
