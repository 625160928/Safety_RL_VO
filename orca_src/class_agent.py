from numpy import array, sqrt, copysign, dot


class Agent(object):
    """A disk-shaped agent."""
    def __init__(self, position, velocity, radius, max_speed, pref_velocity,theta=None):
        self.position = array(position)
        self.velocity = array(velocity)
        self.radius = radius
        self.max_speed = max_speed
        self.pref_velocity = array(pref_velocity)
        self.theta=theta
        super(Agent, self).__init__()