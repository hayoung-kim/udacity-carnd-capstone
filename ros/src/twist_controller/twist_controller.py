
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        pass

    def control(self, current_linear_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = 0.
        if current_linear_vel >= 1.0:
            # calculate target steer
            steer = self.wheel_base * target_angular_vel / current_linear_vel

        steer = self.steer_ratio * steer

        print " [-] target steering wheel angle = %.2f (deg)" % (steer * 3.141592 / 180)

        return 0.2, 0., steer
