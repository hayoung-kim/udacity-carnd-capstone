from pid import PID
import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.

mps2mph = 2.236936

lon_kp = 0.1
lon_ki = 0.1
lon_kd = 0.1

coeff_brake = 1.0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        # PID.step -> step(self, error, sample_time)
        self.PID_lon = PID(lon_kp, lon_ki, lon_kd, mn = 0.0, mx = 1.0)

        self.time_old = rospy.get_time()
        self.time = 0

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, current_linear_vel, target_linear_vel, target_angular_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        self.time = rospy.get_time()
        sample_time = self.time - self.time_old

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        steer = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_linear_vel)

        if dbw_enabled:
            # vel_err = target_linear_vel - mps2mph * current_linear_vel
            vel_err = target_linear_vel - current_linear_vel # m/s - m/s
            throttle = self.PID_lon.step(vel_err, sample_time)

            if vel_err < 0:
                brake = abs(coeff_brake * vel_err)

        self.time_old = self.time

        return throttle, brake, steer
