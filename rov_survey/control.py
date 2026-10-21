import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def allocation_matrix(model_file):
    model = ET.parse(model_file).getroot().find(".//model[@name='bluerov2_ping']")
    columns = []
    for i in range(1, 7):
        pose = np.fromstring(model.find(f"link[@name='thruster{i}']/pose").text, sep=" ")
        axis = np.fromstring(model.find(f"joint[@name='thruster{i}_joint']/axis/xyz").text, sep=" ")
        force = Rotation.from_euler("xyz", pose[3:]).apply(axis)
        moment = np.cross(pose[:3], force)
        columns.append(np.r_[force, moment[0], moment[2]])

    return np.array(columns).T


class RobustController:
    def __init__(self, model_file):
        self.B = allocation_matrix(model_file)
        self.inverse = np.linalg.pinv(self.B)
        self.integral = np.zeros(3)
        self.v_filtered = np.zeros(3)
        self.last_thrust = np.zeros(6)

    def command(self, position, rotation, velocity_body, omega, target_velocity,
                yaw_target, depth, dt):
        world_velocity = rotation @ velocity_body
        command = np.r_[target_velocity, np.clip(1.1*(-depth-position[2]), -0.20, 0.20)]

        delta = np.clip(command-self.v_filtered, -0.18*dt, 0.18*dt)
        self.v_filtered += delta
        error = self.v_filtered-world_velocity
        old_integral = self.integral.copy()
        self.integral = np.clip(self.integral + dt*error, -2.0, 2.0)
        force_world = (np.array([24., 28., 32.])*error + 4.0*self.integral
                       + np.array([1.8, 2., 2.])*np.tanh(error/0.055))
        force_body = rotation.T @ force_world
        force_body += np.array([33.732, 54.16, 73.225])*np.abs(velocity_body)*velocity_body

        roll, pitch, yaw = Rotation.from_matrix(rotation).as_euler("xyz")
        yaw_rate = np.clip(1.5*wrap(yaw_target-yaw), -0.55, 0.55)
        yaw_torque = 1.8*(yaw_rate-omega[2]) + 3.992*abs(omega[2])*omega[2]
        roll_torque = -1.2*roll - 0.8*omega[0]
        wrench = np.r_[force_body, roll_torque, yaw_torque]
        raw = self.inverse @ wrench
        thrust = np.clip(raw, -18., 18.)
        if np.any(np.abs(raw) > 18.):
            self.integral = old_integral
        self.last_thrust = thrust
        return thrust
