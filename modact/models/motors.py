from math import pi, sqrt

import numpy as np
from trimesh.primitives import Cylinder
from trimesh.transformations import translation_matrix

from .base import Model


class Stepper(Model):
    """Stepper class"""
    def __init__(self, name, motor_data):
        self.name = name
        self.motor_data = motor_data.copy()
        # Not a deepcopy as only top dict is modified
        self.fill_factor = 1
        self.r_scale = 1
        self.disp = 0
        self.adjust_coil(self.fill_factor, self.r_scale)

    def adjust_coil(self, fill_factor, r_scale):
        """Calculate the resistance and the number of turns given two scaling
        factors.

        Args:
            fill_factor: 0. - 1.
            r_scale: 0. - 1.
        """
        self.fill_factor = fill_factor
        self.r_scale = r_scale
        self.motor_data['R'] = self.motor_data['RNom'] * r_scale
        self.motor_data['Nw'] = self.motor_data['NwNom']*sqrt(r_scale*fill_factor)

    @property
    def height(self):
        return self.motor_data['mesh']['h']

    def mesh(self, previous_edge, groups=None):
        """Return the mesh of the motor described in mesh_data and centered
        around `center`
        """
        sign = np.sign(self.disp) if self.disp != 0 else 1
        previous_edge.dot(
            translation_matrix([0, 0, self.disp+sign*self.height/2]),
            out=previous_edge)
        mesh_data = self.motor_data['mesh']
        mesh = Cylinder(radius=mesh_data['r'], height=mesh_data['h'],
                        transform=previous_edge.copy())

        if groups is not None:
            groups[-1].append(mesh)

        return mesh,

    @property
    def volume(self):
        mesh_data = self.motor_data['mesh']
        return mesh_data['r']**2*mesh_data['h']*pi

    @property
    def i(self):
        return self.motor_data['Nm']  # speed ratio f_drive / f_mechanical

    def get_speed_torque(self, op):
        Vm = op.V - 0.1

        Reff = self.motor_data['R']
        Rtot = Reff + 1.
        Nw = self.motor_data['Nw']
        km0 = self.motor_data['km0']
        km = km0 * Nw
        L = self.motor_data['L0'] * Nw**2

        imax = 4/np.pi*Vm / Rtot
        if op.imax is not None and op.imax < imax:
            imax = op.imax

        Q_fstat = self.motor_data['Q_fstat']
        Q_fdyn = self.motor_data['Q_fdyn']

        wloop = op.speed
        omega = wloop / self.motor_data['Nm']

        RL = Rtot**2 + wloop**2*L**2
        i = Vm*4/np.pi/np.sqrt(RL) - (km)*omega*Rtot/RL
        Tp2 = np.minimum(i, imax)*km-Q_fstat-Q_fdyn*omega
        Tp2 = np.maximum(Tp2, 0)

        out = op.copy()
        out.speed = omega
        out.torque = Tp2
        out.imax = i
        return out

    @property
    def cost(self):
        cost = self.motor_data['ca']
        cost += self.fill_factor*self.motor_data['cb']
        return cost


motor_data = {
    'A': {'L0': 162.e-9, 'Nm': 5,
          'NwNom': 550., 'RNom': 32., 'km0': 68.5e-6,
          'Q_fstat': 0.5e-3, 'Q_fdyn': 1e-5,
          'cb':  0.0558285056, 'ca': 0.09696198,
          'mesh': {'r': 11, 'h': 16.8}},
    'B': {'L0': 150.e-9, 'Nm': 6,
          'NwNom': 500., 'RNom': 21., 'km0': 115.2e-6,
          'Q_fstat': 1.e-3, 'Q_fdyn': 3e-5,
          'cb': 0.1557764096, 'ca': 0.17555808,
          'mesh': {'r': 16.1, 'h': 16.4}},
    'C': {'L0': 234.e-9, 'Nm': 6,
          'NwNom': 300., 'RNom': 12., 'km0': 96.e-6,
          'Q_fstat': 0.5e-3, 'Q_fdyn': 2e-5,
          'cb': 0.061133184, 'ca': 0.2219547,
          'mesh': {'r': 12.5, 'h': 16.}},
    'D': {'L0': 316.e-9, 'Nm': 6,
          'NwNom': 240., 'RNom': 4.8, 'km0': 144.e-6,
          'Q_fstat': 0.5e-3, 'Q_fdyn': 9e-5,
          'cb': 0.20377728, 'ca': 0.40736514,
          'mesh': {'r': 17., 'h': 22.}},
    'E': {'L0': 23.e-9, 'Nm': 5,
          'NwNom': 1008., 'RNom': 52.0, 'km0': 32.5e-6,
          'Q_fstat': 0.8e-3, 'Q_fdyn': 0.5e-6,
          'cb': 0.0189545216, 'ca': 0.09626514,
          'mesh': {'r': 14., 'h': 8}}
}

motor_names = list(motor_data.keys())
motor_names.sort()


def get_stepper(name_or_number, fill_factor=1., r_scale=1.):
    if isinstance(name_or_number, (int, float)):
        name = motor_names[int(name_or_number)]
    else:
        name = name_or_number
    s = Stepper(name, motor_data[name])
    s.adjust_coil(fill_factor, r_scale)
    return s
