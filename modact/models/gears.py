from collections import namedtuple
from math import acos, atan, cos, pi, sin, sqrt, tan
from operator import attrgetter

import numpy as np
from cached_property import cached_property
from scipy import optimize
from trimesh.primitives import Cylinder
from trimesh.transformations import rotation_matrix, translation_matrix

from ..materials import Material, get_material
from .base import Model


def inv(alpha):
    return tan(alpha) - alpha


def alpha_p_with_shifts(aw, a, Z1, x1, Z2, x2):
    return inv(aw) - 2 * (x1+x2)/(Z1+Z2) * tan(a) - inv(a)


"""Derived using ISO 21771:2007 & 1328-1:2013 & 53 & 6336 Method B & 1122-1"""


class SpurGear(object):
    def __init__(self, Z, m, x, b, material='steel', stretch=0):
        self.alpha = pi/9  # 20° contact angle
        self.alpha_p = self.alpha  # working contact angle
        self.m = m  # module
        self.ha = self.m  # addendum height
        self.hf = 1.25*self.m
        self.x = x  # profil shift
        self.Z = Z
        self.b = b*self.m  # gear width
        self.stretch = stretch
        if isinstance(material, Material):
            self.material = material
        else:
            self.material = get_material(material)
        self.update_prime()

    def update_prime(self):
        """Set working dimensions (prime) based on :math:`\alpha'`"""
        self.m_p = self.m * cos(self.alpha)/cos(self.alpha_p)
        self.tan_alpha_t_p = tan(self.alpha_p)
        self.alpha_t = atan(tan(self.alpha))
        self.alpha_t_p = atan(self.tan_alpha_t_p)

    @property
    def d(self):
        return self.m*self.Z

    @property
    def d_p(self):
        return self.m_p*self.Z

    @property
    def db(self):
        return self.d_p * cos(self.alpha_p)

    @property
    def da(self):
        return self.d + 2 * (self.ha + self.x*self.m)

    @property
    def df(self):
        return self.d - 2 * (self.hf - self.x*self.m)

    @property
    def h(self):
        return self.ha + self.hf

    @property
    def CT(self):
        return 0.5*self.db*tan(self.alpha_p)

    @property
    def rho_A(self):
        return 0.5*(sqrt(self.da**2 - self.db**2))

    @property
    def g(self):
        return self.rho_A - self.CT

    @property
    def volume(self):
        return pi*self.d**2/4*(self.b+self.stretch)*1e-9

    @property
    def cost(self):
        return self.volume*self.material.rho*self.material.cost

    @property
    def height(self):
        return self.b

    def load(self, torque):
        """Return load acting on a tooth.

        out : (Ft, Fr, Fa, Fn)"""
        Ft = 2*torque/(self.d_p*1e-3)
        return (Ft, Ft*self.tan_alpha_t_p, 0, Ft/cos(self.alpha_p))

    def mesh(self, at):
        return Cylinder(radius=self.d_p/2-0.005, height=self.b+self.stretch,
                        transform=at.copy())


TwoGears = namedtuple('TwoGears', ['p', 'g'])  # Pinion, gear


class GearPair(Model):
    stretch_margin: float = .001

    def __init__(self, pinion, gear, disp=0, angle=None):
        self.gears = TwoGears(p=pinion, g=gear)
        self.alpha = pinion.alpha
        self.a0 = 0.5*(self.gears.p.d + self.gears.g.d)
        if self.gears.g.Z > self.gears.p.Z:
            self.u = self.gears.g.Z / self.gears.p.Z
        else:
            self.u = self.gears.p.Z / self.gears.g.Z
        self.i = self.gears.g.Z / self.gears.p.Z

        self.disp = disp
        self.angle = angle

        pinion.stretch = max(0, abs(disp) - self.stretch_margin)

        self.set_working_conditions()

    def set_working_conditions(self):
        p = self.gears.p
        g = self.gears.g
        try:
            self.alpha_p = optimize.brentq(alpha_p_with_shifts, 0.1, pi/2,
                                           args=(self.alpha, p.Z, p.x, g.Z, g.x))
        except ValueError:
            raise
        p.alpha_p = self.alpha_p
        p.update_prime()
        g.alpha_p = self.alpha_p
        g.update_prime()
        self.ap = 0.5*(self.gears.p.d_p + self.gears.g.d_p)

    @property
    def T1T2(self):
        return self.ap * sin(self.alpha_p)

    @property
    def ET1(self):
        return self.gears.p.rho_A

    @property
    def AT2(self):
        return self.gears.g.rho_A

    @property
    def AT1(self):
        return self.T1T2 - self.AT2

    @property
    def ET2(self):
        return self.T1T2 - self.ET1

    @cached_property
    def interference(self):
        """Calculate tooth interference. No interference returns 0.
        Otherwise returns negative value"""
        delta_gf1 = self.gears.g.CT - self.gears.p.g
        delta_ga1 = self.gears.p.CT - self.gears.g.g
        # Temporarly to fix error with negative AT2
        interf = min(delta_gf1, 0) + min(delta_ga1, 0) + min(self.AT2, 0)
        if interf < 0 and interf >= -1e-6:
            interf = 0
        return interf

    @cached_property
    def contact_ratio(self):
        gf1 = self.gears.g.g
        ga1 = self.gears.p.g
        return (gf1+ga1)/(pi*self.gears.p.m_p*cos(self.alpha_p))

    @cached_property
    def specific_speed(self):
        gs1 = 1 - self.AT2/self.u/self.AT1
        gs2 = 1 - self.u*self.ET1/self.ET2
        return (gs1, gs2)

    def get_speed_torque(self, op):
        out = op.copy()
        out.speed = op.speed / self.i
        out.torque = op.torque*self.i
        return out

    @cached_property
    def volume(self):
        """Sum of the volume of the gears composing the pair"""
        return sum(map(attrgetter('volume'), self.gears))

    @cached_property
    def cost(self):
        """Sum of the cost of the gears composing the pair"""
        return sum(map(attrgetter('cost'), self.gears))

    @property
    def Z_eps(self):
        """Calculate stress correction factor due to the contact ratio"""
        return sqrt((4-self.contact_ratio)/3)

    def sigma_h_0(self, Ft):
        p = self.gears.p
        g = self.gears.g
        Z_H = sqrt(2/cos(p.alpha_t)**2/p.tan_alpha_t_p)
        Z_E = sqrt(1/((1-p.material.nu**2)/p.material.E + (1-g.material.nu**2)/g.material.E)*1/pi)

        Z_eps = self.Z_eps
        return Z_H*Z_E*Z_eps*sqrt(Ft/(p.d*1e-3)/(p.b*1e-3)*(self.u+1)/self.u)

    def sigma_h(self, speed, torque):
        Ft, _, _, _ = self.gears.p.load(torque)
        sigma_h_0 = self.sigma_h_0(Ft)
        p = self.gears.p
        g = self.gears.g
        M1 = p.tan_alpha_t_p/sqrt((sqrt(p.da**2/p.db**2-1) - 2*pi/p.Z)*(sqrt(g.da**2/g.db**2-1)-(self.contact_ratio-1)*2*pi/g.Z))
        ZB = M1 if M1 >= 1. else 1.
        try:
            M2 = p.tan_alpha_t_p/sqrt((sqrt(g.da**2/g.db**2-1) - 2*pi/g.Z)*(sqrt(p.da**2/p.db**2-1)-(self.contact_ratio-1)*2*pi/p.Z))
        except ValueError:
            M2 = 1.
        ZD = M2 if M2 >= 1. else 1.
        return sigma_h_0, (sigma_h_0*ZB*sqrt(1.25), sigma_h_0*ZD*sqrt(1.25))

    def security_h(self, condition):
        _, sig = self.sigma_h(condition.speed, condition.torque)
        Z_corr = 0.85
        sigma_h_lim_p = self.gears.p.material.sigma_h_lim*Z_corr
        sigma_h_lim_g = self.gears.g.material.sigma_h_lim*Z_corr
        return (sigma_h_lim_p/sig[0], sigma_h_lim_g/sig[1])

    def sigma_f_0(self, Ft, gear):
        """
        Using x and not xE because wheel are injected...
        """
        # Follwing ISO 6336-2 6.2 to calculate Y_F
        # with spr = 0
        rho_fp = 0.38*gear.m
        E = pi/4*gear.m - gear.hf*tan(gear.alpha) - (1-sin(gear.alpha))*rho_fp/cos(gear.alpha)
        G = rho_fp/gear.m - gear.hf/gear.m + gear.x
        zn = gear.Z
        dn = gear.m*zn
        eps_an = self.contact_ratio
        dbn = dn*cos(gear.alpha)
        dan = dn+gear.da-gear.d
        den = 2*sqrt((sqrt(dan**2/4-dbn**2/4)-pi*gear.d*cos(gear.alpha)/gear.Z*(eps_an-1))**2 + dbn**2/4)
        alpha_en = acos(dbn/den)
        ge = (0.5*pi+2*tan(gear.alpha)*gear.x)/zn + inv(gear.alpha) - inv(alpha_en)
        alpha_Fen = alpha_en - ge
        T = pi/3  # exterior gear
        H = 2./zn*(pi/2-E/gear.m)-T

        def f_theta(x):
            return x - 2*G/zn*tan(x) + H

        theta = optimize.fsolve(f_theta, pi/6)[0]
        sFnom = zn*sin(T-theta) + sqrt(3)*(G/cos(theta)-rho_fp/gear.m)
        hFeom = 0.5*((cos(ge)-sin(ge)*tan(alpha_Fen))*den/gear.m - zn*cos(T-theta) - (G/cos(theta) - rho_fp/gear.m))
        rhofom = rho_fp/gear.m + 2*G**2/(cos(theta)*(zn*cos(theta)**2-2*G))
        Y_F = 6*hFeom*cos(alpha_Fen)/(sFnom**2*cos(gear.alpha))
        L = sFnom/hFeom
        qs = sFnom/rhofom/2
        Y_S = (1.2 + 0.13*L)*qs**(1/(1.21+2.3/L))
        Y_DT = 1.  # For classes > 4
        return Ft/(gear.b*gear.m*1e-6)*Y_F*Y_S*Y_DT

    def sigma_f(self, speed, torque):
        p = self.gears.p
        g = self.gears.g
        Ft, _, _, _ = p.load(torque)
        sigma_f_01 = self.sigma_f_0(Ft, p)
        sigma_f_02 = self.sigma_f_0(Ft, g)
        sigma_f1 = sigma_f_01*1.25
        sigma_f2 = sigma_f_02*1.25
        return (sigma_f_01, sigma_f_02), (sigma_f1, sigma_f2)

    def security_f(self, condition):
        """Calculate security factor of tooth foot"""
        _, sig = self.sigma_f(condition.speed, condition.torque)
        Y_corr = 2*0.85  # empirical extraction
        sigma_f_lim_p = self.gears.p.material.sigma_f_lim*Y_corr
        sigma_f_lim_g = self.gears.g.material.sigma_f_lim*Y_corr

        stress_safety_1 = sigma_f_lim_p/sig[0]
        stress_safety_2 = sigma_f_lim_g/sig[1]

        return (stress_safety_1, stress_safety_2)

    @property
    def height(self):
        return self.gears.p.height

    def mesh(self, at, groups=None):
        """Generate mesh for gear pair at position given by `at`.

        .. note: `at` is/must be edited by reference
        """
        sign = np.sign(self.disp) if self.disp != 0 else 1
        if abs(self.disp) < self.stretch_margin:
            stretch = self.disp + sign*self.height/2
        else:
            stretch = sign*(self.height+self.gears.p.stretch+2*self.stretch_margin)/2
        at.dot(translation_matrix([0, 0, stretch]), out=at)
        at.dot(rotation_matrix(self.angle, [0, 0, 1]), out=at)
        p_mesh = self.gears.p.mesh(at)
        at.dot(translation_matrix(
            [self.ap, 0, sign*self.gears.p.stretch/2]), out=at)
        g_mesh = self.gears.g.mesh(at)
        if groups is not None:
            groups[-1].append(p_mesh)
            groups.append([g_mesh])
        return (p_mesh, g_mesh)


def make_gearpair(Z1, x1, Z2, x2, m, b, disp=0, angle=None):
    p = SpurGear(Z1, m, x1, b)
    g = SpurGear(Z2, m, x2, b)
    return GearPair(p, g, disp, angle)
