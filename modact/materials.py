import attr


@attr.s(auto_attribs=True, slots=True)
class Material(object):
    "Class to store material properties"
    name: str
    E: float
    rho: float
    nu: float
    cost: float
    sigma_f_lim: float
    sigma_h_lim: float


material_data = {
    'steel': {
        'E': 210e9,
        'nu': 0.3,
        'sigma_h_lim': 400e6,
        'sigma_f_lim': 200e6,
        'cost': 0.4,
        'rho': 7800.
    },
    'POM': {
        'E': 2.85e9,
        'nu': 0.44,
        'cost': 2.20,
        'rho': 1400,
        'sigma_f_lim': 90.e6,
        'sigma_h_lim': 90.e6
    }
}


def get_material(name):
    return Material(name=name, **material_data[name])
