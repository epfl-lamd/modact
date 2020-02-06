import typing
from operator import attrgetter

import attr
import numpy as np
import trimesh
from cached_property import cached_property
from trimesh.transformations import translation_matrix

from .meshutils import merge_meshes
from .materials import get_material
from .models import Model, OperatingCondition, GearPair


@attr.s(auto_attribs=True)
class Actuator(object):
    components: typing.List[Model] = attr.Factory(list)

    @cached_property
    def mesh(self):
        components = []
        groups = [[]]

        last_position = translation_matrix([0, 0, 0])
        last_height = 0
        for comp in self.components:
            disp = comp.disp
            sign = np.sign(disp) if disp != 0 else 1
            # move coordinates to edge
            last_position.dot(translation_matrix([0., 0., sign*last_height/2]),
                              out=last_position)
            last_height = comp.height
            mesh = comp.mesh(last_position, groups)
            components.extend(mesh)

        space = merge_meshes(components)

        return components, groups, space

    @cached_property
    def i(self):
        i_tot = 1
        for comp in self.components:
            i_tot *= comp.i
        return i_tot

    @cached_property
    def i_gp(self):
        i_tot = 1
        for comp in self.components:
            if isinstance(comp, GearPair):
                i_tot *= comp.i
        return i_tot

    @cached_property
    def volume(self):
        return sum(map(attrgetter("volume"), self.components))

    def matched_speed_control(self, conditions):
        i_tot = self.i

        in_conditions = [
            OperatingCondition(cond.speed * i_tot, 0, cond.V, cond.imax)
            for cond in conditions]
        return in_conditions

    def get_speed_torque(self, in_conditions, target=False):
        op_per_comp = [[] for _ in range(len(self.components))]
        out_conditions = []

        for i, cond in enumerate(in_conditions):
            # c_speed = in_speed[i]
            # c_torque = in_torque[i]
            next_op = in_conditions[i]
            for j, comp in enumerate(self.components):
                # op = OperatingCondition(c_speed, c_torque, cond.V, cond.imax)
                op_per_comp[j].append(next_op)
                next_op = comp.get_speed_torque(next_op)
                if next_op.torque <= 0:
                    # No torque available
                    # Use small amount for numerical reasons
                    next_op.torque = 1.e-6

            if target and next_op.torque > target[i].torque:
                # Output torque is higher than required torque
                # Scale down all torques
                alpha = next_op.torque/target[i].torque
                next_op.torque /= alpha
                # Correct OPs for constraints
                for j in range(len(op_per_comp)):
                    op_orig = op_per_comp[j][-1]
                    op_orig.torque /= alpha

            out_conditions.append(next_op)

        return out_conditions, op_per_comp

    def gear_constraints(self, op_per_comp):
        gear_idx = [i for i, comp in enumerate(self.components)
                    if isinstance(comp, GearPair)]
        if not gear_idx:
            return np.zeros((0, 0)), np.zeros((0, 0, 0))
        n_gears = len(gear_idx)
        n_conditions = len(op_per_comp[gear_idx[0]])
        kinematic = np.zeros((n_gears, 4))
        resistance = np.zeros((n_gears, n_conditions, 4))
        for i, idx in enumerate(gear_idx):
            comp = self.components[idx]
            conditions = op_per_comp[idx]
            kinematic[i, :] = (comp.interference, comp.contact_ratio,
                               *comp.specific_speed)
            for j, cond in enumerate(conditions):
                resistance[i, j, :] = (*comp.security_h(cond),
                                       *comp.security_f(cond))
        
        return kinematic, resistance

    def cost(self, with_hull=False):
        comp_cost = list(map(attrgetter("cost"), self.components))
        if with_hull:
            _, _, space = self.mesh
            hull_area = space.convex_hull.area
            body_volume = hull_area*1.5  # 1.5 mm thickness
            pom = get_material('POM')
            cost_hull = body_volume/1e9 * pom.rho * pom.cost
            comp_cost.append(cost_hull)
        return comp_cost

    def internal_collisions(self):
        cm = trimesh.collision.CollisionManager()
        meshes, _, space = self.mesh
        for i, m in enumerate(meshes):
            cm.add_object(i, m)
        _, names = cm.in_collision_internal(return_names=True)
        return len(names)/len(space.faces)
