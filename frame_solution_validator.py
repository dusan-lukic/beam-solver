# AI generated
# human reviewed


import math
from scene_hydrator import HydratedScene
from fundamental_data import FrameSolution, JointDeflection
from frame_solver import forcesOnA, forcesOnB


def validate_equilibrium(scene: HydratedScene, solution: FrameSolution):
    for hl in scene.lines:
        lstress = solution.line_stresses[hl.line.id]
        total_ax = lstress.S[-1].y - lstress.S[0].y + sum(p_ax[1] for p_ax in hl.p_ax()) + hl.q_ax()*hl.length()
        if abs(total_ax) > 1:
            print(f"Equilibrium validation failed for line {hl.line.id}: S at A + S at B + sum(p_ax) + q_ax*length should be 0 but is {total_ax}")
        
        total_perp = lstress.V[-1].y - lstress.V[0].y + sum(p_perp[1] for p_perp in hl.p_perp()) + hl.q_perp()*hl.length()
        if abs(total_perp) > 1:
            print(f"Equilibrium validation failed for line {hl.line.id}: V at A + V at B + sum(p_perp) + q_perp*length should be 0 but is {total_perp}")
        
        total_m_wrt_A = lstress.M[-1].y - lstress.M[0].y \
            + sum(p_perp[1]*p_perp[0] for p_perp in hl.p_perp()) \
            + hl.q_perp()*hl.length()*hl.length()/2 \
            + lstress.V[-1].y*hl.length()
        if abs(total_m_wrt_A) > 1:
            print(f"Equilibrium validation failed for line {hl.line.id}: M at A + M at B + sum(p_perp*c) + q_perp*length^2/2 should be 0 but is {total_m_wrt_A}")
    
    for p in scene.points:
        total_Fx = 0.0
        total_Fy = 0.0
        total_M = 0.0
        for l in p.lines_a:
            lstress = solution.line_stresses[l.line.id]
            Fx, Fy, M = forcesOnA(lstress, l.point_a.point, l.point_b.point)
            total_Fx -= Fx
            total_Fy -= Fy
            total_M -= M
        for l in p.lines_b:
            lstress = solution.line_stresses[l.line.id]
            Fx, Fy, M = forcesOnB(lstress, l.point_a.point, l.point_b.point)
            total_Fx -= Fx
            total_Fy -= Fy
            total_M -= M
        for load in p.point.loads:
            total_Fx += load.Px
            total_Fy += load.Py
        if p.sup is not None:
            sr = solution.support_reactions[p.sup.support.anch]
            total_Fx += sr.R_x
            total_Fy += sr.R_y
            total_M += sr.R_m
        
        if (abs(total_Fx) > 1 or abs(total_Fy) > 1 or abs(total_M) > 1):
            print(f"Equilibrium validation failed at point {p.point.id}: total_Fx={total_Fx}, total_Fy={total_Fy}, total_M={total_M}")

    pass


def validate_deflections(scene: HydratedScene, solution: FrameSolution):
    for hl in scene.lines:
        lstrain = solution.line_strains[hl.line.id]
        a_def: JointDeflection = solution.point_deflections[hl.point_a.point.id]
        x_a = hl.point_a.point.x + a_def.d_x
        y_a = hl.point_a.point.y + a_def.d_y
        b_def: JointDeflection = solution.point_deflections[hl.point_b.point.id]
        x_b = hl.point_b.point.x + b_def.d_x
        y_b = hl.point_b.point.y + b_def.d_y

        length_according_to_endpoints = math.hypot(x_b - x_a, y_b - y_a)
        length_according_to_elongation = hl.length() * (1 + lstrain.e)
        # TODO: make tollerances relative to calculated vars.
        # TODO: calculate units o that expected vars are close to each others
        if abs(length_according_to_endpoints - length_according_to_elongation) > 1e-3: # tollerance == 1um.
            print(f"Deflection validation failed for line {hl.line.id}: length discrepancy is {length_according_to_endpoints - length_according_to_elongation}")
        
        cl_slope_inc = math.atan2(y_b - y_a, x_b - x_a) - math.atan2(hl.y_length(), hl.x_length())
        if (cl_slope_inc > math.pi): cl_slope_inc -= math.tau
        if (cl_slope_inc < -math.pi): cl_slope_inc += math.tau
        if abs(cl_slope_inc + lstrain.theta_a - a_def.d_t) > 1e-6:
            print(f"Deflection validation failed for line {hl.line.id} at point A: slope discrepancy is {cl_slope_inc + lstrain.theta_a - a_def.d_t}")
        if abs(cl_slope_inc + lstrain.theta_b - b_def.d_t) > 1e-6:
            print(f"Deflection validation failed for line {hl.line.id} at point B: slope discrepancy is {cl_slope_inc + lstrain.theta_b - b_def.d_t}")

    pass