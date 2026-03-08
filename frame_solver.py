# LLM-generated
# human reviewed

from typing import List, Dict, Tuple, Optional
from fundamental_data import FrameJoint, FrameMember, PointForce, UniformForce, Support, MemberStress, MemberStrain, JointDeflection, FrameSolution, SupportReactions, Scene
import numpy as np
import math
from scene_hydrator import HydratedScene, HydratedLine

#TODO: document perf optimization oportunities
#TODO: encapsulate with underscore convention + linter

class VariablePositions:
    member_vars: Dict[str, int]
    joint_vars: Dict[str, int]
    support_vars: Dict[str, int]

    def __init__(self, mv: Dict[str, int], jv: Dict[str, int], sv: Dict[str, int]):
        self.member_vars = mv
        self.joint_vars = jv
        self.support_vars = sv


def solve_scene(scene: HydratedScene) -> FrameSolution:
    var_ptrs = assign_variable_positions(scene)
    (A, b) = assemble_system(scene, var_ptrs)
    
    x = np.linalg.solve(A, b)
    
    # Build and return a FrameSolution derived from x and hydrated structures
    return decode_frame_solution(scene, x, var_ptrs)


def assign_variable_positions(scene: HydratedScene) -> VariablePositions:
    next_var = 0
    member_vars: Dict[str, int] = {}
    for hl in scene.lines:
        member_vars[hl.line.id] = next_var
        next_var += 3
    
    joint_vars: Dict[str, int] = {}
    for hp in scene.points:
        joint_vars[hp.point.id] = next_var
        next_var += 3
    
    support_vars: Dict[str, int] = {}
    for hs in scene.supports:
        support_vars[hs.support.anch] = next_var
        next_var += (1 if hs.support.hor else 0) + (1 if hs.support.ver else 0) + (1 if hs.support.rot else 0)
    
    return VariablePositions(member_vars, joint_vars, support_vars)


def assemble_system(scene: HydratedScene, var_ptrs: VariablePositions) -> Tuple[np.ndarray, np.ndarray]:
    # Calculate total equations and unknowns
    eqs_cnt = 3 * (len(scene.points) + len(scene.lines)) \
        + sum(int(s.support.hor) + int(s.support.ver) + int(s.support.rot) for s in scene.supports)

    A = np.zeros((eqs_cnt, eqs_cnt))
    b = np.zeros(eqs_cnt)

    eq_idx = 0
    
    # --- Equilibrium equations (3 per joint) ---
    for hp in scene.points:
        # F_x balance
        for m in hp.lines_b:
            A[eq_idx, var_ptrs.member_vars[m.line.id]] = 1
        for m in hp.lines_a:
            A[eq_idx, var_ptrs.member_vars[m.line.id]] = -1
        if (hp.sup and hp.sup.support.hor):
            A[eq_idx, var_ptrs.support_vars[hp.sup.support.anch]] = -1
        b[eq_idx] = sum(l.total_load_x() for l in hp.lines_a) + hp.total_load_x()
        eq_idx += 1

        # F_y balance
        for m in hp.lines_b:
            A[eq_idx, var_ptrs.member_vars[m.line.id] + 1] = 1
        for m in hp.lines_a:
            A[eq_idx, var_ptrs.member_vars[m.line.id] + 1] = -1
        if (hp.sup and hp.sup.support.ver):
            A[eq_idx, var_ptrs.support_vars[hp.sup.support.anch] + (1 if hp.sup.support.hor else 0)] = -1
        b[eq_idx] = sum(l.total_load_y() for l in hp.lines_a) + hp.total_load_y()
        eq_idx += 1

        # M balance
        for m in hp.lines_b:
            A[eq_idx, var_ptrs.member_vars[m.line.id] + 2] = 1
        for m in hp.lines_a:
            A[eq_idx, var_ptrs.member_vars[m.line.id]] = m.y_length()
            A[eq_idx, var_ptrs.member_vars[m.line.id] + 1] = -m.x_length()
            A[eq_idx, var_ptrs.member_vars[m.line.id] + 2] = -1
        if (hp.sup and hp.sup.support.rot):
            offset = (1 if hp.sup.support.hor else 0) + (1 if hp.sup.support.ver else 0)
            A[eq_idx, var_ptrs.support_vars[hp.sup.support.anch] + offset] = -1
        b[eq_idx] = sum(l.total_load_moment_wrt_A() for l in hp.lines_a) # nothing else because point loads on joints don't support moments
        eq_idx += 1
    
    # --- Compatibility equations (3 per line) ---
    for hl in scene.lines:
        # common member props
        l = hl.length()
        sin_a = hl.sin_a()
        cos_a = hl.cos_a()
        EI = hl.line.bp.E * hl.line.bp.I # flexural rigidity
        AE = hl.line.bp.E * hl.line.bp.A # axial rigidity
        
        # joint A rotation
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id] + 2] = 1
        A[eq_idx, var_ptrs.member_vars[hl.line.id]] = -sin_a * (l**2) / 3 / EI
        A[eq_idx, var_ptrs.member_vars[hl.line.id] + 1] = cos_a * (l**2) / 3 / EI
        A[eq_idx, var_ptrs.member_vars[hl.line.id] + 2] = l / 2 / EI
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id]] = -sin_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id] + 1] = cos_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id]] = sin_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id] + 1] = -cos_a / l
        b[eq_idx] = -sum(p*(c**2)*(3*l-c)/6/EI/l for (c, p) in hl.p_perp())-hl.q_perp()*(l**3)/8/EI
        eq_idx += 1

        # joint B rotation
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id] + 2] = 1
        A[eq_idx, var_ptrs.member_vars[hl.line.id]] = sin_a * (l**2) / 6 / EI
        A[eq_idx, var_ptrs.member_vars[hl.line.id] + 1] = -cos_a * (l**2) / 6 / EI
        A[eq_idx, var_ptrs.member_vars[hl.line.id] + 2] = -l / 2 / EI
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id]] = -sin_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id] + 1] = cos_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id]] = sin_a / l
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id] + 1] = -cos_a / l
        b[eq_idx] = sum(p*(c**3)/6/EI/l for (c, p) in hl.p_perp())+hl.q_perp()*(l**3)/24/EI
        eq_idx += 1
        
        # member length change
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id]] = -cos_a
        A[eq_idx, var_ptrs.joint_vars[hl.point_a.point.id] + 1] = -sin_a
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id]] = cos_a
        A[eq_idx, var_ptrs.joint_vars[hl.point_b.point.id] + 1] = sin_a
        A[eq_idx, var_ptrs.member_vars[hl.line.id]] = -l*cos_a / AE
        A[eq_idx, var_ptrs.member_vars[hl.line.id] + 1] = -l*sin_a / AE
        b[eq_idx] = (l**2)/2/AE*hl.q_ax()+sum(p*c/AE for (c, p) in hl.p_ax())
        eq_idx += 1
        
    
    # --- Support equations ---
    for hs in scene.supports:
        # Horizontal support equation
        if hs.support.hor:
            A[eq_idx, var_ptrs.joint_vars[hs.point.point.id]] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was support settlement in the x-direction
            eq_idx += 1
        
        # Vertical support equation
        if hs.support.ver:
            A[eq_idx, var_ptrs.joint_vars[hs.point.point.id] + 1] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was support settlement in the y-direction
            eq_idx += 1
        
        # Rotational support equation
        if hs.support.rot:
            A[eq_idx, var_ptrs.joint_vars[hs.point.point.id] + 2] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was rotational support settlement
            eq_idx += 1
    
    return A, b


def decode_frame_solution(scene: HydratedScene, x, var_ptrs: VariablePositions) -> FrameSolution:
    # Points -> JointDeflection
    point_deflections = {}
    for hp in scene.points:
        point_deflections[hp.point.id] = JointDeflection(
            d_x=x[var_ptrs.joint_vars[hp.point.id]],
            d_y=x[var_ptrs.joint_vars[hp.point.id] + 1],
            d_t=x[var_ptrs.joint_vars[hp.point.id] + 2])

    # Supports -> SupportReactions
    support_reactions = {}
    for hs in scene.supports:
        offset = 0
        Rx = Ry = Rm = 0.0
        if hs.support.hor:
            Rx = x[var_ptrs.support_vars[hs.support.anch] + offset]
            offset += 1
        if hs.support.ver:
            Ry = x[var_ptrs.support_vars[hs.support.anch] + offset]
            offset += 1
        if hs.support.rot:
            Rm = x[var_ptrs.support_vars[hs.support.anch] + offset]
        support_reactions[hs.support.anch] = SupportReactions(R_x=Rx, R_y=Ry, R_m=Rm)

    # Lines -> MemberStress and MemberStrain
    line_stresses = {}
    line_strains = {}
    for hl in scene.lines:
        # props
        sin_a = hl.sin_a()
        cos_a = hl.cos_a()
        EI = hl.line.bp.E * hl.line.bp.I
        AE = hl.line.bp.E * hl.line.bp.A
        l = hl.length()

        # stresses
        Xb = x[var_ptrs.member_vars[hl.line.id]]
        Yb = x[var_ptrs.member_vars[hl.line.id] + 1]
        Mb = x[var_ptrs.member_vars[hl.line.id] + 2]

        Bx = Xb*cos_a + Yb*sin_a
        Bp = -Xb*sin_a + Yb*cos_a
        (x_max, M_bnd_max) = maxPosAndBendingMoment(hl, Mb, Bp)

        lstress = MemberStress(
            S = axial_force_diagram(hl, Bx),
            V = shear_force_diagram(hl, Bp),
            M = bending_moment_diagram(hl, Mb, Bp),
            s_max = M_bnd_max/hl.line.bp.I*hl.line.bp.h/2,
            c_max = x_max)

        # strains
        lstrain = MemberStrain(
            e = Bx/AE + hl.q_ax()*l/2/AE + sum(p*c/AE for (c, p) in hl.p_ax())/l,
            theta_a = - Mb*l/2/EI - Bp*(l**2)/3/EI - sum(p*(c**2)*(3*l-c)/6/EI/l for (c, p) in hl.p_perp()) - hl.q_perp()*(l**3)/8/EI,
            theta_b =   Mb*l/2/EI + Bp*(l**2)/6/EI + sum(p*(c**3)        /6/EI/l for (c, p) in hl.p_perp()) + hl.q_perp()*(l**3)/24/EI,
            dc = deflection_curve(hl, Mb, Bp))

        line_stresses[hl.line.id] = lstress
        line_strains[hl.line.id] = lstrain

    return FrameSolution(line_stresses=line_stresses,
                         line_strains=line_strains,
                         point_deflections=point_deflections,
                         support_reactions=support_reactions)


def maxPosAndBendingMoment(hl: HydratedLine, Mb: float, Bp: float) -> Tuple[float, float]:
    l:float = hl.length()
    c_i:float = 0.0
    x_max: float = 0.0
    M_bnd_max:float = abs(Mb + Bp*l + hl.q_perp()*l**2/2 + sum(p_perp*c for (c, p_perp) in hl.p_perp()))
    for x in [*sorted(ptl.c for ptl in hl.line.ptls), l]:
        M_bnd = Mb + Bp*(l-x) + hl.q_perp()*(l-x)**2/2 + sum((p_perp*(c-x) if c>x else 0) for (c, p_perp) in hl.p_perp())
        if abs(M_bnd) > abs(M_bnd_max):
            M_bnd_max = M_bnd
            x_max = x
        
        # now the fun part - check for local extrema due to distributed load
        if hl.line.ul != None:
            inv_q_perp = 1/hl.q_perp()
            x_0 = l + Bp*inv_q_perp + inv_q_perp*sum((p_perp if c >= x else 0) for (c, p_perp) in hl.p_perp())
            if c_i < x_0 < x:
                M_bnd = Mb + Bp*(l-x_0) + hl.q_perp()*(l-x_0)**2/2 + sum((p_perp*(c-x_0) if c>x_0 else 0) for (c, p_perp) in hl.p_perp())
                if abs(M_bnd) > abs(M_bnd_max):
                    M_bnd_max = M_bnd
                    x_max = x_0

        c_i = x

    return x_max, M_bnd_max

def polyline(start_y: float, slope: float, sorted_jumps: List[Tuple[float, float]], end_x: float) -> List[Tuple[float, float]]:
    prev: float = 0.0
    points: List[Tuple[float, float]] = [(prev, start_y)]
    y: float = start_y
    for (x, j) in sorted_jumps:
        y += slope * (x-prev)
        points.append((x, y))
        y += j
        points.append((x, y))
        prev = x
    y += slope * (end_x - prev)
    points.append((end_x, y))
    return points

def shear_force_diagram(hl: HydratedLine, Bp: float) -> List[Tuple[float, float]]: # poly-line plots of (x, V)
    l: float = hl.length()
    sorted_cpp = sorted(hl.p_perp(), key=lambda cpp: cpp[0])  # cpp is (c, p_perp)
    return polyline(
        start_y = hl.q_perp()*l + Bp + sum(p for (c, p) in sorted_cpp),
        slope = -hl.q_perp(),
        sorted_jumps = [(c, -p) for (c, p) in sorted_cpp],
        end_x = l)

def axial_force_diagram(hl: HydratedLine, Bx: float) -> List[Tuple[float, float]]: # poly-line plots of (x, S)
    l: float = hl.length()
    sorted_cpx = sorted(hl.p_ax(), key=lambda cpx: cpx[0])  # cpx is (c, p_ax)
    return polyline(
        start_y = hl.q_ax()*l + Bx + sum(p for (c, p) in sorted_cpx),
        slope = -hl.q_ax(),
        sorted_jumps = [(c, -p) for (c, p) in sorted_cpx],
        end_x = l)

def bending_moment_diagram(hl: HydratedLine, Mb: float, Bp: float) -> List[Tuple[float, float]]: # poly-line plots of (x, M)
    l: float = hl.length()
    x: List[float] = [0, *[c for (c, ptl) in hl.line.ptls], l]
    if hl.line.ul != None:
        x = x + [i* l/20 for i in range(1, 19)]  # 20 segments
    x = sorted(list(set(x)))
    points: List[Tuple[float, float]] = []
    for xi in x:
        M_bnd = Mb + Bp*(l-xi) + hl.q_perp()*(l-xi)**2/2 + sum((p_perp*(c-xi) if c > xi else 0) for (c, p_perp) in hl.p_perp())
        points.append((xi, M_bnd))

    return points

def deflection_curve(hl: HydratedLine, Mb: float, Bp: float) -> List[Tuple[float, float]]: # poly-line plots of (x, d)
    l: float = hl.length()
    EIinv = 1/(hl.line.bp.E * hl.line.bp.I)
    x = sorted(list(set([(c*l/20.0) for c in range(0, 20)] + [c for (c, ptl) in hl.line.ptls])))   # 20 segments + critical points
    points: List[Tuple[float, float]] = []
    for xi in x:
        d = EIinv*(Mb/2*(xi**2-l*xi) + hl.q_perp()/24*(xi**4-4*l*xi**3+6*l**2*xi**2-3*l**3*xi)  + Bp*(xi**2/6*(3*l-xi)-l**2*xi/3) +
            sum(-p_perp*c**2*xi/6/l*(3*l-c)+p_perp/6*(xi**2*(3*c-xi) if xi <= c else c**2*(3*xi-c)) for (c, p_perp) in hl.p_perp()))
        points.append((xi, d))
    
    return points

#TODO: think of the option to make these functions of MemberStress. Need for reference to endpoints makes in nontrivial.

def forcesOnA(lstress: MemberStress, point_a: FrameJoint, point_b: FrameJoint) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    S_a = -lstress.S[0][1]  # S at A is the second value in the first tuple of the S diagram
    V_a = -lstress.V[0][1]   # and similar for other diagrams
    M_a = -lstress.M[0][1]

    dx = point_b.x - point_a.x
    dy = point_b.y - point_a.y
    l = math.hypot(dx, dy)
    cos_a = dx / l
    sin_a = dy / l

    # Forces at A (internal force acting on joint A)
    Fx_A = S_a * cos_a - V_a * sin_a
    Fy_A = S_a * sin_a + V_a * cos_a

    return Fx_A, Fy_A, M_a

def forcesOnB(lstress: MemberStress, point_a: FrameJoint, point_b: FrameJoint) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    S_b = lstress.S[-1][1]  # S at B is the second value in the last tuple of the S diagram
    V_b = lstress.V[-1][1]
    M_b = lstress.M[-1][1]

    dx = point_b.x - point_a.x
    dy = point_b.y - point_a.y
    l = math.hypot(dx, dy)
    cos_a = dx / l
    sin_a = dy / l

    # Forces at B: (internal force acting on joint B)
    Fx_B = S_b * cos_a - V_b * sin_a
    Fy_B = S_b * sin_a + V_b * cos_a

    return Fx_B, Fy_B, M_b