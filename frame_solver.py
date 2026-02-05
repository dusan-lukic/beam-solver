# LLM-generated
# human reviewed

from dataclasses import dataclass
from typing import List
from fundamental_data import *
import math
import numpy as np

# --- Hydrated Data Structures ---

@dataclass
class EquationsBricklayer:
    var_ptr: int

@dataclass
class HydratedSupport(EquationsBricklayer):
    support: Support
    point: 'HydratedPoint'
    
    def __init__(self, var_ptr: int, support: Support, point: 'HydratedPoint'):
        super().__init__(var_ptr)
        self.support = support
        self.point = point

@dataclass
class HydratedPoint(EquationsBricklayer):
    point: Point
    sup: Optional['HydratedSupport']
    lines_a: List['HydratedLine']
    lines_b: List['HydratedLine']
    
    def __init__(self, var_ptr: int, point: Point, sup: Optional['HydratedSupport'] = None):
        super().__init__(var_ptr)
        self.point = point
        self.sup = sup
        self.lines_a = []
        self.lines_b = []
    
    def total_load_x(self) -> float:
        return sum(load.Px for load in self.point.loads)
    
    def total_load_y(self) -> float:
        return sum(load.Py for load in self.point.loads)

#TODO: document perf optimization oportunities
#TODO: encapsulate with underscore convention + linter
@dataclass
class HydratedLine(EquationsBricklayer):
    line: Line
    point_a: HydratedPoint
    point_b: HydratedPoint

    def __init__(self, var_ptr: int, line: Line, point_a: HydratedPoint, point_b: HydratedPoint):
        super().__init__(var_ptr)
        self.line = line
        self.point_a = point_a
        self.point_b = point_b
    
    def total_load_x(self) -> float:
        return self.line.ul.qx * self.length() if self.line.ul else 0.0 \
            + sum(ptl.ptl.Px for ptl in self.line.ptls) if self.line.ptls else 0.0
    
    def total_load_y(self) -> float:
        return self.line.ul.qy * self.length() if self.line.ul else 0.0 \
            + sum(ptl.ptl.Py for ptl in self.line.ptls) if self.line.ptls else 0.
    
    def total_load_moment_wrt_B(self) -> float:
        return self.line.ul.qx * self.length() * self.y_length() / 2 if self.line.ul else 0.0 \
            - self.line.ul.qy * self.length() * self.x_length() / 2 if self.line.ul else 0.0 \
            + sum(ptl.ptl.Px * (1 - ptl.c/self.length()) * self.y_length() for ptl in self.line.ptls) if self.line.ptls else 0.0 \
            - sum(ptl.ptl.Py * (1 - ptl.c/self.length()) * self.x_length() for ptl in self.line.ptls) if self.line.ptls else 0.0
    
    def x_length(self) -> float:
        return self.point_b.point.x - self.point_a.point.x
    
    def y_length(self) -> float:
        return self.point_b.point.y - self.point_a.point.y
    
    def length(self) -> float:
        return math.sqrt(self.x_length()**2 + self.y_length()**2)
    
    def angle_radians(self) -> float:
        return math.atan2(self.y_length(), self.x_length())
    
    def sin_a(self) -> float:
        return self.y_length() / self.length()
    
    def cos_a(self) -> float:
        return self.x_length() / self.length()

    def q_perp(self) -> float:
        return (-self.line.ul.qx*self.y_length() + self.line.ul.qy*self.x_length()) / self.length() if self.line.ul else 0.0
    
    def q_ax(self) -> float:
        return (self.line.ul.qx*self.x_length() + self.line.ul.qy*self.y_length()) / self.length() if self.line.ul else 0.0

    def p_perp(self) -> List[Tuple[float, float]]:
        return ((ptl.c, -ptl.ptl.Px * self.sin_a() + ptl.ptl.Py * self.cos_a()) for ptl in self.line.ptls)
    
    def p_ax(self) -> List[Tuple[float, float]]:
        return ((ptl.c, ptl.ptl.Px * self.cos_a() + ptl.ptl.Py * self.sin_a()) for ptl in self.line.ptls)

def build_hydrated_structures(points: List[Point], lines: List[Line], supports: List[Support]) -> Tuple[List[HydratedPoint], List[HydratedLine], List[HydratedSupport]]:
    # variable positions
    next_member_var = 0
    first_joint_var = 3*len(lines)
    next_support_var = first_joint_var + 3*len(points)

    # Create HydratedPoints
    point_dict = {p.id: HydratedPoint(first_joint_var + 3*i, p) for i, p in enumerate(points)}

    # Create HydratedSupports and link to HydratedPoints
    support_list = []
    for s in supports:
        hp = point_dict[s.anch]
        hs = HydratedSupport(next_support_var, s, hp)
        next_support_var += (int(s.ver) + int(s.hor) + int(s.rot))
        hp.sup = hs
        support_list.append(hs)
    
    # Create HydratedLines and link to HydratedPoints
    line_list = []
    for l in lines:
        hp_a = point_dict[l.a]
        hp_b = point_dict[l.b]
        hl = HydratedLine(next_member_var, l, hp_a, hp_b)
        next_member_var += 3
        hp_a.lines_a.append(hl)
        hp_b.lines_b.append(hl)
        line_list.append(hl)
    
    return list(point_dict.values()), line_list, support_list

def assemble_system(hydrated_points: List[HydratedPoint], hydrated_lines: List[HydratedLine], hydrated_supports: List[HydratedSupport]) -> Tuple:
    # Calculate total equations and unknowns
    eqs_cnt = 3 * (len(hydrated_points) + len(hydrated_lines)) \
        + sum(int(s.support.hor) + int(s.support.ver) + int(s.support.rot) for s in hydrated_supports)

    A = np.zeros((eqs_cnt, eqs_cnt))
    b = np.zeros(eqs_cnt)

    eq_idx = 0
    
    # --- Equilibrium equations (3 per joint) ---
    for hp in hydrated_points:
        # F_x balance
        for m in hp.lines_b:
            A[eq_idx, m.var_ptr] = 1
        for m in hp.lines_a:
            A[eq_idx, m.var_ptr] = -1
        if (hp.sup and hp.sup.support.hor):
            A[eq_idx, hp.sup.var_ptr] = -1
        b[eq_idx] = sum(l.total_load_x() for l in hp.lines_a) + hp.total_load_x()
        eq_idx += 1

        # F_y balance
        for m in hp.lines_b:
            A[eq_idx, m.var_ptr + 1] = 1
        for m in hp.lines_a:
            A[eq_idx, m.var_ptr + 1] = -1
        if (hp.sup and hp.sup.support.ver):
            A[eq_idx, hp.sup.var_ptr + (1 if hp.sup.support.hor else 0)] = -1
        b[eq_idx] = sum(l.total_load_y() for l in hp.lines_a) + hp.total_load_y()
        eq_idx += 1

        # M balance
        for m in hp.lines_b:
            A[eq_idx, m.var_ptr + 2] = 1
        for m in hp.lines_a:
            A[eq_idx, m.var_ptr] = -m.y_length()
            A[eq_idx, m.var_ptr + 1] = m.x_length()
            A[eq_idx, m.var_ptr + 2] = -1
        if (hp.sup and hp.sup.support.rot):
            offset = (1 if hp.sup.support.hor else 0) + (1 if hp.sup.support.ver else 0)
            A[eq_idx, hp.sup.var_ptr + offset] = -1
        b[eq_idx] = sum(l.total_load_moment_wrt_B() for l in hp.lines_a) # nothing else because point loads on joints don't support moments
        eq_idx += 1
    
    # --- Compatibility equations (3 per line) ---
    for hl in hydrated_lines:
        # common member props
        l = hl.length()
        sin_a = hl.sin_a()
        cos_a = hl.cos_a()
        EI = hl.line.bp.E * hl.line.bp.I # flexural rigidity
        AE = hl.line.bp.E * hl.line.bp.A # axial rigidity
        
        # joint A rotation
        A[eq_idx, hl.point_a.var_ptr + 2] = 1
        A[eq_idx, hl.var_ptr] = -sin_a * (l**2) / 3 / EI
        A[eq_idx, hl.var_ptr + 1] = cos_a * (l**2) / 3 / EI
        A[eq_idx, hl.var_ptr + 2] = l / 2 / EI
        A[eq_idx, hl.point_a.var_ptr] = -sin_a / l
        A[eq_idx, hl.point_a.var_ptr + 1] = cos_a / l
        A[eq_idx, hl.point_b.var_ptr] = sin_a / l
        A[eq_idx, hl.point_b.var_ptr + 1] = -cos_a / l
        b[eq_idx] = -sum(p*(c**2)*(3*l-c)/6/EI/l for (c, p) in hl.p_perp())-hl.q_perp()*(l**3)/8/EI
        eq_idx += 1

        # joint B rotation
        A[eq_idx, hl.point_b.var_ptr + 2] = 1
        A[eq_idx, hl.var_ptr] = sin_a * (l**2) / 6 / EI
        A[eq_idx, hl.var_ptr + 1] = -cos_a * (l**2) / 6 / EI
        A[eq_idx, hl.var_ptr + 2] = -l / 2 / EI
        A[eq_idx, hl.point_a.var_ptr] = -sin_a / l
        A[eq_idx, hl.point_a.var_ptr + 1] = cos_a / l
        A[eq_idx, hl.point_b.var_ptr] = sin_a / l
        A[eq_idx, hl.point_b.var_ptr + 1] = -cos_a / l
        b[eq_idx] = sum(p*(c**3)/6/EI/l for (c, p) in hl.p_perp())+hl.q_perp()*(l**3)/24/EI
        eq_idx += 1
        
        # member length change
        A[eq_idx, hl.point_a.var_ptr] = -cos_a
        A[eq_idx, hl.point_a.var_ptr + 1] = -sin_a
        A[eq_idx, hl.point_b.var_ptr] = cos_a
        A[eq_idx, hl.point_b.var_ptr + 1] = sin_a
        A[eq_idx, hl.var_ptr] = -l*cos_a / AE
        A[eq_idx, hl.var_ptr + 1] = -l*sin_a / AE
        b[eq_idx] = (l**2)/2/AE*hl.q_ax()+sum(p*c/AE for (c, p) in hl.p_ax())
        eq_idx += 1
        
    
    # --- Support equations ---
    for hs in hydrated_supports:
        # Horizontal support equation
        if hs.support.hor:
            A[eq_idx, hs.point.var_ptr] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was support settlement in the x-direction
            eq_idx += 1
        
        # Vertical support equation
        if hs.support.ver:
            A[eq_idx, hs.point.var_ptr + 1] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was support settlement in the y-direction
            eq_idx += 1
        
        # Rotational support equation
        if hs.support.rot:
            A[eq_idx, hs.point.var_ptr + 2] = 1 # technically any non-zero number works because b[eq_idx] is zero
            # intentionally leave b[eq_idx] = 0. This would not be the case if there was rotational support settlement
            eq_idx += 1
    
    return A, b

def solve_scene(scene: Scene) -> FrameSolution:
    hydrated_points, hydrated_lines, hydrated_supports = build_hydrated_structures(scene.points, scene.lines, scene.supports)
    (A, b) = assemble_system(hydrated_points, hydrated_lines, hydrated_supports)
    
    x = np.linalg.solve(A, b)
    
    # Build and return a FrameSolution derived from x and hydrated structures
    return build_frame_solution(hydrated_points, hydrated_lines, hydrated_supports, x)


def build_frame_solution(hydrated_points: List[HydratedPoint], hydrated_lines: List[HydratedLine], hydrated_supports: List[HydratedSupport], x):
    """
    Produce a FrameSolution from solved system variable vector `x` and hydrated model
    objects.

    For each hydrated object this creates a corresponding entry in the output
    dictionaries (point_deflections, line_stresses_and_strains, support_reactions).
    """

    # Points -> PointDeflection
    point_deflections = {}
    for hp in hydrated_points:
        point_deflections[hp.point.id] = PointDeflection(d_x=x[hp.var_ptr], d_y=x[hp.var_ptr+1], d_t=x[hp.var_ptr+2])

    # Supports -> SupportReactions
    support_reactions = {}
    for hs in hydrated_supports:
        offset = 0
        Rx = Ry = Rm = 0.0
        if hs.support.hor:
            Rx = x[hs.var_ptr + offset]
            offset += 1
        if hs.support.ver:
            Ry = x[hs.var_ptr + offset]
            offset += 1
        if hs.support.rot:
            Rm = x[hs.var_ptr + offset]
        support_reactions[hs.support.anch] = SupportReactions(R_x=Rx, R_y=Ry, R_m=Rm)

    # Lines -> LineStress and LineStrain
    line_stresses_and_strains = {}
    for hl in hydrated_lines:
        # props
        sin_a = hl.sin_a()
        cos_a = hl.cos_a()
        EI = hl.line.bp.E * hl.line.bp.I
        AE = hl.line.bp.E * hl.line.bp.A
        l = hl.length()

        # stresses
        Xb = x[hl.var_ptr]
        Yb = x[hl.var_ptr + 1]
        Mb = x[hl.var_ptr + 2]
        
        Bx = Xb*cos_a + Yb*sin_a
        Bp = -Xb*sin_a + Yb*cos_a
        (x_max, M_bnd_max) = maxPosAndBendingMoment(hl, Mb, Bp)

        lstress = LineStress(
            S = axial_force_diagram(hl, Bx),
            V = shear_force_diagram(hl, Bp),
            M = bending_moment_diagram(hl, Mb, Bp),
            s_max = M_bnd_max/hl.line.bp.I*hl.line.bp.h/2,
            c_max = x_max)

        # strains
        lstrain = LineStrain(
            e = Bx*l/AE + (l**2)/2/AE*hl.q_ax() + sum(p*c/AE for (c, p) in hl.p_ax()),
            theta_a = - Mb*l/2/EI - Bp*(l**2)/3/EI - sum(p*(c**2)*(3*l-c)/6/EI/l for (c, p) in hl.p_perp()) - hl.q_perp()*(l**3)/8/EI,
            theta_b =   Mb*l/2/EI + Bp*(l**2)/6/EI + sum(p*(c**3)        /6/EI/l for (c, p) in hl.p_perp()) + hl.q_perp()*(l**3)/24/EI,
            dc = deflection_curve(hl, Mb, Bp))

        line_stresses_and_strains[hl.line.id] = (lstress, lstrain)

    return FrameSolution(line_stresses_and_strains=line_stresses_and_strains,
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