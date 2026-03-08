# LLM-generated
# human reviewed

from typing import List
from fundamental_data import *
from dataclasses import dataclass
import math


# --- Hydrated Data Structures ---


@dataclass
class HydratedSupport:
    support: Support
    point: 'HydratedPoint'
    
    def __init__(self, support: Support, point: 'HydratedPoint'):
        self.support = support
        self.point = point


@dataclass
class HydratedPoint:
    point: Point
    sup: Optional['HydratedSupport']
    lines_a: List['HydratedLine']
    lines_b: List['HydratedLine']
    
    def __init__(self, point: Point, sup: Optional['HydratedSupport'] = None):
        self.point = point
        self.sup = sup
        self.lines_a = []
        self.lines_b = []
    
    def total_load_x(self) -> float:
        return sum(load.Px for load in self.point.loads)
    
    def total_load_y(self) -> float:
        return sum(load.Py for load in self.point.loads)


@dataclass
class HydratedLine:
    line: Line
    point_a: HydratedPoint
    point_b: HydratedPoint

    def __init__(self, line: Line, point_a: HydratedPoint, point_b: HydratedPoint):
        self.line = line
        self.point_a = point_a
        self.point_b = point_b
    
    def total_load_x(self) -> float:
        return self.line.ul.qx * self.length() if self.line.ul else 0.0 \
            + sum(ptl.ptl.Px for ptl in self.line.ptls) if self.line.ptls else 0.0
    
    def total_load_y(self) -> float:
        return self.line.ul.qy * self.length() if self.line.ul else 0.0 \
            + sum(ptl.ptl.Py for ptl in self.line.ptls) if self.line.ptls else 0.0
    
    def total_load_moment_wrt_A(self) -> float:
        return -self.line.ul.qx * self.length() * self.y_length() / 2 if self.line.ul else 0.0 \
            + self.line.ul.qy * self.length() * self.x_length() / 2 if self.line.ul else 0.0 \
            - sum(ptl.ptl.Px * ptl.c/self.length() * self.y_length() for ptl in self.line.ptls) if self.line.ptls else 0.0 \
            + sum(ptl.ptl.Py * ptl.c/self.length() * self.x_length() for ptl in self.line.ptls) if self.line.ptls else 0.0
    
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


@dataclass
class HydratedScene:
    points: List[HydratedPoint]
    lines: List[HydratedLine]
    supports: List[HydratedSupport]

    def __init__(self, points: List[HydratedPoint], lines: List[HydratedLine], supports: List[HydratedSupport]):
        self.points = points
        self.lines = lines
        self.supports = supports


def build_hydrated_structures(scene: Scene) -> HydratedScene:
    # Create HydratedPoints
    point_dict = {p.id: HydratedPoint(p) for p in scene.points}

    # Create HydratedSupports and link to HydratedPoints
    support_list = []
    for s in scene.supports:
        hp = point_dict[s.anch]
        hs = HydratedSupport(s, hp)
        hp.sup = hs
        support_list.append(hs)
    
    # Create HydratedLines and link to HydratedPoints
    line_list = []
    for l in scene.lines:
        hp_a = point_dict[l.a]
        hp_b = point_dict[l.b]
        hl = HydratedLine(l, hp_a, hp_b)
        hp_a.lines_a.append(hl)
        hp_b.lines_b.append(hl)
        line_list.append(hl)
    
    return HydratedScene(list(point_dict.values()), line_list, support_list)