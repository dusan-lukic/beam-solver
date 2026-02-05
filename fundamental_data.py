# LLM-generated
# human reviewed

from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict
import json

# --- Data structures representing frame, loads, supports, stress and strain data ---
#
# The data encoded here *can be incompatible*, like duplicated points, or crossing
# lines, which are unfeasible in 2D. It is not the responsibility of this module to
# validate such inconsistencies; that is the responsibility of higher-level modules.
# Further incompatibility examples include stress descriptors which don't satisfy
# equilibrium equations, or unsupported points with applied loads.
# What is the responsibility of this module is to provide enough data for modules
# such as frame viewer or solver to perform their tasks (drawing, forming and solving
# systems of equations, validating data for these purposes etc).

@dataclass
class PointLoad:
    Px: float
    Py: float

@dataclass
class UniformLoad:
    qx: float
    qy: float

@dataclass
class Support:
    ver: bool
    hor: bool
    rot: bool
    anch: str

@dataclass
class Point:
    id: str
    x: float
    y: float
    loads: List[PointLoad]

@dataclass
class Line:
    id: str
    a: str
    b: str
    @dataclass
    class BeamProps:
        E: float
        I: float
        A: float
        h: float
    bp: BeamProps
    ul: Optional[UniformLoad]
    @dataclass
    class PointLoadOnLine:
        c: float
        ptl: PointLoad
    ptls: List[PointLoadOnLine]

@dataclass
class LineStress:
    """
    LineStress describes internal forces and maximal stress info for a beam.

    Fields:
        S: Axial force diagram. Positive along the beam in the A-to-B direction.
        V: Shear force diagram. Positive upwards when the beam is horizontal and A is on the left.
        P_b: Perpendicular force at endpoint B. Positive upwards when the beam is horizontal and A is on the left.
        M: Bending moment iagram. Positive CCW.
        s_max: Maximal stress in the beam (magnitude or signed as appropriate for subsequent processing).
        c_max: Position along the beam length where the maximal stress occurs (distance from A; same units as point coordinates).
    """
    S: List[float]
    V: List[float]
    M: List[float]
    s_max: float
    c_max: float

@dataclass
class LineStrain:
    """
    LineStrain describes axial and angular strain of a beam.

    Fields:
        e: Elongation of the centerline of the beam (positive means elongation, negative means shortening).
        theta_a: Angular rotation at endpoint A relative to the centerline (radians, positive CCW).
        theta_b: Angular rotation at endpoint B relative to the centerline (radians, positive CCW).
        dc: deflection curve sampled at several equidistant points along the beam length.
    """
    e: float
    theta_a: float
    theta_b: float
    dc: List[float]

@dataclass
class PointDeflection:
    """
    PointDeflection contains displacement and rotation of a joint.

    Fields:
        d_x: Deflection in the global x direction.
        d_y: Deflection in the global y direction.
        d_t: Angular deflection (rotation) at the point, positive CCW (radians).
    """
    d_x: float
    d_y: float
    d_t: float

@dataclass
class SupportReactions:
    """
    SupportReactions contains reaction forces/moment at a support.

    Fields:
        R_x: Reaction force on the joint in x direction (positive in global +x).
        R_y: Reaction force on the joint in y direction (positive in global +y).
        R_m: Reaction moment (positive CCW) by which the support acts on the attached joint.
    """
    R_x: float
    R_y: float
    R_m: float

@dataclass
class Scene:
    points: List[Point]
    lines: List[Line]
    supports: List[Support]

@dataclass
class FrameSolution:
    line_stresses_and_strains: Dict[str, Tuple[LineStress, LineStrain]]
    point_deflections: Dict[str, PointDeflection]
    support_reactions: Dict[str, SupportReactions]

# beam props for few common beams
#   60x40x3 - "A": 5.41e-4, "E": 2e11, "I": 2.417e-9, "h": 0.06
#
SAMPLE_DATA = """
{
    "points": [
        {"id": "P1", "x": 0, "y": 0, "loads": [] },
        {"id": "P2", "x": 6000, "y": 0, "loads": [] },
        {"id": "P3", "x": 3000, "y": 1000, "loads": [{"Px": 0, "Py": -10000}] }
    ],
    "supports": [
        { "anch": "P1", "ver": true, "hor": true, "rot": false },
        { "anch": "P2", "ver": true, "hor": false, "rot": false }
    ],
    "lines": [
        { "id": "L1", "a": "P1", "b": "P2", "bp": { "A": 5.41e-4, "E": 2e11, "I": 2.417e-9, "h": 0.06 }, "ptls": [] },
        { "id": "L2", "a": "P1", "b": "P3", "bp": { "A": 5.41e-4, "E": 2e11, "I": 2.417e-9, "h": 0.06 }, "ptls": [] },
        { "id": "L3", "a": "P2", "b": "P3", "bp": { "A": 5.41e-4, "E": 2e11, "I": 2.417e-9, "h": 0.06 }, "ptls": [] }
    ]
}
"""

def parse_list(raw_list: List, element_parser: callable, list_name: str) -> List:
    if not isinstance(raw_list, list):
        raise ValueError(f"'{list_name}' must be a list")
    return [element_parser(re) for re in raw_list]

def parse_point_load(raw_load: dict) -> PointLoad:
    if not isinstance(raw_load, dict):
        raise ValueError("Point load must be an object")
    return PointLoad(
        Px = float(raw_load.get("Px")),
        Py = float(raw_load.get("Py")))

def parse_point(raw_pt: dict) -> Point:
    if not isinstance(raw_pt, dict):
        raise ValueError("Point must be an object.")
    return Point(
        id = str(raw_pt["id"]),
        x = float(raw_pt["x"]),
        y = float(raw_pt["y"]),
        loads = parse_list(raw_pt.get("loads", []), parse_point_load, "loads"))

def parse_support(raw_support: dict) -> Support:
    if not isinstance(raw_support, dict):
        raise ValueError("Support must be an object.")
    return Support(
        ver = bool(raw_support.get("ver")),
        hor = bool(raw_support.get("hor")),
        rot = bool(raw_support.get("rot")),
        anch = str(raw_support.get("anch")))

def parse_uniform_load(raw_ul: dict) -> UniformLoad:
    if not isinstance(raw_ul, dict):
        raise ValueError("Uniform load must be an object")
    return UniformLoad(
        qx = float(raw_ul.get("qx")),
        qy = float(raw_ul.get("qy")))

def parse_point_load_on_line(raw_pll: dict) -> Line.PointLoadOnLine:
    if not isinstance(raw_pll, dict):
        raise ValueError("Point load on line must be an object")
    return Line.PointLoadOnLine(
        c = float(raw_pll.get("c")),
        ptl = parse_point_load(raw_pll.get("ptl")))

def parse_beam_props(raw_bp: dict) -> Line.BeamProps:
    if not isinstance(raw_bp, dict):
        raise ValueError("Beam properties must be an object")
    return Line.BeamProps(
        E = float(raw_bp.get("E")),
        I = float(raw_bp.get("I")),
        A = float(raw_bp.get("A")),
        h = float(raw_bp.get("h")))

def parse_line(raw_line: dict) -> Line:
    if not isinstance(raw_line, dict):
        raise ValueError("Each line must be an object.")
    return Line(
        id = str(raw_line["id"]),
        a = str(raw_line["a"]),
        b = str(raw_line["b"]),
        bp = parse_beam_props(raw_line.get("bp")),
        ul = parse_uniform_load(raw_line.get("ul")) if "ul" in raw_line and raw_line.get("ul") is not None else None,
        ptls = parse_list(raw_line.get("ptls", []), parse_point_load_on_line, "ptls"))

def parse_txt_data(txt: str) -> Tuple[str, Scene]:
    if not txt: return "Text box is empty.", None

    try: data = json.loads(txt)
    except Exception as e: return f"Invalid JSON:\n{e}", None

    if not isinstance(data, dict): return "Root JSON must be an object.", None

    try: points: List[Point] = parse_list(data.get("points"), parse_point, "points")
    except Exception as e: return f"Invalid points data: {e}", None

    try: supports: List[Support] = parse_list(data.get("supports"), parse_support, "supports")
    except Exception as e: return f"Invalid supports data: {e}", None

    try: lines: List[Line] = parse_list(data.get("lines"), parse_line, "lines")
    except Exception as e: return f"Invalid lines data: {e}", None

    return "", Scene(points, lines, supports)