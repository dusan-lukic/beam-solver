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

SAMPLE_DATA = """
{
    "points": [
        {"id": "P1", "x": 0, "y": 0, "loads": [] },
        {"id": "P2", "x": 6000, "y": 0, "loads": [] },
        {"id": "P3", "x": 3000, "y": 1000, "loads": [] }
    ],
    "supports": [
        { "anch": "P1", "ver": true, "hor": true, "rot": false },
        { "anch": "P2", "ver": true, "hor": false, "rot": false }
    ],
    "lines": [
        { "id": "L1", "a": "P1", "b": "P2", "bp": { "A": 1, "E": 1, "I": 1 }, "ptls": [] },
        { "id": "L2", "a": "P1", "b": "P3", "bp": { "A": 1, "E": 1, "I": 1 }, "ptls": [] },
        { "id": "L3", "a": "P2", "b": "P3", "bp": { "A": 1, "E": 1, "I": 1 }, "ptls": [] }
    ]
}
"""

def parse_txt_data(txt: str) -> Tuple[str, Scene]:
    #TODO: don't be so permisive for missing props, throw instead
    if not txt: return "Text box is empty.", None

    try:
        data = json.loads(txt)
    except Exception as e:
        return f"Invalid JSON:\n{e}", None

    if not isinstance(data, dict): return "Root JSON must be an object.", None

    raw_points = data.get("points")
    raw_supports = data.get("supports")
    raw_lines = data.get("lines")
    if not isinstance(raw_points, list) or not isinstance(raw_supports, list) or not isinstance(raw_lines, list):
        return "Root object must contain 'points', 'supports' and 'lines' lists.", None

    # Parse points
    points: List[Point] = []
    try:
        for rp in raw_points:
            if not isinstance(rp, dict):
                raise ValueError("Each point must be an object.")
            pid = str(rp["id"])
            x = float(rp["x"])
            y = float(rp["y"])
            raw_loads = rp.get("loads", [])
            if not isinstance(raw_loads, list):
                raise ValueError("'loads' must be a list")
            loads: List[PointLoad] = []
            for rl in raw_loads:
                if not isinstance(rl, dict):
                    raise ValueError("Each load must be an object")
                px = float(rl.get("Px", rl.get("px", 0.0)))
                py = float(rl.get("Py", rl.get("py", 0.0)))
                loads.append(PointLoad(Px=px, Py=py))
            points.append(Point(id=pid, x=x, y=y, loads=loads))
    except KeyError as ke:
        return f"Point missing required property: {ke}", None
    except Exception as e:
        return f"Invalid point data: {e}", None

    # Parse supports
    supports: List[Support] = []
    try:
        for rs in raw_supports:
            if not isinstance(rs, dict):
                raise ValueError("Each support must be an object.")
            ver = bool(rs.get("ver", False))
            hor = bool(rs.get("hor", False))
            rot = bool(rs.get("rot", False))
            anch = str(rs.get("anch", ""))
            supports.append(Support(ver=ver, hor=hor, rot=rot, anch=anch))
    except Exception as e:
        return f"Invalid support data: {e}", None

    # Parse lines
    lines: List[Line] = []
    try:
        for rl in raw_lines:
            if not isinstance(rl, dict):
                raise ValueError("Each line must be an object.")
            lid = str(rl["id"])
            a = str(rl["a"])
            b = str(rl["b"])

            # Beam properties (support both uppercase and lowercase keys)
            bp_raw = rl.get("bp", {}) or {}
            E = float(bp_raw.get("E", bp_raw.get("e", 1.0)))
            I = float(bp_raw.get("I", bp_raw.get("i", 1.0)))
            A = float(bp_raw.get("A", bp_raw.get("a", 1.0)))
            bp = Line.BeamProps(E=E, I=I, A=A)

            # Uniform load on member (optional)
            ul_obj = None
            if "ul" in rl and rl.get("ul") is not None:
                ul_raw = rl.get("ul")
                if not isinstance(ul_raw, dict):
                    raise ValueError("'ul' must be an object")
                qx = float(ul_raw.get("qx", ul_raw.get("Qx", ul_raw.get("qx", 0.0))))
                qy = float(ul_raw.get("qy", ul_raw.get("Qy", ul_raw.get("qy", 0.0))))
                ul_obj = UniformLoad(qx=qx, qy=qy)

            # Point loads on the line
            ptls_list: List[Line.PointLoadOnLine] = []
            raw_ptls = rl.get("ptls", []) or []
            if not isinstance(raw_ptls, list):
                raise ValueError("'ptls' must be a list")
            for rpl in raw_ptls:
                if not isinstance(rpl, dict):
                    raise ValueError("Each entry in 'ptls' must be an object")
                c = float(rpl["c"]) if "c" in rpl else float(rpl.get("C", 0.0))
                p_raw = rpl.get("ptl") or rpl.get("ptl", {})
                if not isinstance(p_raw, dict):
                    raise ValueError("'ptl' must be an object")
                px = float(p_raw.get("Px", p_raw.get("px", 0.0)))
                py = float(p_raw.get("Py", p_raw.get("py", 0.0)))
                p_obj = PointLoad(Px=px, Py=py)
                ptls_list.append(Line.PointLoadOnLine(c=c, ptl=p_obj))

            lines.append(Line(id=lid, a=a, b=b, bp=bp, ul=ul_obj, ptls=ptls_list))
    except KeyError as ke:
        return f"Line missing required property: {ke}", None
    except Exception as e:
        return f"Invalid line data: {e}", None
    
    return "", Scene(points, lines, supports)