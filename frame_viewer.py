#!/usr/bin/env python3

# LLM-generated
# human reviewed

from dataclasses import dataclass
import json
import math
import tkinter as tk
from tkinter import Scrollbar, messagebox
from typing import List, Optional, Tuple
from float_to_sig import float_to_str_sig

# --- Data structures (for future deserialization) ---
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
    bp: BeamProps
    ul: UniformLoad
    @dataclass
    class PointLoadOnLine:
        c: float
        ptl: PointLoad
    ptls: List[PointLoadOnLine]

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
#TODO: move solver stuff to a separate module
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
        return (-self.line.ul.qx*self.y_length() + self.line.ul.qy*self.x_length()) / self.length()
    
    def q_ax(self) -> float:
        return (self.line.ul.qx*self.x_length() + self.line.ul.qy*self.y_length()) / self.length()

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
    import numpy as np
    
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
            A[eq_idx, hp.sup.var_ptr] = 1
        b[eq_idx] = sum(l.total_load_x() for l in hp.lines_a) + hp.total_load_x()
        eq_idx += 1

        # F_y balance
        for m in hp.lines_b:
            A[eq_idx, m.var_ptr + 1] = 1
        for m in hp.lines_a:
            A[eq_idx, m.var_ptr + 1] = -1
        if (hp.sup and hp.sup.support.ver):
            A[eq_idx, hp.sup.var_ptr + (1 if hp.sup.support.hor else 0)] = 1
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
            A[eq_idx, hp.sup.var_ptr + offset] = 1
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

# --- Data Utils -------------------------------------
    
def bounding_box(points: List[Point]):
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    return minx, miny, maxx, maxy

SAMPLE_DATA = """
{
    "points": [
        {"id": "P1", "x": 0, "y": 0 },
        {"id": "P2", "x": 6000, "y": 0 },
        {"id": "P3", "x": 3000, "y": 1000 }
    ],
    "supports": [
        { "anch": "P1", "ver": true, "hor": true, "rot": false },
        { "anch": "P2", "ver": true, "hor": false, "rot": false }
    ],
    "lines": [
        { "id": "L1", "a": "P1", "b": "P2" },
        { "id": "L2", "a": "P1", "b": "P3" },
        { "id": "L3", "a": "P2", "b": "P3" }
    ]
}
"""
    
# ----------------------------------------------------

CANVAS_MARGIN_SIZE = 30
POINT_DIAMETER = 8
POINT_RADIUS = POINT_DIAMETER / 2.0

class SimpleApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Text + Canvas Layout")

        # Configure grid: row 0 stretches vertically, column 1 (canvas) stretches horizontally
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Left frame with fixed width 200 px for the text widget
        left_frame = tk.Frame(self, width=200)
        left_frame.grid(row=0, column=0, sticky="ns")
        left_frame.grid_propagate(False)  # keep the frame at the specified width

        # Text widget with vertical scrollbar
        self.text = tk.Text(left_frame, wrap="none")
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        vsb = Scrollbar(left_frame, orient=tk.VERTICAL, command=self.text.yview)
        self.text.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)

        # Right frame that expands (contains the canvas)
        right_frame = tk.Frame(self)
        right_frame.grid(row=0, column=1, sticky="nsew")
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_columnconfigure(0, weight=1)

        # Canvas fills the right frame and resizes with the window
        self.canvas = tk.Canvas(right_frame, bg="white")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # Initialize text box with sample JSON
        self.text.insert("1.0", SAMPLE_DATA)

        # initial window size
        self.geometry("1900x700")

        # world to canvas transformation data
        self.world_origin_x_in_canvas_coordinates = CANVAS_MARGIN_SIZE
        self.world_origin_y_in_canvas_coordinates = 600 - CANVAS_MARGIN_SIZE
        self.world_over_canvas_size_ratio: float = 1.0

        # event handlers
        self.bind("<F5>", self.visualize_data)
        self.bind("<F9>", self.export_system)

    # ------------------------
    # Utils
    # ------------------------
    def canvas_to_world(self, x: float, y: float):
        wx = (x - self.world_origin_x_in_canvas_coordinates) * self.world_over_canvas_size_ratio
        wy = (self.world_origin_y_in_canvas_coordinates - y) * self.world_over_canvas_size_ratio
        return wx, wy

    def world_to_canvas(self, wx: float, wy: float):
        x = wx / self.world_over_canvas_size_ratio + self.world_origin_x_in_canvas_coordinates
        y = self.world_origin_y_in_canvas_coordinates - wy / self.world_over_canvas_size_ratio
        return x, y

    def parse_txt_data(self):
        txt = self.text.get("1.0", tk.END).strip()
        if not txt:
            messagebox.showinfo("No data", "Text box is empty.")
            return None

        try:
            data = json.loads(txt)
        except Exception as e:
            messagebox.showerror("JSON error", f"Invalid JSON:\n{e}")
            return None

        if not isinstance(data, dict):
            messagebox.showerror("Format error", "Root JSON must be an object.")
            return None

        raw_points = data.get("points")
        raw_supports = data.get("supports")
        raw_lines = data.get("lines")
        if not isinstance(raw_points, list) or not isinstance(raw_supports, list) or not isinstance(raw_lines, list):
            messagebox.showerror("Format error", "Root object must contain 'points', 'supports' and 'lines' lists.")
            return None

        # Parse points
        points: List[Point] = []
        try:
            for rp in raw_points:
                if not isinstance(rp, dict):
                    raise ValueError("Each point must be an object.")
                pid = str(rp["id"])
                x = float(rp["x"])
                y = float(rp["y"])
                points.append(Point(id=pid, x=x, y=y))
        except KeyError as ke:
            messagebox.showerror("Format error", f"Point missing required property: {ke}")
            return None
        except Exception as e:
            messagebox.showerror("Format error", f"Invalid point data: {e}")
            return None

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
            messagebox.showerror("Format error", f"Invalid support data: {e}")
            return None

        # Parse lines
        lines: List[Line] = []
        try:
            for rl in raw_lines:
                if not isinstance(rl, dict):
                    raise ValueError("Each line must be an object.")
                lid = str(rl["id"])
                a = str(rl["a"])
                b = str(rl["b"])
                lines.append(Line(id=lid, a=a, b=b))
        except KeyError as ke:
            messagebox.showerror("Format error", f"Line missing required property: {ke}")
            return None
        except Exception as e:
            messagebox.showerror("Format error", f"Invalid line data: {e}")
            return None
        
        return points, lines, supports

    def update_canvas_transform_to_fit(self, points: List[Point]):
        self.update_idletasks()
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        
        # If no points, set a reasonable default transform and return empty mapping
        if not points:
            # Keep existing transform but ensure it's reasonable with current canvas size
            self.world_origin_x_in_canvas_coordinates = CANVAS_MARGIN_SIZE
            self.world_origin_y_in_canvas_coordinates = ch - CANVAS_MARGIN_SIZE
            self.world_over_canvas_size_ratio = 1.0
            return

        minx, miny, maxx, maxy = bounding_box(points)

        bbox_w = maxx - minx
        bbox_h = maxy - miny
        # Avoid zero sizes to prevent division by zero
        if bbox_w <= 0:
            bbox_w = 1.0
        if bbox_h <= 0:
            bbox_h = 1.0

        avail_w = max(1.0, cw - 2 * CANVAS_MARGIN_SIZE)
        avail_h = max(1.0, ch - 2 * CANVAS_MARGIN_SIZE)

        # world_over_canvas_size_ratio: world units per canvas pixel
        # We need r such that bbox_w / r <= avail_w  and bbox_h / r <= avail_h
        # => r >= bbox_w / avail_w and r >= bbox_h / avail_h
        r_x = bbox_w / avail_w
        r_y = bbox_h / avail_h
        r = max(r_x, r_y, 1e-9)  # avoid small numbers

        # used pixel sizes after mapping
        used_w_pixels = bbox_w / r
        used_h_pixels = bbox_h / r

        extra_x = (avail_w - used_w_pixels) / 2.0
        extra_y = (avail_h - used_h_pixels) / 2.0

        # Update transform properties
        self.world_over_canvas_size_ratio = r
        self.world_origin_x_in_canvas_coordinates = CANVAS_MARGIN_SIZE + extra_x - (minx / r)
        self.world_origin_y_in_canvas_coordinates = (maxy / r) + CANVAS_MARGIN_SIZE + extra_y

    def draw_scene(self, points: List[Point], lines: List[Line], supports: List[Support]):
        # 1) remove anything already in the canvas
        self.canvas.delete("all")

        # Precompute canvas centers for points using world_to_canvas
        id_to_canvas = {}
        max_x = 0 # needed for label manipulation at the end
        for p in points:
            cx, cy = self.world_to_canvas(p.x, p.y)
            id_to_canvas[p.id] = (cx, cy)
            max_x = max(max_x, cx)

        # 2) Draw lines first so they appear under points
        for ln in lines:
            if ln.a not in id_to_canvas or ln.b not in id_to_canvas:
                messagebox.showinfo("Bad data", "Line connecting to an unknown point.")
                return
            x1, y1 = id_to_canvas[ln.a]
            x2, y2 = id_to_canvas[ln.b]
            self.canvas.create_line(x1, y1, x2, y2, fill="black", width=1)

            # label near middle with small perpendicular offset
            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length == 0:
                offx = 0
                offy = -8
            else:
                offx = -dy / length * 8
                offy = dx / length * 8
            self.canvas.create_text(mx + offx, my + offy, text=ln.id, fill="black")

        # 3) Draw supports (triangles) - under corresponding point circles
        s = 10.0  # side length for triangle
        h = s * math.sqrt(3) / 2.0
        for sup in supports:
            if sup.anch not in id_to_canvas:
                messagebox.showinfo("Bad data", "support under an unknown point.")
                return
            cx, cy = id_to_canvas[sup.anch]

            # top of triangle just below the circle
            top_y = cy + POINT_RADIUS + 2
            left_x = cx - s / 2.0
            right_x = cx + s / 2.0
            bottom_y = top_y + h

            # color rules
            ver = bool(sup.ver)
            hor = bool(sup.hor)
            rot = bool(sup.rot)
            if ver and hor and rot:
                tri_color = "red"
            elif ver and hor and not rot:
                tri_color = "blue"
            elif ver and not hor and not rot:
                tri_color = "cyan"
            else:
                tri_color = "magenta"

            self.canvas.create_polygon(
                left_x, bottom_y, right_x, bottom_y, cx, top_y,
                fill=tri_color, outline=tri_color
            )

        # 4) Draw points (circles) and their labels
        for p in points:
            cx, cy = id_to_canvas[p.id]
            x0 = cx - POINT_RADIUS
            y0 = cy - POINT_RADIUS
            x1 = cx + POINT_RADIUS
            y1 = cy + POINT_RADIUS
            self.canvas.create_oval(x0, y0, x1, y1, fill="skyblue", outline="black", width=1)
            lab = f"{p.id}: {float_to_str_sig(p.x, 3)}, {float_to_str_sig(p.y, 3)}"
            lab_x = cx + POINT_RADIUS + 6 if cx < max_x - 100 else cx + POINT_RADIUS - 100
            lab_y = cy + POINT_RADIUS + 6
            self.canvas.create_text(lab_x, lab_y, text=lab, anchor="w", fill="black")


    def visualize_data(self, e):
        points, lines, supports = self.parse_txt_data()
        self.update_canvas_transform_to_fit(points)
        self.draw_scene(points, lines, supports)

    def export_system(self, e):
        points, lines, supports = self.parse_txt_data()
        hydrated_points, hydrated_lines, hydrated_supports = build_hydrated_structures(points, lines, supports)

        A = []
        b = []

        for p in points:
            A = A + []
            
            



if __name__ == "__main__":
    app = SimpleApp()
    app.mainloop()
