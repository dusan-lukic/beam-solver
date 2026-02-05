#!/usr/bin/env python3

# LLM-generated
# human reviewed

import math
import tkinter as tk
from tkinter import Scrollbar, messagebox
from typing import List
from float_to_sig import float_to_str_sig
from fundamental_data import *
from frame_solver import solve_scene


# ------------------------
# Utils
# ------------------------

CANVAS_MARGIN_SIZE = 30
POINT_DIAMETER = 8
POINT_RADIUS = POINT_DIAMETER / 2.0

def bounding_box(points: List[Point]):
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    return minx, miny, maxx, maxy


# ------------------------
# GUI
# ------------------------

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

    def canvas_to_world(self, x: float, y: float):
        wx = (x - self.world_origin_x_in_canvas_coordinates) * self.world_over_canvas_size_ratio
        wy = (self.world_origin_y_in_canvas_coordinates - y) * self.world_over_canvas_size_ratio
        return wx, wy

    def world_to_canvas(self, wx: float, wy: float):
        x = wx / self.world_over_canvas_size_ratio + self.world_origin_x_in_canvas_coordinates
        y = self.world_origin_y_in_canvas_coordinates - wy / self.world_over_canvas_size_ratio
        return x, y

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

    def draw_solution(self, solution: FrameSolution):
        # Helper to compute color from s_max: 0 -> green, 1e8+ -> red
        def _color_from_smax(s_max: float) -> str:
            t = max(0.0, min(1.0, s_max / (10**8)))
            r = int(255 * t)
            g = int(255 * (1.0 - t))
            return f"#{r:02x}{g:02x}00"

        for line_id, (lstress, _) in solution.line_stresses_and_strains.items():
            self.canvas.itemconfigure(line_id, fill = _color_from_smax(lstress.s_max))

    def parse_and_validate_data(self) -> Scene | None:
        (err, scene) = parse_txt_data(self.text.get("1.0", tk.END).strip())
        if err:
            messagebox.showerror("Data error", err)
            return None
        return scene

    def visualize_data(self, e):
        scene: Scene | None = self.parse_and_validate_data()
        if scene:
            self.update_canvas_transform_to_fit(scene.points)
            self.draw_scene(scene.points, scene.lines, scene.supports)

    def export_system(self, e):
        scene: Scene | None = self.parse_and_validate_data()
        if scene:
            solution: FrameSolution = solve_scene(scene)
            self.draw_solution(solution)


if __name__ == "__main__":
    app = SimpleApp()
    app.mainloop()
    