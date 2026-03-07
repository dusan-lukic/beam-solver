# LLM-generated
# human reviewed

"""Custom pop-up dialogs used by the frame viewer.

This module contains placeholder dialog classes for later visualization of
forces at a point and on a line.  The implementations are intentionally
deliberate skeletons: UI layout and data binding will be supplied in future
work when the visualization details are finalized.
"""

import math
import tkinter as tk
from tkinter import simpledialog
from typing import List, Dict


class PointForceDialog(simpledialog.Dialog):
    """Dialog for visualizing forces acting on a point (joint).

    The dialog takes a dictionary mapping a string label to a 3-element list
    `[Fx, Fy, M]` where Fx and Fy are the Cartesian components of a force and
    M is the scalar moment (positive CCW).  The forces are drawn as a set of
    arrows originating from the canvas centre; moments are shown as tangential
    arrows on a reference circle.
    """

    def __init__(self, parent, pid: str, forces: dict[str, List[float]]):
        self.pid = pid
        self.forces = forces
        super().__init__(parent, title=f"Forces at {pid}")

    def body(self, master):
        # summary label
        total_x = sum(v[0] for v in self.forces.values())
        total_y = sum(v[1] for v in self.forces.values())
        total_m = sum(v[2] for v in self.forces.values())
        label = tk.Label(master, text=f"Point: {self.pid}  ΣFx={total_x:.1f}, ΣFy={total_y:.1f}, ΣM={total_m:.1f}")
        label.pack()

        # compute max vector magnitude
        max_mag = max([math.hypot(fx, fy) for (fx, fy, _) in self.forces.values()])
        canvas_size = 800
        margin_size = 100
        R = canvas_size / 2 - margin_size
        scale = (R / 1.1 / max_mag) if max_mag > 0 else 1.0

        # canvas size based on R
        c = tk.Canvas(master, width=canvas_size, height=canvas_size, bg="white")
        c.pack()
        cx = cy = canvas_size / 2
        # draw reference circle
        c.create_oval(cx - R, cy - R, cx + R, cy + R, outline="gray")

        # draw force and momentarrows
        mmax = max([abs(m) for _, (_, _, m) in self.forces.items()])
        for lbl, (fx, fy, m) in self.forces.items():
            mag = math.hypot(fx, fy)
            if mag == 0:
                fx = fy = 1.0
            angle = math.atan2(fy, fx)
            ex = cx + fx * scale
            ey = cy - fy * scale
            c.create_line(cx, cy, ex, ey, arrow=tk.LAST)
            # label near end
            lx = ex + 5 * math.cos(angle)
            ly = ey - 5 * math.sin(angle)
            c.create_text(lx, ly, text=lbl, anchor="nw")

            # moment
            if m == 0: m = 1.0
            px = cx + R * math.cos(angle)
            py = cy - R * math.sin(angle)
            tan_angle = angle + (math.pi/2 if m > 0 else -math.pi/2)
            Lm = (abs(m) / mmax) * R * math.pi/3
            tx = px + Lm * math.cos(tan_angle)
            ty = py - Lm * math.sin(tan_angle)
            c.create_line(px, py, tx, ty, arrow=tk.LAST)

        return master

    def apply(self):
        # Called when the user confirms the dialog
        pass


class LineForceDialog(simpledialog.Dialog):
    """Dialog for visualizing axial/shear forces and moments along a line.

    Attributes:
        line: the Line instance associated with the dialog
        lstress: the LineStress instance for that line
    """

    def __init__(self, parent):
        super().__init__(parent, title="Forces on Line")

    def body(self, master):
        label = tk.Label(master, text="Line: ")
        label.pack()
        # placeholder area
        return master

    def apply(self):
        # Perform any cleanup or data extraction as needed
        pass
