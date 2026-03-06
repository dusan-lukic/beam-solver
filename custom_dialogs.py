# LLM-generated
# human reviewed

"""Custom pop-up dialogs used by the frame viewer.

This module contains placeholder dialog classes for later visualization of
forces at a point and on a line.  The implementations are intentionally
deliberate skeletons: UI layout and data binding will be supplied in future
work when the visualization details are finalized.
"""

import tkinter as tk
from tkinter import simpledialog
from typing import List


class PointForceDialog(simpledialog.Dialog):
    """Dialog for visualizing forces acting on a point (joint).

    Attributes:
        point: the Point instance associated with the dialog
        solution: the FrameSolution object containing computed reactions
    """

    def __init__(self, parent, pid: str, forces: List[List[float]]):
        self.pid = pid
        self.forces = forces
        super().__init__(parent, title="Forces at Point")

    def body(self, master):
        label = tk.Label(master, text=f"Point: {self.pid}. X: {sum(f[0] for f in self.forces):.2f}, Y: {sum(f[1] for f in self.forces):.2f}, M: {sum(f[2] for f in self.forces):.2f}")
        label.pack()

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
