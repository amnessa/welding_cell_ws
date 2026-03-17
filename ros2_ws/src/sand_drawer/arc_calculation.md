
### 1. Why are the arcs fragmented? (The Boundary Clipping Problem)
**What is happening:** When you watched the robot execute the sweep, it drew a large arc by chaining together several smaller arcs, likely lifting the pen or stuttering in between.
**Why it happens:** In our previous `_generate_sweep_3d` logic, we strictly enforced that *every single point* must lie inside the mathematical rectangle of the table. If a circular arc crossed the corner of the table, went "out of bounds" by 1 millimeter, and came back in, the algorithm chopped it into two separate arcs.
**What the books say:** In the *Springer Handbook of Robotics* (Coverage Path Planning), this is a known flaw of naive geometric clipping. The literature suggests that for tool-based coverage (like milling, painting, or sweeping), **overshoot is vastly superior to fragmentation**. It is physically faster and smoother for the robot to sweep a continuous arc that slightly overhangs the edge of the table (sweeping air for a few centimeters) than it is to stop, lift the tool, move, and drop it back down.
**The Fix:** We need to relax the strict rectangle clipping. If an arc intersects the table, we should let the robot sweep the *entire continuous segment* from the left edge to the right edge in one fluid stroke, even if the middle of the arc bows slightly off the table.

2. **Continuous Arcs:** Update the arc generator to find the extreme left and right angular limits of the table for a given radius, and just draw one single, unbroken arc between them.
