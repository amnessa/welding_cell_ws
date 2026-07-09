If Object B gets within a few centimeters of Object A (which is already fixed to the fixture), Object A's points will enter Object B's CropBox. SciPy's `cKDTree` has no idea what "Object A" or "Object B" is—it only sees points. It will absolutely match Object B's CAD points to Object A's physical surface, and your pose estimation will warp or completely snap to the wrong part.

To solve this, we don't just rely on the Robust ICP math; we use your pipeline's greatest advantage: **You already know the exact pose of the fixed objects.** Here is how you refine your methodology to make your system "blind" to previously assembled parts, using a technique called **Model-Based Background Subtraction (or Digital Twin Masking).**

### The Assembly Tracking Methodology

Instead of just maintaining the state of the *moving* object, your system will now maintain a **Static Environment Point Cloud (SEPC)**.

#### Step 1: Place and Save (The First Object)

1. You track Object A using your current CropBox + Voxel + ICP loop.
2. The robot moves Object A into the final fixture position.
3. You trigger a "Save" command.
4. **The Critical Addition:** You take Object A's CAD point cloud, transform it by its final saved 6D pose, and add those points to your `SEPC` (Static Environment Point Cloud) in memory.
5. You build a new SciPy `cKDTree` specifically for this `SEPC`.

#### Step 2: Track the New Object (The Subtraction Phase)

Now you introduce Object B and initialize it with SAM-6D. Your fast tracking loop for Object B begins, but we add *one* new step right before your Voxel Downsampler:

1. **The CropBox:** Crop the live RealSense cloud around Object B's `current_pose` (e.g., yields ~50k points).
2. **The Masking Check (New):** You query the `SEPC` KD-tree you built in Step 1. For every point in your 50k cropped live points, you ask: *"How close is this point to the assembled Object A?"*
3. **The Subtraction:** If a live point is within a noise threshold (e.g., `distance < 5mm`) of *any* point in the `SEPC`, **delete it**.
4. **The Rest of Your Pipeline:** Take the *remaining* points, Voxel Downsample them down to ~2k, calculate normals, and run your Fast ICP.

### Why This Works Beautifully for Assembly

By doing this, as Object B physically approaches Object A, Object A literally **disappears** from the point cloud that your ICP algorithm sees.

Your ICP `cKDTree` for Object B will never get confused, because the physical points belonging to Object A were erased in Step 3. You can push Object B completely flush against Object A, and the algorithm will only track the geometry of Object B.

When you fix Object B, you simply transform its CAD model, append it to the `SEPC`, rebuild the background KD-tree, and bring in Object C.
