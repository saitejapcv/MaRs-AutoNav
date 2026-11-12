#!/usr/bin/env python3
import math
import numpy as np
import os

def generate_cleated_wheel(output_path, radius=0.15, width=0.11, num_cleats=16, cleat_height=0.015):
    """
    Generates a 3D cylindrical sand wheel with chevron / paddle cleats
    along the perimeter, matching the physical rover's wheels.
    """
    verts = []
    faces = []
    
    # Outer cylinder with cleats
    n_segments = num_cleats * 4
    half_w = width / 2.0
    
    # Generate rim vertices for z = -half_w and z = +half_w
    # Axis of wheel along Z (standard cylinder orientation in URDF before mounting)
    for seg in range(n_segments):
        angle = 2.0 * math.pi * seg / n_segments
        # Modulation for paddle cleats
        cleat_phase = (seg % 4)
        if cleat_phase == 1 or cleat_phase == 2:
            r = radius + cleat_height
        else:
            r = radius
            
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        verts.append([x, y, -half_w]) # front rim
        verts.append([x, y,  half_w]) # back rim
        
    # Central hub vertices (inner circles)
    hub_r = radius * 0.45
    hub_start = len(verts)
    for seg in range(n_segments):
        angle = 2.0 * math.pi * seg / n_segments
        x = hub_r * math.cos(angle)
        y = hub_r * math.sin(angle)
        verts.append([x, y, -half_w])
        verts.append([x, y,  half_w])
        
    # Faces for outer tread surface
    for seg in range(n_segments):
        next_seg = (seg + 1) % n_segments
        v0 = seg * 2 + 1
        v1 = seg * 2 + 2
        v2 = next_seg * 2 + 1
        v3 = next_seg * 2 + 2
        faces.append([v0, v2, v3])
        faces.append([v0, v3, v1])
        
    # Faces for sidewalls (front and back annular plates)
    for seg in range(n_segments):
        next_seg = (seg + 1) % n_segments
        # Front sidewall (z = -half_w)
        outer_f1 = seg * 2 + 1
        outer_f2 = next_seg * 2 + 1
        inner_f1 = hub_start + seg * 2 + 1
        inner_f2 = hub_start + next_seg * 2 + 1
        faces.append([outer_f1, inner_f1, inner_f2])
        faces.append([outer_f1, inner_f2, outer_f2])
        
        # Back sidewall (z = +half_w)
        outer_b1 = seg * 2 + 2
        outer_b2 = next_seg * 2 + 2
        inner_b1 = hub_start + seg * 2 + 2
        inner_b2 = hub_start + next_seg * 2 + 2
        faces.append([outer_b1, outer_b2, inner_b2])
        faces.append([outer_b1, inner_b2, inner_b1])
        
    # Center caps for hub
    front_center = len(verts) + 1
    verts.append([0.0, 0.0, -half_w])
    back_center = len(verts) + 1
    verts.append([0.0, 0.0,  half_w])
    
    for seg in range(n_segments):
        next_seg = (seg + 1) % n_segments
        # Front hub cap
        f1 = hub_start + seg * 2 + 1
        f2 = hub_start + next_seg * 2 + 1
        faces.append([front_center, f2, f1])
        # Back hub cap
        b1 = hub_start + seg * 2 + 2
        b2 = hub_start + next_seg * 2 + 2
        faces.append([back_center, b1, b2])

    # Compute normals for wheel vertices
    normals = []
    for v in verts:
        # Radial outward normal for outer tread, axial for end caps
        r_xy = math.sqrt(v[0]**2 + v[1]**2)
        if r_xy > 0.01:
            normals.append([v[0] / r_xy, v[1] / r_xy, 0.0])
        else:
            # Axis normal for hub centers
            normals.append([0.0, 0.0, 1.0 if v[2] > 0 else -1.0])

    with open(output_path, 'w') as f:
        f.write("# Cleated Sand Wheel Mesh\n")
        f.write("o CleatedWheel\n")
        for v in verts:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        for vn in normals:
            f.write(f"vn {vn[0]:.4f} {vn[1]:.4f} {vn[2]:.4f}\n")
        for fc in faces:
            f.write(f"f {fc[0]}//{fc[0]} {fc[1]}//{fc[1]} {fc[2]}//{fc[2]}\n")
            
    print(f"Cleated wheel written to {output_path} ({len(verts)} verts, {len(normals)} normals, {len(faces)} faces)")

if __name__ == '__main__':
    out = "/home/yashvant/Desktop/MaRs-Caterpillar/src/description/meshes/cleated_wheel.obj"
    generate_cleated_wheel(out)
