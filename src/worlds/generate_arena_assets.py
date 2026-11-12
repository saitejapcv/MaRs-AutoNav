#!/usr/bin/env python3
import os
import math
import numpy as np
import cv2
import cv2.aruco as aruco

def generate_aruco_markers(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    for marker_id in range(4):
        # Generate 400x400 marker image
        marker_img = aruco.generateImageMarker(dictionary, marker_id, 400)
        # Add a 50px white border for high contrast
        bordered = cv2.copyMakeBorder(marker_img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
        path = os.path.join(output_dir, f"aruco_tag_{marker_id}.png")
        cv2.imwrite(path, bordered)
        print(f"Generated ArUco marker ID {marker_id} -> {path}")

def generate_terrain_assets(models_dir, tex_dir):
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(tex_dir, exist_ok=True)
    
    obj_path = os.path.join(models_dir, "cat_terrain.obj")
    mtl_path = os.path.join(models_dir, "cat_terrain.mtl")
    tex_path = os.path.join(tex_dir, "cat_sand_diffuse.png")
    
    print("Generating arena terrain mesh with deep, wide crater depressions...")
    # Arena dimensions in meters
    Lx = 9.0
    Ly = 5.0
    # Fine grid resolution: 2.0 cm grid
    res = 0.020
    x = np.arange(0, Lx + res, res)
    y = np.arange(0, Ly + res, res)
    nx = len(x)
    ny = len(y)
    
    X, Y = np.meshgrid(x, y) # shape: (ny, nx)
    Z = np.zeros_like(X)

    # Defined deep craters: (xc, yc, radius, depth)
    # Craters sized between 1.0m and 1.2m diameter (wider than the 0.76m rover track width!)
    # and 30-35 cm deep so the rover CANNOT straddle them and MUST sink in!
    # Crater 1 is placed at (3.2, 3.8) directly in front of the starting zone heading (Y=4.0)!
    craters = [
        (3.2, 3.8, 0.55, 0.30),  # 1.10m diameter, 30cm depth (Direct path from start zone)
        (5.2, 3.1, 0.60, 0.35),  # 1.20m diameter, 35cm depth (Large central hazard crater)
        (3.5, 1.6, 0.50, 0.30),  # 1.00m diameter, 30cm depth (South traverse crater)
        (6.6, 2.0, 0.55, 0.32)   # 1.10m diameter, 32cm depth (Excavation approach crater)
    ]

    for xc, yc, R, depth in craters:
        dist_sq = (X - xc)**2 + (Y - yc)**2
        mask = dist_sq < (R**2)
        r = np.sqrt(dist_sq[mask])
        # Power cosine bowl profile: drops steeply from rim to center
        bowl = -depth * (np.cos((math.pi * r) / (2.0 * R)) ** 0.6)
        Z[mask] += bowl

    # 1. Generate Material File (MTL)
    with open(mtl_path, 'w') as f:
        f.write("# Sand Terrain Material Definition\n")
        f.write("newmtl SandMaterial\n")
        f.write("Ka 0.75 0.65 0.50\n")
        f.write("Kd 0.85 0.72 0.55\n")
        f.write("Ks 0.05 0.05 0.05\n")
        f.write("Ns 10.0\n")
        f.write("map_Kd ../textures/cat_sand_diffuse.png\n")
    print(f"Material written to {mtl_path}")

    # 2. Export as Wavefront OBJ with UV mapping, normals, and material
    with open(obj_path, 'w') as f:
        f.write("# Caterpillar Challenge Sand Arena Terrain\n")
        f.write("mtllib cat_terrain.mtl\n")
        f.write("o SandTerrain\n")
        
        # Vertices
        for j in range(ny):
            for i in range(nx):
                f.write(f"v {X[j, i]:.4f} {Y[j, i]:.4f} {Z[j, i]:.4f}\n")
        
        # UVs
        for j in range(ny):
            for i in range(nx):
                u = X[j, i] / Lx
                v = Y[j, i] / Ly
                f.write(f"vt {u:.4f} {v:.4f}\n")
        
        # Vertex normals
        for j in range(ny):
            for i in range(nx):
                dz_dx = 0.0
                dz_dy = 0.0
                if 0 < i < nx - 1:
                    dz_dx = (Z[j, i+1] - Z[j, i-1]) / (2.0 * res)
                if 0 < j < ny - 1:
                    dz_dy = (Z[j+1, i] - Z[j-1, i]) / (2.0 * res)
                n = np.array([-dz_dx, -dz_dy, 1.0])
                n /= np.linalg.norm(n)
                f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")

        # Faces with Material
        f.write("usemtl SandMaterial\n")
        for j in range(ny - 1):
            for i in range(nx - 1):
                v00 = j * nx + i + 1
                v10 = j * nx + (i + 1) + 1
                v11 = (j + 1) * nx + (i + 1) + 1
                v01 = (j + 1) * nx + i + 1
                f.write(f"f {v00}/{v00}/{v00} {v10}/{v10}/{v10} {v11}/{v11}/{v11}\n")
                f.write(f"f {v00}/{v00}/{v00} {v11}/{v11}/{v11} {v01}/{v01}/{v01}\n")

    print(f"Terrain mesh written to {obj_path} ({nx*ny} vertices, {(nx-1)*(ny-1)*2} faces, max depth: {abs(np.min(Z)):.2f}m)")

    # 3. Generate High-Resolution Sand Diffuse Texture (1800 x 1000 px)
    tex_w = 1800
    tex_h = 1000
    # Natural sand base color (BGR: 125, 175, 215)
    sand_bgr = np.full((tex_h, tex_w, 3), (125, 175, 215), dtype=np.uint8)
    
    # Add fine sand grain noise
    np.random.seed(42)
    grain = np.random.normal(0, 7.0, (tex_h, tex_w, 3)).astype(np.int16)
    sand_bgr = np.clip(sand_bgr.astype(np.int16) + grain, 0, 255).astype(np.uint8)

    # Shade dark crater bowls with distinct visual rim highlights
    y_coords, x_coords = np.ogrid[:tex_h, :tex_w]
    for xc, yc, R, depth in craters:
        # Texture coordinates: X in [0..Lx] -> [0..tex_w], Y in [0..Ly] -> [0..tex_h] (OBJ UV: v=0 at y=0)
        # Note: in image coordinate system, row 0 is y=0 if we map directly, or row 0 is top.
        # UV coordinate has v=Y/Ly, so row j corresponding to v is:
        # In OBJ: vt u v where v=0 is Y=0. In OpenCV image: row 0 is top (Y=Ly).
        # Therefore, py = int((1.0 - yc / Ly) * tex_h)
        px = int((xc / Lx) * tex_w)
        py = int((1.0 - (yc / Ly)) * tex_h)
        pr = int((R / Lx) * tex_w)

        d2 = (x_coords - px)**2 + (y_coords - py)**2
        mask = d2 < (pr**2)
        norm_r = np.sqrt(d2[mask]) / pr
        
        # Deep dark crater center + shadow falloff
        shade_factor = 0.45 + 0.55 * (norm_r ** 0.8)
        for c in range(3):
            sand_bgr[mask, c] = np.clip(sand_bgr[mask, c] * shade_factor, 0, 255).astype(np.uint8)
            
        # Draw subtle highlighted circular rim
        rim_mask = (d2 >= (pr - 4)**2) & (d2 <= (pr + 4)**2)
        sand_bgr[rim_mask] = np.clip(sand_bgr[rim_mask].astype(np.int16) + 30, 0, 255).astype(np.uint8)

    # Paint visual competition zone markings into the texture
    # Starting Zone (2m x 2m at X:[0..2], Y:[3..5]) -> Green border
    x0, x1 = 0, int((2.0 / Lx) * tex_w)
    y0 = int((1.0 - (5.0 / Ly)) * tex_h) # 0
    y1 = int((1.0 - (3.0 / Ly)) * tex_h)
    cv2.rectangle(sand_bgr, (x0 + 4, y0 + 4), (x1, y1), (40, 200, 50), 6) # Green

    # Construction Zone (2m x 1.5m at X:[0..2], Y:[0..1.5]) -> Cyan border
    y_c0 = int((1.0 - (1.5 / Ly)) * tex_h)
    y_c1 = tex_h - 4
    cv2.rectangle(sand_bgr, (x0 + 4, y_c0), (x1, y_c1), (220, 160, 40), 6) # Cyan/Blue

    # Berm Target (1.25m x 0.90m inside construction zone) -> Orange border
    bx0 = int((0.375 / Lx) * tex_w)
    bx1 = int((1.625 / Lx) * tex_w)
    by0 = int((1.0 - (1.20 / Ly)) * tex_h)
    by1 = int((1.0 - (0.30 / Ly)) * tex_h)
    cv2.rectangle(sand_bgr, (bx0, by0), (bx1, by1), (30, 120, 230), 4) # Orange

    # Excavation Zone dividing line at X = 7.5 -> Magenta/White dashed line
    ex_x = int((7.5 / Lx) * tex_w)
    cv2.line(sand_bgr, (ex_x, 0), (ex_x, tex_h), (200, 50, 220), 6)

    cv2.imwrite(tex_path, sand_bgr)
    print(f"Sand terrain diffuse texture written to {tex_path}")

def generate_boulder(output_path, radius=0.175, height_ratio=0.75, h1=0.10, h2=0.08, h3=0.08):
    """
    Generates a realistic, natural rounded rocky boulder with smooth low-frequency 
    surface harmonics, perfectly in scale with the official 30-40cm competition spec.
    """
    n_lat = 18
    n_lon = 26
    
    verts = []
    
    # Top pole vertex
    top_r = radius * (1.0 + h2) * height_ratio
    verts.append([0.0, 0.0, top_r])
    
    for i in range(1, n_lat):
        theta = math.pi * i / n_lat
        for j in range(n_lon):
            phi = 2.0 * math.pi * j / n_lon
            
            harm = (h1 * math.sin(2*theta) * math.cos(phi) + 
                    h2 * math.cos(3*theta) + 
                    h3 * math.sin(2*phi) * math.sin(theta))
            
            r = radius * (1.0 + harm)
            z = r * math.cos(theta) * height_ratio
            if z < -0.06:
                z = max(z, -radius * height_ratio * 0.85)
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            verts.append([x, y, z])
            
    # Bottom pole vertex
    verts.append([0.0, 0.0, -radius * height_ratio * 0.85])
    verts = np.array(verts)

    faces = []
    # Top fan
    for j in range(n_lon):
        p1 = 1
        p2 = 1 + j + 1
        p3 = 1 + ((j + 1) % n_lon) + 1
        faces.append([p1, p2, p3])

    # Middle quad strips
    for i in range(n_lat - 2):
        row1 = 1 + i * n_lon + 1
        row2 = 1 + (i + 1) * n_lon + 1
        for j in range(n_lon):
            next_j = (j + 1) % n_lon
            v00 = row1 + j
            v10 = row1 + next_j
            v01 = row2 + j
            v11 = row2 + next_j
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])

    # Bottom fan
    bottom_idx = len(verts)
    last_row = 1 + (n_lat - 2) * n_lon + 1
    for j in range(n_lon):
        p1 = bottom_idx
        p2 = last_row + ((j + 1) % n_lon)
        p3 = last_row + j
        faces.append([p1, p2, p3])

    # Compute analytical vertex normals
    normals = []
    for i, v in enumerate(verts):
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            normals.append(v / norm)
        else:
            normals.append(np.array([0.0, 0.0, 1.0]))
    normals = np.array(normals)

    dx = np.ptp(verts[:, 0])
    dy = np.ptp(verts[:, 1])
    dz = np.ptp(verts[:, 2])

    with open(output_path, 'w') as f:
        f.write("# Natural Rocky Boulder Mesh\n")
        f.write("o Boulder\n")
        for v in verts:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        for vn in normals:
            f.write(f"vn {vn[0]:.4f} {vn[1]:.4f} {vn[2]:.4f}\n")
        for fc in faces:
            f.write(f"f {fc[0]}//{fc[0]} {fc[1]}//{fc[1]} {fc[2]}//{fc[2]}\n")
            
    print(f"Boulder written to {output_path} | Size: {dx*100:.1f}cm x {dy*100:.1f}cm x {dz*100:.1f}cm height ({len(verts)} verts)")

def main():
    base_dir = "/home/yashvant/Desktop/MaRs-Caterpillar/src/worlds"
    tex_dir = os.path.join(base_dir, "textures")
    models_dir = os.path.join(base_dir, "models")
    
    generate_aruco_markers(tex_dir)
    generate_terrain_assets(models_dir, tex_dir)
    
    # Generate 4 natural boulders matching competition specifications (30-40 cm diameter)
    boulder_configs = [
        ("boulder_1.obj", 0.175, 0.75, 0.11, 0.08, 0.09), # ~36 cm dia
        ("boulder_2.obj", 0.190, 0.70, 0.12, 0.09, 0.07), # ~40 cm dia
        ("boulder_3.obj", 0.160, 0.80, 0.09, 0.07, 0.10), # ~34 cm dia
        ("boulder_4.obj", 0.130, 0.75, 0.10, 0.06, 0.08), # ~27 cm dia (excavation zone)
    ]
    for filename, r, hr, h1, h2, h3 in boulder_configs:
        generate_boulder(os.path.join(models_dir, filename), radius=r, height_ratio=hr, h1=h1, h2=h2, h3=h3)

if __name__ == "__main__":
    main()
