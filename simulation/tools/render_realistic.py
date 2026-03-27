#!/usr/bin/env python3
"""Render 3D world image with pyrender — loads .dae meshes from Fuel cache + heightmap"""

import os
os.environ['PYOPENGL_PLATFORM'] = 'egl'

import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import trimesh.transformations as tf
import pyrender
from PIL import Image


# paths and params
SDF_PATH = '/workspace/simulation/src/ugv_gazebo/worlds/outdoor_terrain.sdf'
HEIGHTMAP_PATH = '/workspace/simulation/src/ugv_gazebo/worlds/heightmap.png'
OUTPUT_PATH = '/workspace/simulation/docs/images/world_realistic.png'
FUEL_BASE = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models')

# heightmap params — auto-parsed from SDF
TERRAIN_SIZE_X = 100.0
TERRAIN_SIZE_Y = 100.0
TERRAIN_HEIGHT = 5.0
HMAP_Z_OFFSET = None  # auto-detected from SDF

# model URI → local mesh mapping
MODEL_MESH_MAP = {
    'oak tree':            'oak tree/7/meshes/oak_tree.dae',
    'pine tree':           'pine tree/6/meshes/pine_tree.dae',
    'falling rock 1':      'falling rock 1/6/meshes/FallingRock01.dae',
    'construction barrel': 'construction barrel/4/meshes/construction_barrel.dae',
    'construction cone':   'construction cone/3/meshes/construction_cone.dae',
    'house 1':             'house 1/3/meshes/house_1.dae',
    'house 2':             'house 2/3/meshes/house_2.dae',
    'house 3':             'house 3/3/meshes/house_3.dae',
    'collapsed house':     'collapsed house/4/meshes/collapsed_house.dae',
    'lake house':          'lake house/3/meshes/house.dae',
    'depot':               'depot/6/meshes/Depot.dae',
    'lamp post':           'lamp post/3/meshes/lamp_post.dae',
    'dumpster':            'dumpster/3/meshes/dumpster.dae',
    'fire hydrant':        'fire hydrant/3/meshes/fire_hydrant.dae',
    'urban wall debris':   'urban wall debris/3/meshes/walldebris.dae',
}

# target heights for scaling (meters)
MODEL_TARGET_HEIGHT = {
    'oak tree':            7.0,
    'pine tree':           5.0,
    'falling rock 1':      1.2,
    'construction barrel': 0.9,
    'construction cone':   0.7,
    'house 1':             6.0,
    'house 2':             6.0,
    'house 3':             5.5,
    'collapsed house':     4.0,
    'lake house':          5.0,
    'depot':               4.0,
    'lamp post':           4.5,
    'dumpster':            1.5,
    'fire hydrant':        0.8,
    'urban wall debris':   2.0,
}

# fallback colors when mesh has no material (RGBA)
MODEL_COLORS = {
    'oak tree':            [0.22, 0.45, 0.12, 1.0],
    'pine tree':           [0.12, 0.38, 0.15, 1.0],
    'falling rock 1':      [0.50, 0.45, 0.40, 1.0],
    'construction barrel': [1.00, 0.50, 0.00, 1.0],
    'construction cone':   [1.00, 0.40, 0.00, 1.0],
    'house 1':             [0.82, 0.76, 0.66, 1.0],
    'house 2':             [0.78, 0.72, 0.62, 1.0],
    'house 3':             [0.85, 0.80, 0.70, 1.0],
    'collapsed house':     [0.55, 0.50, 0.45, 1.0],
    'lake house':          [0.70, 0.65, 0.55, 1.0],
    'depot':               [0.60, 0.55, 0.45, 1.0],
    'lamp post':           [0.30, 0.30, 0.30, 1.0],
    'dumpster':            [0.35, 0.50, 0.30, 1.0],
    'fire hydrant':        [0.85, 0.15, 0.10, 1.0],
    'urban wall debris':   [0.60, 0.55, 0.50, 1.0],
}


def parse_sdf(sdf_path):
    """Parse SDF for model placements, buildings, and heightmap z_offset"""
    global HMAP_Z_OFFSET, TERRAIN_HEIGHT
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # auto-detect z_offset and terrain height
    for hm in root.iter('heightmap'):
        pos_el = hm.find('pos')
        size_el = hm.find('size')
        if pos_el is not None:
            vals = pos_el.text.strip().split()
            HMAP_Z_OFFSET = float(vals[2])
        if size_el is not None:
            vals = size_el.text.strip().split()
            TERRAIN_HEIGHT = float(vals[2])
        break
    if HMAP_Z_OFFSET is None:
        HMAP_Z_OFFSET = -1.79
    print(f"  z_offset={HMAP_Z_OFFSET:.2f}, terrain_height={TERRAIN_HEIGHT}")

    # fuel models
    models = []
    for inc in root.iter('include'):
        uri_el = inc.find('uri')
        pose_el = inc.find('pose')
        if uri_el is None or pose_el is None:
            continue
        uri = uri_el.text.strip()
        model_name = uri.rstrip('/').split('/')[-1].lower()
        pose_vals = [float(v) for v in pose_el.text.strip().split()]
        x, y, z, roll, pitch, yaw = pose_vals
        models.append((model_name, x, y, z, roll, pitch, yaw))

    # inline buildings — box geometry in SDF
    buildings = []
    for model in root.iter('model'):
        name = model.get('name', '')
        if name in ('ground_plane', 'terrain'):
            continue
        # if it's got box geometry, it's a building
        box_el = model.find('.//visual/geometry/box')
        if box_el is None:
            continue
        pose_el = model.find('pose')
        if pose_el is None:
            continue
        pose_vals = [float(v) for v in pose_el.text.strip().split()]
        size_el = box_el.find('size')
        if size_el is None:
            continue
        size_vals = [float(v) for v in size_el.text.strip().split()]
        # grab wall color if available
        mat_el = model.find('.//visual/material/diffuse')
        color = [0.7, 0.65, 0.55, 1.0]
        if mat_el is not None:
            cv = [float(v) for v in mat_el.text.strip().split()]
            color = cv[:4] if len(cv) >= 4 else cv + [1.0]
        buildings.append({
            'name': name,
            'pose': pose_vals,
            'size': size_vals,
            'color': color,
        })

    return models, buildings


def create_terrain_mesh(heightmap_path):
    """Trimesh from 16-bit heightmap PNG with vertex colors"""
    img = Image.open(heightmap_path)
    hmap = np.array(img, dtype=np.float64)
    hmap = hmap / 65535.0
    hmap = hmap * TERRAIN_HEIGHT

    rows, cols = hmap.shape  # 257x257
    xs = np.linspace(-TERRAIN_SIZE_X / 2, TERRAIN_SIZE_X / 2, cols)
    ys = np.linspace(-TERRAIN_SIZE_Y / 2, TERRAIN_SIZE_Y / 2, rows)
    xx, yy = np.meshgrid(xs, ys)
    zz = hmap + HMAP_Z_OFFSET

    vertices = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=-1).astype(np.float32)

    # faces — vectorized indexing
    r_idx = np.arange(rows - 1)
    c_idx = np.arange(cols - 1)
    rr, cc = np.meshgrid(r_idx, c_idx, indexing='ij')
    i = (rr * cols + cc).ravel()
    # two tris per quad (CCW winding)
    faces1 = np.stack([i, i + 1, i + cols], axis=-1)
    faces2 = np.stack([i + 1, i + cols + 1, i + cols], axis=-1)
    faces = np.concatenate([faces1, faces2], axis=0).astype(np.uint32)

    # vertex colors — green (low) → brown (high)
    z_norm = (zz.ravel() - HMAP_Z_OFFSET) / TERRAIN_HEIGHT  # [0..1]
    rv = (60 + 80 * z_norm).astype(np.uint8)
    gv = (140 - 20 * z_norm).astype(np.uint8)
    bv = (40 + 20 * z_norm).astype(np.uint8)
    av = np.full_like(rv, 255)
    vertex_colors = np.stack([rv, gv, bv, av], axis=-1)

    mesh = trimesh.Trimesh(
        vertices=vertices, faces=faces,
        vertex_colors=vertex_colors, process=False
    )
    return mesh


def load_dae_model(model_key):
    """Load .dae from Fuel cache, scale to target height"""
    rel_path = MODEL_MESH_MAP.get(model_key)
    if rel_path is None:
        print(f"  [WARN] No mesh mapping for '{model_key}'")
        return None
    full_path = os.path.join(FUEL_BASE, rel_path)
    if not os.path.exists(full_path):
        print(f"  [WARN] Mesh not found: {full_path}")
        return None

    try:
        scene_or_mesh = trimesh.load(full_path, force='scene')
        if isinstance(scene_or_mesh, trimesh.Scene):
            meshes = []
            for geom_name, geom in scene_or_mesh.geometry.items():
                if isinstance(geom, trimesh.Trimesh):
                    # get node transform for this geom
                    try:
                        node_name = None
                        for node in scene_or_mesh.graph.nodes:
                            try:
                                t, g = scene_or_mesh.graph.get(node)
                                if g == geom_name:
                                    node_name = node
                                    geom = geom.copy()
                                    geom.apply_transform(t)
                                    break
                            except Exception:
                                continue
                    except Exception:
                        pass
                    meshes.append(geom)
            if meshes:
                combined = trimesh.util.concatenate(meshes)
            else:
                return None
        elif isinstance(scene_or_mesh, trimesh.Trimesh):
            combined = scene_or_mesh
        else:
            return None
    except Exception as e:
        print(f"  [WARN] Failed to load {full_path}: {e}")
        # fallback — just use a box
        box = trimesh.creation.box(extents=[0.5, 0.5, 1.0])
        return box

    # scale to match target height
    current_height = combined.extents[2]
    target_height = MODEL_TARGET_HEIGHT.get(model_key, 1.0)
    if current_height > 0:
        scale = target_height / current_height
        combined.apply_scale(scale)
        print(f"    Scaled {model_key}: {current_height:.2f} -> {target_height:.2f}m (factor={scale:.4f})")

    # center base at z=0 so it sits on ground
    bounds = combined.bounds
    combined.apply_translation([0, 0, -bounds[0, 2]])

    return combined


def make_rotation_matrix(roll, pitch, yaw):
    """4x4 rotation from RPY"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R = Rz @ Ry @ Rx
    mat = np.eye(4)
    mat[:3, :3] = R
    return mat


def make_pyrender_mesh_from_trimesh(tri_mesh, color):
    """Trimesh → pyrender Mesh with solid color"""
    material = pyrender.MetallicRoughnessMaterial(
        baseColorFactor=color,
        metallicFactor=0.0,
        roughnessFactor=0.9,
    )
    # strip texture visual — avoids GL issues
    tri_mesh.visual = trimesh.visual.ColorVisuals(mesh=tri_mesh)
    return pyrender.Mesh.from_trimesh(tri_mesh, material=material, smooth=True)


def make_terrain_pyrender_mesh(tri_mesh):
    """Terrain trimesh → pyrender Mesh (vertex colors via Primitive)"""
    positions = tri_mesh.vertices.astype(np.float32)
    normals = tri_mesh.vertex_normals.astype(np.float32)
    indices = tri_mesh.faces.astype(np.uint32)
    colors = tri_mesh.visual.vertex_colors  # (N, 4) uint8

    # pyrender wants float colors
    colors_f = colors.astype(np.float32) / 255.0

    material = pyrender.MetallicRoughnessMaterial(
        baseColorFactor=[1.0, 1.0, 1.0, 1.0],
        metallicFactor=0.0,
        roughnessFactor=1.0,
    )
    primitive = pyrender.Primitive(
        positions=positions,
        normals=normals,
        indices=indices,
        color_0=colors_f,
        material=material,
    )
    return pyrender.Mesh(primitives=[primitive])


def look_at(eye, target, up=None):
    """4x4 camera pose (OpenGL convention: -Z forward, +Y up)"""
    if up is None:
        up = np.array([0.0, 0.0, 1.0])
    eye = np.array(eye, dtype=np.float64)
    target = np.array(target, dtype=np.float64)
    up = np.array(up, dtype=np.float64)

    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    mat = np.eye(4)
    mat[:3, 0] = right
    mat[:3, 1] = cam_up
    mat[:3, 2] = -forward
    mat[:3, 3] = eye
    return mat


def align_direction(target_dir):
    """4x4 rotation aligning local -Z to target_dir (for lights)"""
    target_dir = np.array(target_dir, dtype=np.float64)
    target_dir /= np.linalg.norm(target_dir)
    neg_z = np.array([0.0, 0.0, -1.0])
    v = np.cross(neg_z, target_dir)
    c = np.dot(neg_z, target_dir)
    mat = np.eye(4)
    if np.linalg.norm(v) > 1e-6:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
        mat[:3, :3] = R
    return mat


def main():
    print("=== Realistic World Renderer ===")

    # parse SDF for models
    print("Parsing SDF...")
    models, buildings = parse_sdf(SDF_PATH)
    print(f"  Found {len(models)} Fuel models, {len(buildings)} buildings")

    # build terrain mesh
    print("Creating terrain mesh from heightmap...")
    terrain_mesh = create_terrain_mesh(HEIGHTMAP_PATH)
    print(f"  Terrain: {len(terrain_mesh.vertices)} vertices, {len(terrain_mesh.faces)} faces")
    print(f"  Terrain Z range: {terrain_mesh.vertices[:, 2].min():.2f} to {terrain_mesh.vertices[:, 2].max():.2f}")

    # build pyrender scene
    print("Building pyrender scene...")
    scene = pyrender.Scene(
        ambient_light=np.array([0.35, 0.35, 0.38]),
        bg_color=np.array([0.53, 0.74, 0.95, 1.0]),  # sky blue
    )

    # terrain goes in first
    terrain_pr = make_terrain_pyrender_mesh(terrain_mesh)
    scene.add(terrain_pr)

    # load + place models
    model_cache = {}
    for model_key, x, y, z, roll, pitch, yaw in models:
        if model_key not in model_cache:
            print(f"  Loading mesh: {model_key}...")
            model_cache[model_key] = load_dae_model(model_key)

        tri_mesh = model_cache[model_key]
        if tri_mesh is None:
            continue

        mesh_copy = tri_mesh.copy()

        # rotation + translation transform
        transform = make_rotation_matrix(roll, pitch, yaw)
        transform[:3, 3] = [x, y, z]

        color = MODEL_COLORS.get(model_key, [0.6, 0.6, 0.6, 1.0])
        pr_mesh = make_pyrender_mesh_from_trimesh(mesh_copy, color)
        scene.add(pr_mesh, pose=transform)

    # buildings — simple colored boxes
    for bld in buildings:
        x, y, z, roll, pitch, yaw = bld['pose']
        w, d, h = bld['size']
        box = trimesh.creation.box(extents=[w, d, h])
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=bld['color'], metallicFactor=0.0, roughnessFactor=0.8)
        box.visual = trimesh.visual.ColorVisuals(mesh=box)
        pr_box = pyrender.Mesh.from_trimesh(box, material=material)
        t = make_rotation_matrix(roll, pitch, yaw)
        t[:3, 3] = [x, y, z]
        scene.add(pr_box, pose=t)

    print(f"  Scene has {len(scene.mesh_nodes)} mesh nodes")

    # lights
    # sun
    sun = pyrender.DirectionalLight(
        color=np.array([1.0, 0.95, 0.85]),
        intensity=5.0,
    )
    sun_pose = align_direction([-0.5, 0.2, -0.9])
    sun_pose[:3, 3] = [0, 0, 50]
    scene.add(sun, pose=sun_pose)

    # fill light
    fill = pyrender.DirectionalLight(
        color=np.array([0.7, 0.7, 0.85]),
        intensity=2.0,
    )
    fill_pose = align_direction([0.5, -0.3, -0.5])
    fill_pose[:3, 3] = [0, 0, 50]
    scene.add(fill, pose=fill_pose)

    # rim light for some depth separation
    back = pyrender.DirectionalLight(
        color=np.array([0.9, 0.85, 0.7]),
        intensity=1.5,
    )
    back_pose = align_direction([0.0, 0.8, -0.3])
    back_pose[:3, 3] = [0, 0, 50]
    scene.add(back, pose=back_pose)

    # camera
    camera = pyrender.PerspectiveCamera(
        yfov=np.pi / 4.5,
        aspectRatio=1920.0 / 1080.0,
        znear=0.5,
        zfar=500.0,
    )
    # render from multiple angles
    renderer = pyrender.OffscreenRenderer(1920, 1080)

    views = [
        # (name, eye, target)
        ("world_realistic.png",       [70, -70, 50],  [0, 0, 0]),       # overview
        ("world_forest.png",          [-25, -25, 15], [-40, 0, 0]),     # forest / spawn
        ("world_village.png",         [55, -20, 12],  [33, 0, 1]),      # village
        ("world_trail.png",           [10, -30, 20],  [-5, 0, 0]),      # trail mid
    ]

    for fname, eye, target in views:
        cam = pyrender.PerspectiveCamera(yfov=np.pi/4.5, aspectRatio=1920/1080, znear=0.5, zfar=500)
        cam_pose = look_at(eye, target)
        cam_node = scene.add(cam, pose=cam_pose)
        print(f"Rendering {fname}...")
        color_img, _ = renderer.render(scene)
        scene.remove_node(cam_node)
        out = os.path.join(os.path.dirname(OUTPUT_PATH), fname)
        Image.fromarray(color_img).save(out, optimize=True)
        sz = os.path.getsize(out)
        print(f"  Saved {out} ({sz//1024} KB)")

    renderer.delete()
    print("Done!")


if __name__ == '__main__':
    main()
