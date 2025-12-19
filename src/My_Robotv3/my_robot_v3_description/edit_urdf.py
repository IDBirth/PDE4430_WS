import os
import trimesh

# change this to your actual meshes folder
MESH_DIR = "meshes"

# Offsets taken directly from your xacro visual origins
OFFSETS = {
    "camera_1.stl": (-0.336, 0.0, -0.714),
    "lidar_1.stl": (0.046, 0.186, -1.124),
    "left_wheel_1.stl": (0.024, -0.2012, -0.508),
    "right_wheel_1.stl": (0.024, 0.5732, -0.508),
    "caster_wheel_1.stl": (0.365, 0.186227, -0.464),
    "left_arm_1.stl": (0.057381, -0.331012, -0.1545),
    "right_arm_1.stl": (0.057381, 0.331013, -0.1545),
}

for fname, (x, y, z) in OFFSETS.items():
    src = os.path.join(MESH_DIR, fname)
    dst = os.path.join(MESH_DIR, fname.replace(".stl", "_local.stl"))

    mesh = trimesh.load(src, force="mesh")
    mesh.apply_translation([x, y, z])  # bake the URDF visual origin into the mesh
    mesh.export(dst)
    print("Wrote:", dst)
