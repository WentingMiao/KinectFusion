
import trimesh
import numpy as np

# Create a cube mesh
mesh_1 = trimesh.creation.box()

# Create a second cube mesh, translated by (1,0,0)
translation_matrix = np.eye(4)
translation_matrix[:3, 3] = [0.5, 0.5, 0]
mesh_2 = mesh_1.copy()
mesh_2.apply_transform(translation_matrix)


# Generate the first .off file
mesh_1.export("cube_1.off")

# Generate the second .off file
mesh_2.export("cube_2.off")