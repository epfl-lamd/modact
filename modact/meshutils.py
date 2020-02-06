import numpy as np
import trimesh


def merge_meshes(meshes):
    """
    Concatenate meshes.
    Parameters
    ----------
    meshes: List of Trimeshes
    Returns
    ----------
    result: Trimesh object containing all faces of meshes
    """
    new_normals = np.vstack([m.face_normals for m in meshes])
    faces_list = []
    face_sources = []
    faces_count = 0
    for i, m in enumerate(meshes):
        faces_list.append(m.faces + faces_count)
        faces_count += len(m.vertices)
        face_sources.append(np.ones(len(m.faces)) * i)

    new_faces = np.vstack(faces_list)
    new_vertices = np.vstack([m.vertices for m in meshes])

    result = trimesh.Trimesh(vertices=new_vertices,
                             faces=new_faces,
                             face_normals=new_normals,
                             process=False)
    result._cache.id_set()

    result.metadata['face_sources'] = np.concatenate(face_sources)

    return result
