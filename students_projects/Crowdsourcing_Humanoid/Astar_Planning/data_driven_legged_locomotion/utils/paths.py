import numpy as np

def MA_filter(path, p = 15, q = 15):
    """Applies a MA filter to the path to smooth it"""
    extended_path = np.concatenate([path[0]*np.ones((p,2)), path, path[-1]*np.ones((q,2))], axis=0)
    smoothed_path = np.zeros_like(extended_path)
    for i in range(extended_path.shape[0]):
        smoothed_path[i] = np.mean(extended_path[max(0, i-p):min(i+q, extended_path.shape[0])], axis=0)
    path_diff = np.diff(smoothed_path, axis=0)
    distances = np.linalg.norm(path_diff, axis=1)
    critical_points = np.argwhere(distances < 0.03) + 1
    smoothed_path = np.delete(smoothed_path, critical_points, axis=0)
    return smoothed_path

def path_tangent_vectors(path):
    path_diff = np.diff(path, axis=0)
    path_diff = path_diff / np.linalg.norm(path_diff, axis=1)[:,None]
    return path_diff

def closest_point_on_path(x, path, path_d):
    x = np.array(x)
    dist = np.linalg.norm(x-path, axis=-1)
    min_index = np.argmin(dist)
    tangent_vector = path_d[min_index]
    return min_index, dist[min_index], tangent_vector