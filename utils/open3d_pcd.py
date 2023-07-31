Simport open3d as o3d
import numpy as np
import argparse

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', "--path", required=True, type=str)
    parser.add_argument('-f', "--flip", action='store_true')
    opt = parser.parse_args()

    # pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(opt.path)
    # print(pcd_data.path)
    # print('pcd', pcd)
    if opt.flip:
        pts = np.asarray(pcd.points)
        pts[:, 0] = -pts[:, 0]
        pcd.points = o3d.utility.Vector3dVector(pts)
    # print('points', pts, pts.shape)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()