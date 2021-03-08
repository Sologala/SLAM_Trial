import pangolin
import OpenGL.GL as gl
import random
import numpy as np

import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as sciRotation
from copy import deepcopy
import g2o
from collections import defaultdict
from utils.tools import *

# read points

with open("data.txt", "r") as f:
    data = [[float(word) for word in line.strip().split(" ")]
            for line in f.readlines()]
#
points = [d[:3] for d in data]
colors = [d[3:] for d in data]


# read poses

with open("pose.txt", 'r') as f:
    data = [[float(word) for word in line.strip().split(" ")]
            for line in f.readlines()]

GTquats = [d[:4] for d in data]
GTtranslate = [d[4:] for d in data]


poses = quatAndt2pose(GTquats, GTtranslate)


Rigs = []

for i in range(len(poses)):
    rig = Rig()
    rig.setPose(poses[i])
    # print(poses[i])
    # print(rig.L.getPose(), "---- \n ---", rig.W.getPose())
    Rigs.append(rig)


prj_res = []
print(len(Rigs))
for i in range(len(points)):
    res = {"L": {},
           "W": {}}
    for j in range(len(Rigs)):
        if (Rigs[j].L.checkInview(points[i])):
            res["L"][j] = Rigs[j].L.project(points[i])
        if (Rigs[j].W.checkInview(points[i])):
            res["W"][j] = Rigs[j].W.project(points[i])
    prj_res.append(res)


# showPoints(points, colors, Rigs)
quats = GTquats.copy()
translate = GTtranslate.copy()
# sample new traanslate with noise
for j in range(len(GTquats)):
    for i in range(2):
        translate[j][i] += 1 * random.gauss(0, 0.12)

    for i in range(4):
        quats[j][i] += random.gauss(0, 0.12)

posesFK = quatAndt2pose(quats, translate)
for i, rig in enumerate(Rigs):
    rig.setPose(posesFK[i])


# project


'''
    trail1
    全局BA
'''


def optimization():
    # inint optimizer
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    focal_length = 900
    principal_point = (256, 256)
    cam = g2o.CameraParameters(focal_length, principal_point, 0)
    cam.set_id(0)
    optimizer.add_parameter(cam)

    # true_points = np.hstack([
    #     np.random.random((500, 1)) * 3 - 1.5,
    #     np.random.random((500, 1)) - 0.5,
    #     np.random.random((500, 1)) + 3])

    sse = defaultdict(float)

    true_poses = poses
    poseVecterices = []
    # add pose verteices 添加rig 的pose
    for i, rig in enumerate(Rigs):
        # pose here means transform points from world coordinates to camera coordinates

        pose = g2o.SE3Quat(rig.L.getRotation(), rig.L.getTraslation())

        v_se3 = g2o.VertexSE3Expmap()
        v_se3.set_id(i)

        v_se3.set_estimate(pose)
        # set the firset frame fixed

        if i < 2:
            v_se3.set_fixed(True)

        error = v_se3.estimate().translation() - true_poses[i][:3, 3]

        sse[0] += np.sum(error**2)
        poseVecterices.append(v_se3)
        optimizer.add_vertex(v_se3)

    # 添加地图点
    id = len(Rigs)
    true_points = np.array(points)

    for i, point in enumerate(true_points):
        vp = g2o.VertexSBAPointXYZ()
        vp.set_id(id)

        vp.set_marginalized(True)
        #  add normal noise to each point
        vp.set_estimate(point)
        optimizer.add_vertex(vp)

        visible = prj_res[i]

        for j, uv in visible["L"].items():
            print(i, j, uv)
            edge = g2o.EdgeProjectP2MC()
            edge.set_vertex(0, vp)
            edge.set_vertex(1, optimizer.vertex(j))
            edge.set_measurement(uv)
            edge.set_information(np.identity(2))
            edge.set_robust_kernel(g2o.RobustKernelHuber())
            # 观测的两个相机的 参数
            # edge.set_parameter_id(0, 0)
            optimizer.add_edge(edge)

        id += 1

    print('num vertices:', len(optimizer.vertices()))
    print('num edges:', len(optimizer.edges()))

    print('Performing full BA:')
    optimizer.initialize_optimization()
    optimizer.set_verbose(True)
    optimizer.optimize(100)
    print("done ")
    for i in range(len(Rigs)):
        v_se3 = optimizer.vertex(i)
        print(v_se3.estimate().to_homogeneous_matrix())
        print(g2o.SE3Quat(true_poses[i][:3, :3],
                          true_poses[i][:3, 3]).to_homogeneous_matrix())
        print("------------------")

        error = v_se3.estimate().translation() - true_poses[i][:3, 3]

        sse[1] += np.sum(error**2)

    print('\nRMSE (inliers only):')
    print('before optimization:', np.sqrt(sse[0] / len(Rigs)))
    print('after  optimization:', np.sqrt(sse[1] / len(Rigs)))


optimization()
