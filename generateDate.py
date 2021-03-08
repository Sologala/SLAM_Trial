import pangolin
import OpenGL.GL as gl
import random
import numpy as np

import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as sciRotation


# generate data
npts = 1000 * 4
w = 200
h = 300
xx = []
yy = []
zz = []
r = []
g = []
b = []

for i in range(npts):
    x = w * np.random.uniform() - w / 2
    y = h * np.random.uniform() - h / 2
    z = 90 + 5 * (math.sin((x + w) * 1.0 / 60) +
                  math.cos((y + h) * 1.0 / 60))
    xx.append(x)
    yy.append(y)
    zz.append(z)
mx, Mx = min(xx), max(xx)
my, My = min(yy), max(yy)
mz, Mz = min(zz), max(zz)

for i in range(npts):
    x, y, z = xx[i], yy[i], zz[i]
    r.append(((x - mx) / (Mx - mx) * 1))
    g.append(((y - my) / (My - my) * 1))
    b.append(((z - mz) / (Mz - mz) * 1))
points = np.ones((npts, 3))
points[:, 0] = np.array(xx)
points[:, 1] = np.array(yy)
points[:, 2] = np.array(zz)
colors = np.ones((npts, 3))
colors[:, 0] = np.array(b)
colors[:, 1] = np.array(g)
colors[:, 2] = np.array(r)

print(points.shape)
# write out to file
with open("data.txt", "w") as f:
    for i in range(npts):
        s = "{} {} {} {} {} {}\n".format(points[i, 0], points[i, 1],
                                         points[i, 2], colors[i, 0], colors[i, 1], colors[i, 2])
        print(s)
        f.write(s)


# ax = plt.subplot(111, projection='3d')  # 创建一个三维的绘图工程
# ax.scatter(xx, yy, zz)
# plt.show()


pangolin.CreateWindowAndBind('Main', 640, 480)
gl.glEnable(gl.GL_DEPTH_TEST)

# set init pose

# Define Projection and initial ModelView matrix
scam = pangolin.OpenGlRenderState(
    pangolin.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 20000),
    pangolin.ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin.AxisDirection.AxisY))
handler = pangolin.Handler3D(scam)

# Create Interactive View in window
dcam = pangolin.CreateDisplay()
dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0/480.0)
dcam.SetHandler(handler)

panel = pangolin.CreatePanel('ui')
panel.SetBounds(0.0, 1.0, 0.0, 180/640.)

button = pangolin.VarBool('ui.Button', value=False, toggle=False)

clickCnt = 0
nPose = 15

# noise parameters
mu = 0
sigma = 0.12

# with open("pose.txt", "w") as f:
#     while not pangolin.ShouldQuit():
#         gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
#         gl.glClearColor(1.0, 1.0, 1.0, 1.0)
#         dcam.Activate(scam)
#         if pangolin.Pushed(button):

#             print('currnet pose')
#             pose = scam.GetModelViewMatrix()
#             pose = np.array(pose)
#             R = pose[:3, :3]
#             t = pose[:3, 3]

#             # print(R, t)
#             sciR = sciRotation.from_matrix(R)
#             quat = sciR.as_quat()
#             # write out pose to file
#             # print(quat)
#             ss = "{} {} {} {} {} {} {}\n".format(
#                 quat[0], quat[1], quat[2], quat[3], t[0], t[1], t[2])
#             f.write(ss)
#             print(ss)

#             # change pose

#             for i in range(2):
#                 t[i] += 10 + 5 * random.gauss(mu, sigma)
#             for i in range(4):
#                 quat[i] += random.gauss(mu, 0.02)

#             pose[:3, 3] = t
#             pose[:3, :3] = sciRotation.from_quat(quat).as_matrix()
#             newpose = pangolin.OpenGlMatrix(pose)

#             scam.SetModelViewMatrix(newpose)

#             clickCnt += 1
#             if (clickCnt == nPose):
#                 break

#         gl.glPointSize(2)
#         gl.glColor3f(1.0, 0.0, 0.0)
#         pangolin.DrawPoints(points, colors)

#         pangolin.FinishFrame()
