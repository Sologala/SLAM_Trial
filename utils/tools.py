import numpy as np
import pangolin
import OpenGL.GL as gl
from scipy.spatial.transform import Rotation as sciRotation

Camera_fx = 955.40503
Camera_fy = 955.40503
Camera_cx = 256.0
Camera_cy = 256.0

CameraWide_fx = 179.25313
CameraWide_fy = 179.25313
CameraWide_cx = 256.0
CameraWide_cy = 256.0


class Camera:
    def __init__(self, _fx, _fy, _cx, _cy):
        self.fx = _fx
        self.fy = _fy
        self.cx = _cx
        self.cy = _cy
        self.K = np.eye(3)
        self.K[0, 0] = _fx
        self.K[1, 1] = _fy
        self.K[0, 2] = _cx
        self.K[1, 2] = _cy
        print(self.K)
        self.width = 2 * self.cx
        self.height = 2 * self.cy

    def project(self, Pc):

        return [self.fx * Pc[0] / Pc[2] + self.cx,
                self.fy * Pc[1] / Pc[2] + self.cy]

    def checkInview(self, uv):
        if uv[0] >= 0 and uv[0] < self.width and uv[1] >= 0 and uv[1] < self.height:
            return True
        else:
            return False


class Obj:
    def __init__(self, _cam, _Tcw=np.eye(4)):
        self.setPose(_Tcw)
        self.cam = _cam

    def project(self, Pw):
        # print(np.hstack([Pw, [1]]))
        Pc = np.matmul(self.getPose(), np.hstack([Pw, [1]]))
        # print("pc", Pc)
        return self.cam.project(Pc[:3])

    def checkInview(self, Pw):
        # print("pw", Pw)
        uv = self.project(Pw)
        # print("uv", uv)
        return self.cam.checkInview(uv)

    def setPose(self, _Tcw):
        self.mTcw = _Tcw.copy()
        self.updatePose()

    def getPose(self):
        return np.array(self.mTcw)

    def getPoseInverse(self):
        return np.array(self.mTwc)

    def getRotation(self):
        return np.array(self.mRcw)

    def getTraslation(self):
        return np.array(self.mtcw)

    def updatePose(self):
        self.mRcw = self.mTcw[:3, :3]
        self.mRwc = self.mRcw.T
        self.mtcw = self.mTcw[:3, 3]
        self.mOw = np.matmul(-self.mRwc,  self.mtcw)
        self.mTwc = np.eye(4)
        self.mTwc[:3, :3] = self.mRwc
        self.mTwc[:3, 3] = self.mOw


gcamL = Camera(Camera_fx, Camera_fy, Camera_cx, Camera_cy)
gcamW = Camera(CameraWide_fx, CameraWide_fy, CameraWide_cx, CameraWide_cy)


class Rig:
    def __init__(self):
        self.L = Obj(gcamL)
        self.W = Obj(gcamW)
        self.Twl = np.array([1., 0., 0., 0.40, 0., 1., 0.,
                             0., 0., 0., 1., 0., 0., 0., 0., 1.]).reshape((4, 4))
        self.Tlw = np.array([1.,  0.,  0., -0.40,  0.,  1.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,
                             0.,  0.,  1.]).reshape((4, 4))

    def setPose(self, _Tcw):
        self.L.setPose(_Tcw)
        self.W.setPose(np.matmul(_Tcw, self.Twl))


def showPoints(points, colors, Rigs):

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
    # print(poses)
    while not pangolin.ShouldQuit():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        dcam.Activate(scam)
        gl.glPointSize(2)
        gl.glColor3f(1.0, 0.0, 0.0)
        # draw points
        pangolin.DrawPoints(points, colors)

        # draw poses
        for rig in Rigs:
            drawrig(rig)
        pangolin.FinishFrame()


def drawrig(rig):
    drawCamera(rig.L.getPose(), (0, 1, 0))
    drawCamera(rig.W.getPose(), (1, 1, 0))


def drawCamera(pose, _color):
    mTcw = pose
    mRcw = mTcw[:3, :3]
    mRwc = mRcw.T
    mtcw = mTcw[:3, 3]
    mOw = np.matmul(-mRwc, mtcw)

    mTwc = np.eye(4)
    mTwc[:3, :3] = mRwc
    mTwc[:3, 3] = mOw
    mTwc = mTwc.T
    w = 4
    h = w * 0.75
    z = w * 0.6

    gl.glPushMatrix()

    gl.glMultMatrixd(np.array(mTwc))

    gl.glLineWidth(w)
    gl.glColor3f(_color[0], _color[1], _color[2])

    gl.glBegin(gl.GL_LINES)
    gl.glVertex3f(0, 0, 0)
    gl.glVertex3f(w, h, z)
    gl.glVertex3f(0, 0, 0)
    gl.glVertex3f(w, -h, z)
    gl.glVertex3f(0, 0, 0)
    gl.glVertex3f(-w, -h, z)
    gl.glVertex3f(0, 0, 0)
    gl.glVertex3f(-w, h, z)

    gl.glVertex3f(w, h, z)
    gl.glVertex3f(w, -h, z)

    gl.glVertex3f(-w, h, z)
    gl.glVertex3f(-w, -h, z)

    gl.glVertex3f(-w, h, z)
    gl.glVertex3f(w, h, z)

    gl.glVertex3f(-w, -h, z)
    gl.glVertex3f(w, -h, z)
    gl.glEnd()

    gl.glPopMatrix()


def quatAndt2pose(_quat, _t):
    _pose = []
    for i in range(len(_quat)):
        R = sciRotation.from_quat(_quat[i]).as_matrix()
        temp_pose = np.eye(4)
        temp_pose[:3, :3] = R
        temp_pose[:3, 3] = _t[i]
        # print("dsafsda")
        _pose.append(temp_pose.copy())
    return _pose


if __name__ == "__main__":
    with open("data.txt", "r") as f:
        data = [[float(word) for word in line.strip().split(" ")]
                for line in f.readlines()]
    #
    points = [d[:3] for d in data]
    colors = [d[3:] for d in data]

    with open("pose.txt", 'r') as f:
        data = [[float(word) for word in line.strip().split(" ")]
                for line in f.readlines()]
    # print(data)
    GTquats = [d[:4] for d in data]
    GTtranslate = [d[4:] for d in data]
    pose = quatAndt2pose(GTquats, GTtranslate)
    # print(points, colors, pose)
    showPoints(points, colors, pose)
