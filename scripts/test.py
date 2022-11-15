# -*- coding: utf-8 -*-

# TODO: robotiq-2F-85の平行リンク(mimic)モジュール作成

import time
import random
import numpy as np
import pybullet as p
import pybullet_data as pd
import matplotlib.pyplot as plt
from collections import namedtuple

class Robot():
    def __init__(self, pos, ori):
        self.base_pos = pos
        self.base_ori = p.getQuaternionFromEuler(ori)

    def load(self):
        self.__init_robot__()
        self.__parse_joint_info__()
        self.__post_load__()

    def __init_robot__(self):
        raise NotImplementedError

    def __parse_joint_info__(self):
        numJoints = p.getNumJoints(self.id)
        jointInfo = namedtuple(
                'jointInfo', ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity', 'controllable'])
        self.joints = []
        self.controllable_joints = []

        for i in range(numJoints):
            info = p.getJointInfo(self.id, i)

            jointID = info[0]
            jointName = info[1].decode('utf-8')
            jointType = info[2] # [JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED]
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            print('\n\n\n')
            print(jointID)
            print(jointName)
            print(jointType)
            print(jointDamping)
            print(jointFriction)
            print('\n\n\n')

            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
            info = jointInfo(
                    jointID,
                    jointName,
                    jointType,
                    jointDamping,
                    jointFriction,
                    jointLowerLimit,
                    jointUpperLimit,
                    jointMaxForce,
                    jointMaxVelocity,
                    controllable)
            self.joints.append(info)

        assert len(self.controllable_joints) >= self.arm_num_dofs
        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]

    def __post_load__(self):
        pass

    def move_ee(self, control_method, action):
        """
        @param control_method: ['end' or 'joint'] (str)
        @param action: if 'end' -> [(x, y, z, r, p, y)] (list)
                       if 'joint' -> [each joint angle] (list)

        """
        assert control_method in ('end', 'joint')

        if control_method == 'end':
            x, y, z, roll, pitch, yaw = action

            pos = (x, y, z)
            ori = p.getQuaternionFromEuler((roll, pitch, yaw))

            joint_poses = p.calculateInverseKinematics(
                    self.id,
                    self.eef_id,
                    pos,
                    ori,
                    self.arm_lower_limits,
                    self.arm_upper_limits,
                    self.arm_joint_ranges,
                    self.arm_rest_poses,
                    maxNumIterations=50)
        elif control_method == 'joint':
            assert len(action) == self.arm_num_dofs
            joint_poses = action

        for (i, joint_id) in enumerate(self.arm_controllable_joints):
            p.setJointMotorControl2(
                    self.id,
                    joint_id,
                    p.POSITION_CONTROL,
                    joint_poses[i],
                    force=self.joints[joint_id].maxForce,
                    maxVelocity=self.joints[joint_id].maxVelocity)

class UR5(Robot):
    def __init_robot__(self):
        self.eef_id = 8
        self.arm_num_dofs = 6
        self.arm_rest_poses = [-0.7854, -1.5708, 1.5708, -1.5708, -1.5708, 0.]
        self.id = p.loadURDF(
                '../urdf/tsuzuki_ur5.urdf',
                self.base_pos,
                self.base_ori,
                useFixedBase=True,
                flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

class TsuzukiSimEnv():
    def __init__(self, robot, vis=True):
        self._robot = robot
        self._vis = vis
        self._GRAVITY = -9.81
        self._dt = 1/100.

        self.physicsClient = p.connect(p.GUI if self._vis else p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.setGravity(0, 0, self._GRAVITY)
        p.setTimeStep(self._dt)
        p.setPhysicsEngineParameter(fixedTimeStep=self._dt, numSolverIterations=100, numSubSteps=10)

        self._planeID = p.loadURDF('plane.urdf')
        self._robot.load()

        self._robot.move_ee('joint', self._robot.arm_rest_poses)
        self.set_camera(
                robot_id=self._robot.id,
                size=(1280, 1024),
                near=0.27,
                far=3.0)
        """
        self.set_container()
        self.set_objects(num=10)
        """

    def set_objects(self, num):
        objID = []

        for i in range(num):
            x = random.uniform(0.4, 0.55)
            y = random.uniform(-0.2, 0.2)
            z = random.uniform(0.9, 1.5)
            pos = [x, y, z]

            rx = random.uniform(-0.05, 0.05)
            ry = random.uniform(-0.05, 0.05)
            rz = random.uniform(-0.05, 0.05)
            ori = p.getQuaternionFromEuler([rx, ry, rz])

            objID.append(p.loadURDF(
                '../meshes/knuckle/knuckle_approximate.urdf',
                pos,
                ori,
                flags=p.URDF_USE_SELF_COLLISION))

            for _ in range(50): # 物体を1つずつ落下させる猶予
                p.stepSimulation()

    def set_container(self):
        self.container_visualID = p.createVisualShape(
                shapeType=p.GEOM_MESH,
                fileName='../meshes/container/container.stl',
                rgbaColor=[0.75, 0.75, 0.75, 0.85],
                specularColor=[0., 0., 0.],
                meshScale=[.001, .001, .001])
 
        self.container_collisionID = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName='../meshes/container/container.stl',
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                meshScale=[.001, .001, .001])
 
        self.container_init_position = [0.495, 0., 0.215]
        self.container_init_orientation = p.getQuaternionFromEuler([0., 0., 0.])
 
        self.container_ID = p.createMultiBody(
                baseMass=0, # bulletでは mass=0 でstatic-rigidbody
                baseVisualShapeIndex=self.container_visualID,
                baseCollisionShapeIndex=self.container_collisionID,
                basePosition=self.container_init_position,
                baseOrientation=self.container_init_orientation)

    def set_camera(self, robot_id, size, near, far):
        """
        - https://www.programcreek.com/python/example/122131/pybullet.computeViewMatrix
        - https://qiita.com/maiueo/items/7c0c039a92102c7b7fe8
        - https://stackoverflow.com/questions/60430958/understanding-the-view-and-projection-matrix-from-pybullet

        """
        self.width = size[0]
        self.height = size[1]
        self.near = near
        self.far = far
        aspect = self.width/self.height
        fx = 1509.92479709
        fy = 1509.92479709
        cx = 499.077
        cy = 511.474
        #fov = 45.94 # [deg]
        pos, ori, _, _, _, _ = p.getLinkState(robot_id, linkIndex=20) # suppose that the camera link conforms to the definition of a CV coordinate system

        rot_mat = p.getMatrixFromQuaternion(ori)
        rot_mat = np.array(rot_mat).reshape(3, 3)

        cam_eye_pos = np.array(pos)
        cam_vec = rot_mat.dot((0, 0, 1)) # Z-axis base's transform
        cam_up_vec = rot_mat.dot((0, -1, 0)) # Y-axis's negative direction
        cam_target_pos = cam_eye_pos + (self.near*cam_vec) # cam_vec's multiplier: 0.016, self.near

        self.view_matrix = p.computeViewMatrix(cam_eye_pos, cam_target_pos, cam_up_vec)
        _view_matrix = np.array(self.view_matrix).reshape((4, 4), order='F')

        self.projection_matrix = (2.0*fx/self.width,       0.0,                      0.0,                                         0.0,
                                  0.0,                     2.0*fy/self.height,       0.0,                                         0.0,
                                  1.0-(2.0*cx/self.width), (2.0*cy/self.height)-1.0, (self.far+self.near)/(self.near-self.far),  -1.0,
                                  0.0,                     0.0,                      2.0*self.far*self.near/(self.near-self.far), 0.0)
        _projection_matrix = np.array(self.projection_matrix).reshape((4, 4), order='F')

        self.tran_pix_world = np.linalg.inv(_projection_matrix @ _view_matrix)

    def shot_camera(self):
        # Get depth values using the OpenGL renderer
        _, _, rgb, depth, seg = p.getCameraImage(
                width = self.width,
                height = self.height,
                viewMatrix = self.view_matrix,
                projectionMatrix = self.projection_matrix,
                renderer = p.ER_BULLET_HARDWARE_OPENGL,
                #flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                )

        return rgb, depth, seg

    def rgbd_2_world(self, w_val, h_val, d_map):
        """
        - https://stackoverflow.com/a/62247245


        """
        x = (2*w_val-self.width)/self.width
        y = -(2*h_val-self.height)/self.height
        z = 2*d_map[h_val, w_val]-1

        pix_pos = np.array((x, y, z, 1))
        position = self.tran_pix_world @ pix_pos
        position /= position[3]

        return position[:3]

    def rgbd_2_world_org(self, w_val, h_val, d_val):
        """
        - https://stackoverflow.com/a/62247245


        """
        x = (2*w_val-self.width)/self.width
        y = -(2*h_val-self.height)/self.height
        z = 2*d_val-1

        pix_pos = np.array((x, y, z, 1))
        position = self.tran_pix_world @ pix_pos
        position /= position[3]

        return position[:3]

    def rgbd_2_world_batch(self, depth):
        """
        - https://stackoverflow.com/a/62247245


        """
        x = (2*np.arange(0, self.width)-self.width)/self.width
        x = np.repeat(x[None, :], self.height, axis=0)
        y = -(2*np.arange(0, self.height)-self.height)/self.height
        y = np.repeat(y[:, None], self.width, axis=1)
        z = 2*depth-1

        pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
        position = self.tran_pix_world @ pix_pos.T
        position = position.T
        position[:, :] /= position[:, 3:4]

        return position[:, :3].reshape(*x.shape, -1)

    def step(self):
        p.stepSimulation()

if __name__ == '__main__':
    robot = UR5(
            (0, 0, 0),
            (0, 0, 0))

    env = TsuzukiSimEnv(
            robot=robot,
            vis=True)

    """
    c, d, s = env.shot_camera()
    pos_world = env.rgbd_2_world(w_val=300, h_val=200, d_map=d)
    print(pos_world)
    """

    while (1):
        env.step()
