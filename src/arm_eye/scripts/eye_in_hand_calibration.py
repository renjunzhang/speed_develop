import os
import cv2
from math import cos ,sin
import math
import numpy as np
from pyquaternion import Quaternion

from scipy.spatial.transform import Rotation as R_scipy

class EyeInHandCalibration:
    def __init__(self):
        pass

    def Angle2Rotation(self, x, y, z):
        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R

    def Rotation2Quaternion(self,rotateMatrix):
        q = Quaternion(matrix=rotateMatrix)
        return q.x,q.y,q.z,q.w


    def Euler2Quaternion(self,euler):
        r = R_scipy.from_euler('xyz', euler, degrees=True)
        quaternion = r.as_quat()
        return quaternion  #(x, y, z, w)

    def Quaternion2Euler(self,quate):
        r = R_scipy.from_quat(quate)
        euler = r.as_euler('xyz', degrees=True)
        return euler

    def Euler2Rotation(self,euler):
        r = R_scipy.from_euler('xyz', euler, degrees=True)
        rotation_matrix = r.as_matrix()
        return rotation_matrix

    def Quaternion2Rotation(self, ox, oy, oz,ow):
        q=[ox,oy,oz,ow]
        rot_matrix = np.array(
            [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
             [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
             [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
            dtype=np.float64)
        return rot_matrix



    def Gripper2Base(self, tx, ty, tz, ox, oy, oz,ow):

        R_gripper2base = self.Quaternion2Rotation(ox, oy, oz,ow)
        T_gripper2base = np.array([[tx], [ty], [tz]])
        Matrix_gripper2base = np.column_stack([R_gripper2base, T_gripper2base])
        Matrix_gripper2base = np.row_stack((Matrix_gripper2base, np.array([0, 0, 0, 1])))
        R_gripper2base = Matrix_gripper2base[:3, :3]
        T_gripper2base = Matrix_gripper2base[:3, 3].reshape((3, 1))
        return R_gripper2base, T_gripper2base



    def Target2Camera(self, tx, ty, tz, ox, oy, oz,ow):
        R_target2camera = self.Quaternion2Rotation(ox, oy, oz,ow)
        T_target2camera = np.array([[tx], [ty], [tz]])
        Matrix_target2camera = np.column_stack([R_target2camera, T_target2camera])
        Matrix_target2camera = np.row_stack((Matrix_target2camera, np.array([0, 0, 0, 1])))
        R_target2camera = Matrix_target2camera[:3, :3]
        T_target2camera = Matrix_target2camera[:3, 3].reshape((3, 1))
        return R_target2camera, T_target2camera

    def Calibrate(self,pose_datas):
        R_target2camera_list = []
        T_target2camera_list = []
        R_gripper2base_list = []
        T_gripper2base_list = []
        for data in pose_datas:
            camera_pose=data["camera_pose"]
            robot_pose=data["robot_pose"]


            R_target2camera, T_target2camera = self.Target2Camera(camera_pose["x"],camera_pose["y"],camera_pose["z"],
                                                                  camera_pose["ox"],camera_pose["oy"],camera_pose["oz"],
                                                                  camera_pose["ow"])
            R_gripper2base, T_gripper2base = self.Gripper2Base(robot_pose["x"],robot_pose["y"],robot_pose["z"],
                                                               robot_pose["ox"],robot_pose["oy"],robot_pose["oz"],
                                                               robot_pose["ow"])
            R_target2camera_list.append(R_target2camera)
            T_target2camera_list.append(T_target2camera)
            R_gripper2base_list.append(R_gripper2base)
            T_gripper2base_list.append(T_gripper2base)


        R_camera2gripper, T_camera2gripper = cv2.calibrateHandEye(R_gripper2base_list, T_gripper2base_list,
                                                                  R_target2camera_list, T_target2camera_list,cv2.CALIB_HAND_EYE_TSAI)
        return R_camera2gripper, T_camera2gripper, R_gripper2base_list, T_gripper2base_list, R_target2camera_list, T_target2camera_list


    def process(self, img_path, pose_path):
        image_list = []
        for root, dirs, files in os.walk(img_path):
            if files:
                for file in files:
                    image_name = os.path.join(root, file)
                    image_list.append(image_name)
        R_target2camera_list = []
        T_target2camera_list = []
        for img_path in image_list:
            img = cv2.imread(img_path)
            R_target2camera, T_target2camera = self.target2camera(img)
            R_target2camera_list.append(R_target2camera)
            T_target2camera_list.append(T_target2camera)
        R_gripper2base_list = []
        T_gripper2base_list = []
        data = xlrd2.open_workbook(pose_path)
        table = data.sheets()[0]
        for row in range(table.nrows):
            x = table.cell_value(row, 0)
            y = table.cell_value(row, 1)
            z = table.cell_value(row, 2)
            tx = table.cell_value(row, 3)
            ty = table.cell_value(row, 4)
            tz = table.cell_value(row, 5)
            R_gripper2base, T_gripper2base = self.gripper2base(x, y, z, tx, ty, tz)
            R_gripper2base_list.append(R_gripper2base)
            T_gripper2base_list.append(T_gripper2base)


        R_camera2base, T_camera2base = cv2.calibrateHandEye(R_gripper2base_list, T_gripper2base_list,
                                                            R_target2camera_list, T_target2camera_list)
        return R_camera2base, T_camera2base, R_gripper2base_list, T_gripper2base_list, R_target2camera_list, T_target2camera_list

    def Rotation2Euler(self,R):


        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2]) * 180 / np.pi
            y = math.atan2(-R[2, 0], sy) * 180 / np.pi
            z = math.atan2(R[1, 0], R[0, 0]) * 180 / np.pi
        else:
            x = math.atan2(-R[1, 2], R[1, 1]) * 180 / np.pi
            y = math.atan2(-R[2, 0], sy) * 180 / np.pi
            z = 0

        return x, y, z

    def Rotvector2Quaternion(self,rotvector):

        Rm = cv2.Rodrigues(rotvector)[0]
        return  self.Rotation2Quaternion(Rm)



    def check_result(self, R_cb, T_cb, R_gb, T_gb, R_tc, T_tc):
        for i in range(len(R_gb)):
            RT_gripper2base = np.column_stack((R_gb[i], T_gb[i]))
            RT_gripper2base = np.row_stack((RT_gripper2base, np.array([0, 0, 0, 1])))
            RT_base2gripper = np.linalg.inv(RT_gripper2base)
            print(RT_base2gripper)

            RT_camera_to_base = np.column_stack((R_cb, T_cb))
            RT_camera_to_base = np.row_stack((RT_camera_to_base, np.array([0, 0, 0, 1])))
            print(RT_camera_to_base)

            RT_target_to_camera = np.column_stack((R_tc[i], T_tc[i]))
            RT_target_to_camera = np.row_stack((RT_target_to_camera, np.array([0, 0, 0, 1])))
            RT_camera2target = np.linalg.inv(RT_target_to_camera)
            print(RT_camera2target)

            RT_target_to_gripper = RT_base2gripper @ RT_camera_to_base @ RT_camera2target
            print("第{}次验证结果为:".format(i))
            print(RT_target_to_gripper)
            print('')


    def CheckResult(self,R_cg, T_cg, R_gb, T_gb, R_tc, T_tc):
        for i in range(len(R_gb)):
            RT_gripper2base = np.column_stack((R_gb[i], T_gb[i]))
            RT_gripper2base = np.row_stack((RT_gripper2base, np.array([0, 0, 0, 1])))
            # RT_base2gripper = np.linalg.inv(RT_gripper2base)
            # print(RT_base2gripper)
            RT_camera_to_gripper=np.column_stack((R_cg, T_cg))
            RT_camera_to_gripper = np.row_stack((RT_camera_to_gripper, np.array([0, 0, 0, 1])))

            RT_target_to_camera = np.column_stack((R_tc[i], T_tc[i]))
            RT_target_to_camera = np.row_stack((RT_target_to_camera, np.array([0, 0, 0, 1])))

            RT_target_to_base=RT_gripper2base@RT_camera_to_gripper@RT_target_to_camera


            R_target_to_base=RT_target_to_base[:3,:3]
            ox,oy,oz=self.Rotation2Euler(R_target_to_base)
            x,y,z=RT_target_to_base[0,3],RT_target_to_base[1,3],RT_target_to_base[2,3],

            print("第{}次验证结果为:".format(i))
            # print(RT_target_to_base)
            print(ox,oy,oz,x,y,z)
            print('')





def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    R = np.dot(R_x, np.dot(R_y, R_z))
    return R




if __name__ == "__main__":
    pose_datas=[]
    with open("data_kjg0714.txt") as f:
        for line in f:
            pose_datas.append(eval(line))

    robot_pose_euler=[[4.576,-0.730,0.697],
                      [4.376,0.338,0.557],
                      [4.071,0.655,0.031],
                      [4.289,0.288,-0.343],
                      [4.151,0.809,0.439],
                      [4.229,0.551,0.661],
                      [4.515,-0.463,0.884],
                      [4.442,-1.319,-0.616],
                      [4.487,-1.188,-0.553],
                      [3.752,-2.693,-0.569]]

    calibrator = EyeInHandCalibration()
    eur=[]
    for i in range(len(pose_datas)):
        robot_rx=math.degrees(robot_pose_euler[i][0])
        robot_ry=math.degrees(robot_pose_euler[i][1])
        robot_rz=math.degrees(robot_pose_euler[i][2])
        ox, oy, oz, ow = calibrator.Euler2Quaternion((robot_rx,robot_ry,robot_rz))

        pose_datas[i]["robot_pose"]["x"]=-pose_datas[i]["robot_pose"]["x"]
        pose_datas[i]["robot_pose"]["y"]=-pose_datas[i]["robot_pose"]["y"]
        pose_datas[i]["robot_pose"]["z"]=-pose_datas[i]["robot_pose"]["z"]


        Rx,Ry,Rz=calibrator.Quaternion2Euler((pose_datas[i]["robot_pose"]["ox"],pose_datas[i]["robot_pose"]["oy"],pose_datas[i]["robot_pose"]["oz"],pose_datas[i]["robot_pose"]["ow"]))


        eur.append([math.radians(Rx),math.radians(Ry),math.radians(Rz)])

        pose_datas[i]["robot_pose"]["ox"]=ox
        pose_datas[i]["robot_pose"]["oy"]=oy
        pose_datas[i]["robot_pose"]["oz"]=oz
        pose_datas[i]["robot_pose"]["ow"]=ow

    a=-np.array(eur)+np.array(robot_pose_euler)

    eur=np.array(eur)
    robot_pose_euler=np.array(robot_pose_euler)
    robot_rx=math.degrees(robot_pose_euler[0][0])
    robot_ry=math.degrees(robot_pose_euler[0][1])
    robot_rz=math.degrees(robot_pose_euler[0][2])
    a0=calibrator.Euler2Rotation((robot_rx,robot_ry,robot_rz))



    R_cg, T_cg, R_gb, T_gb, R_tc, T_tc = calibrator.Calibrate(pose_datas)
    q=calibrator.Rotation2Quaternion(R_cg)

    # 0.02457676885228062 0.040442379187918674 0.0021048459947996465        -0.021376497862137127  -0.7039089126841439 -0.7099637462823785 0.0026013357465902473
    calibrator.CheckResult(R_cg, T_cg, R_gb, T_gb, R_tc, T_tc)
