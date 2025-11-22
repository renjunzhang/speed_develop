import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
import json
import time


def GetPoseDatas(nums):
    pub_robot_pose = rospy.Publisher(
        "inquire_robot_pose", String, queue_size=10)
    pub_camera_pose = rospy.Publisher(
        "/inquire_camera_pose", String, queue_size=10)
    n = 0
    pose_datas = []
    while n < nums:

        print("输入任意数字开始采集")
        a = input()

        while pub_robot_pose.get_num_connections() == 0:
            print("机械臂没有订阅节点")
            time.sleep(0.01)
        pub_robot_pose.publish(String("123"))
        current_robot_pose = rospy.wait_for_message(
            "/answer_robot_pose", PoseStamped)

        robot_pose = {"x": current_robot_pose.pose.position.x,
                      "y": current_robot_pose.pose.position.y,
                      "z": current_robot_pose.pose.position.z,
                      "ox": current_robot_pose.pose.orientation.x,
                      "oy": current_robot_pose.pose.orientation.y,
                      "oz": current_robot_pose.pose.orientation.z,
                      "ow": current_robot_pose.pose.orientation.w, }

        while pub_camera_pose.get_num_connections() == 0:
            print("相机没有订阅节点")
            time.sleep(0.01)
        msg = {"code": 21, "solve_type": "pnp"}
        pub_camera_pose.publish(String(json.dumps(msg)))

        # current_camera_pose=rospy.wait_for_message(
        #     "/point_coordinate", PoseStamped)
        # camera_pose={"x":current_camera_pose.pose.position.x,
        #              "y":current_camera_pose.pose.position.y,
        #              "z":current_camera_pose.pose.position.z,
        #              "ox":current_camera_pose.pose.orientation.x,
        #              "oy":current_camera_pose.pose.orientation.y,
        #              "oz":current_camera_pose.pose.orientation.z,
        #              "ow":current_camera_pose.pose.orientation.w,}

        # pose_data={"robot_pose":robot_pose,"camera_pose":camera_pose}
        # pose_datas.append(pose_data)

        current_camera_pose = rospy.wait_for_message(
            "aruco_pose", String)
        current_camera_pose = json.loads(current_camera_pose.data)

        if None == current_camera_pose:
            print("相机信息无效")
            continue

        pose_data = {"robot_pose": robot_pose,
                     "camera_pose": current_camera_pose}
        pose_datas.append(pose_data)
        n += 1
        print("完成第%d次采集" % n)

    return pose_datas


if __name__ == "__main__":
    # 2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("hand_eye_calibrate")

    # a=(0.004355854354798794-0.08426244556903839)**2+(0.014639144763350487--0.006695365067571402)**2
    # b=(-0.5734256872064317--0.6569958525369158)**2

    pose_datas = GetPoseDatas(10)

    with open("./datas/data_sys_08031.txt", "w") as f:
        for line in pose_datas:
            f.write(str(line)+"\n")

    rospy.spin()
