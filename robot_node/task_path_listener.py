import rospy
from custom_msgs.msg import TaskPath

def callback(data):
    rospy.loginfo(f"Received task path for {data.robotName}:")
    rospy.loginfo(f"Current Step: {data.currentWorkflowStep}")
    for point in data.path:
        rospy.loginfo(f"Point: x={point.x}, y={point.y}, z={point.z}")

def task_path_listener():
    robot_name = rospy.get_param("~robot_name", "robot1")
    rospy.init_node("task_path_listener", anonymous=True)

    topic_name = f"/{robot_name}/task_path"
    rospy.Subscriber(topic_name, TaskPath, callback)

    rospy.spin()

if __name__ == "__main__":
    task_path_listener()
