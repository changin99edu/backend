import rospy
from custom_msgs.msg import TaskPath  # TaskPath.msg는 사용자 정의 메시지입니다.
from geometry_msgs.msg import Point
import json

def publish_task_path(data):
    rospy.init_node('task_path_publisher', anonymous=True)

    # 데이터 파싱
    robot_name = data.get("robotName")
    current_step = data.get("currentWorkflowStep")
    path_data = data.get("path")

    if not robot_name:
        rospy.logerr("Robot name is missing in the data.")
        return

    # 네임스페이스 기반 토픽 생성
    topic_name = f"/{robot_name}/task_path"
    pub = rospy.Publisher(topic_name, TaskPath, queue_size=10)

    rospy.loginfo(f"Publishing TaskPath to {topic_name}")

    task_path_msg = TaskPath()
    task_path_msg.robotName = robot_name
    task_path_msg.currentWorkflowStep = current_step

    # Path 데이터 변환
    for point in path_data:
        path_point = Point()
        path_point.x = point["x"]
        path_point.y = point["y"]
        path_point.z = 0  # Z축은 0으로 설정
        task_path_msg.path.append(path_point)

    # 메시지 게시
    rate = rospy.Rate(10)  # 초당 10번 게시
    for _ in range(3):  # 3번 게시
        pub.publish(task_path_msg)
        rate.sleep()

    rospy.loginfo("TaskPath message published successfully.")

if __name__ == "__main__":
    try:
        # 예시 JSON 데이터를 사용하여 테스트
        sample_data = json.loads('''{
            "robotName": "robot1",
            "currentWorkflowStep": "step1",
            "path": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
        }''')
        publish_task_path(sample_data)
    except rospy.ROSInterruptException:
        pass
