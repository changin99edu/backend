#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json

# 콜백 함수: 작업 서버로부터 전달된 작업 명령을 처리
def task_callback(data):
    task_info = data.data  # 작업 명령을 문자열로 받음
    rospy.loginfo(f"Received task: {task_info}")

    # 받은 작업 정보를 JSON 형태로 변환
    task = json.loads(task_info)

    # 작업 세부 내용 로그 출력
    task_id = task.get('taskId', 'N/A')
    task_name = task.get('taskName', 'N/A')
    task_type = task.get('taskType', 'N/A')
    
    rospy.loginfo(f"Task ID: {task_id}")
    rospy.loginfo(f"Task Name: {task_name}")
    rospy.loginfo(f"Task Type: {task_type}")

    # 작업 유형에 따라 로봇의 동작을 결정
    if task_type == 'MoveToLocation':
        move_to_location(task)
    elif task_type == 'PickUpItem':
        pick_up_item(task)
    else:
        rospy.logwarn(f"Unknown task type: {task_type}")

# 특정 위치로 이동하는 작업 처리 함수
def move_to_location(task):
    location = task.get('location', {})
    x = location.get('x', 0)
    y = location.get('y', 0)

    rospy.loginfo(f"Moving to location: x={x}, y={y}")
    # 로봇 이동 로직을 여기에 추가

# 물건을 집는 작업 처리 함수
def pick_up_item(task):
    item_id = task.get('item_id', 'N/A')
    rospy.loginfo(f"Picking up item with ID: {item_id}")
    # 물건 집는 로직을 여기에 추가

# 서브스크라이버 초기화 및 구독 시작
def task_listener():
    # 노드 초기화
    rospy.init_node('task_subscriber_node', anonymous=True)

    # '/robot/task' 토픽을 구독하여 작업 명령 수신
    rospy.Subscriber('/robot/task', String, task_callback)

    # ROS 노드 계속 실행
    rospy.spin()

if __name__ == '__main__':
    task_listener()
