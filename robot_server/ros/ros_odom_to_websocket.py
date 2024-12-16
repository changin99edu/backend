import rospy
from nav_msgs.msg import Odometry
import websocket
import json
import time
import math

# WebSocket 서버와 연결 설정 (WebSocket 서버 주소로 변경)
ws = None
latest_data = {}  # 가장 최근의 /odom 데이터를 저장

# WebSocket 연결 함수
def connect_websocket():
    global ws
    while True:
        try:
            if ws:  # 기존 연결이 있을 경우 닫기
                ws.close()
            ws = websocket.WebSocket()
            ws.connect("ws://172.30.1.33:7000")  # WebSocket 서버 주소를 설정하세요
            rospy.loginfo("WebSocket connected successfully")
            break
        except Exception as e:
            rospy.logwarn(f"Failed to connect to WebSocket server: {e}")
            time.sleep(5)  # 연결 실패 시 5초 대기 후 재시도

# /odom 토픽에서 수신한 데이터를 최신 상태로 저장하는 콜백 함수
def odom_callback(data, namespace):
    global latest_data
    # 네임스페이스가 유효한지 확인
    if not namespace:
        rospy.logwarn("Received data with empty namespace, ignoring.")
        return

    # 로봇의 위치와 방향 정보
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # 속도 정보 계산 (벡터 크기 계산) 후 cm/s 단위로 변환
    linear_velocity = data.twist.twist.linear
    speed_mps = math.sqrt(linear_velocity.x ** 2 + linear_velocity.y ** 2 + linear_velocity.z ** 2)
    speed_cms = round(speed_mps * 100, 2)  # cm/s 단위로 변환하고 소수점 2자리까지

    # 네임스페이스 포함하여 최신 데이터 저장
    latest_data[namespace] = {
        "namespace": namespace,
        "x": position.x,
        "y": position.y,
        "z": position.z,
        "orientation": {
            "x": orientation.x,
            "y": orientation.y,
            "z": orientation.z,
            "w": orientation.w
        },
        "speed": speed_cms  # 속도 추가
    }

# 일정 시간 간격으로 WebSocket에 데이터를 전송하는 함수
def send_data_to_websocket(event):
    global ws
    for namespace, robot_data in latest_data.items():
        try:
            ws.send(json.dumps(robot_data))
            rospy.loginfo(f"Sent data for {namespace}: {robot_data}")
        except (websocket.WebSocketConnectionClosedException, BrokenPipeError):
            rospy.logwarn("WebSocket connection closed, attempting to reconnect...")
            connect_websocket()
        except Exception as e:
            rospy.logwarn(f"Error while sending data: {e}")

# 현재 활성화된 /odom 토픽을 탐색하여 네임스페이스를 자동으로 구독하는 함수
def subscribe_to_odom_topics():
    # 현재 활성화된 모든 토픽 목록 가져오기
    topics = rospy.get_published_topics()
    
    # /odom 토픽이 포함된 네임스페이스 구독
    for topic, topic_type in topics:
        if topic.endswith("/odom") and topic_type == "nav_msgs/Odometry":
            # 네임스페이스 추출
            namespace = topic.split("/odom")[0].strip("/")
            rospy.loginfo(f"Subscribing to topic: {topic} with namespace: {namespace}")
            rospy.Subscriber(topic, Odometry, odom_callback, callback_args=namespace)

# ROS 노드를 초기화하고 /odom 토픽을 자동으로 구독
def listener():
    rospy.init_node('odom_listener', anonymous=True)
    connect_websocket()  # WebSocket 연결 설정
    subscribe_to_odom_topics()  # 활성화된 /odom 토픽 자동 구독
    
    # 0.5초마다 WebSocket에 데이터 전송
    rospy.Timer(rospy.Duration(0.5), send_data_to_websocket)

    rospy.spin()  # 노드를 계속 실행 상태로 유지

if __name__ == '__main__':
    listener()
