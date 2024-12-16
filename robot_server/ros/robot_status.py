#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState   # 배터리 상태
from std_msgs.msg import String            # 상태 (예: "Idle", "Tasking" 등)
import websocket
import json

ws = None               # WebSocket 객체
latest_battery_data = {}  # 최신 배터리 데이터를 저장할 딕셔너리

# WebSocket 연결
def connect_websocket():
    global ws
    while True:
        try:
            ws = websocket.WebSocket()
            ws.connect("ws://172.30.1.33:7000")  # 서버의 WebSocket 주소
            rospy.loginfo("WebSocket connected")
            break
        except Exception as e:
            rospy.logwarn("Failed to connect WebSocket: %s", e)
            rospy.sleep(5)

# 배터리 상태 콜백 함수
def battery_callback(data, namespace):
    global latest_battery_data
    latest_battery_data[namespace] = {
        'type': 'battery',
        'namespace': namespace,
        'battery': {
            'percentage': int(data.percentage * 100)  # 퍼센티지를 정수로 변환
        }
    }

# 주기적으로 배터리 상태 전송
def send_battery_status(event):
    if latest_battery_data:
        for namespace, battery_data in latest_battery_data.items():
            try:
                ws.send(json.dumps(battery_data))
                rospy.loginfo(f"Battery data sent for {namespace}: {battery_data}")
            except (websocket.WebSocketConnectionClosedException, BrokenPipeError):
                rospy.logwarn("WebSocket connection closed, trying to reconnect...")
                connect_websocket()

# 활성화된 배터리 상태 토픽을 구독하여 네임스페이스를 분리
def subscribe_to_battery_topics():
    topics = rospy.get_published_topics()
    for topic, topic_type in topics:
        if topic.endswith("/battery_state") and topic_type == "sensor_msgs/BatteryState":
            # 네임스페이스 추출
            namespace = topic.split("/battery_state")[0].strip("/")
            rospy.loginfo(f"Subscribing to battery topic: {topic} with namespace: {namespace}")
            rospy.Subscriber(topic, BatteryState, battery_callback, callback_args=namespace)

if __name__ == '__main__':
    rospy.init_node('robot_monitor')

    # WebSocket 연결 시도
    connect_websocket()

    # 활성화된 배터리 상태 토픽 자동 구독
    subscribe_to_battery_topics()

    # 10초 주기로 배터리 상태 전송
    rospy.Timer(rospy.Duration(10), send_battery_status)

    rospy.spin()
