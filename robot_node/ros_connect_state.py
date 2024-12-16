import rospy
import json
from std_msgs.msg import String
import requests

ROBOT_IP = "172.30.1.61"  # 로봇의 IP 주소
API_ENDPOINT = "http://172.30.1.33:5559/robot/update_status"  # 상태를 업데이트하는 API 엔드포인트

def callback(data):
    message = json.loads(data.data)
    
    # 새로운 노드가 등록되었을 때 로봇 이름과 일치하는지 확인
    if message.get("event") == "node_started" and ROBOT_IP in message.get("node"):
        update_robot_status('Waiting')

def update_robot_status(status):
    payload = {
        'robot_ip': ROBOT_IP,
        'status': status
    }
    try:
        response = requests.post(API_ENDPOINT, json=payload)
        if response.status_code == 200:
            rospy.loginfo(f"로봇 상태가 '{status}'으로 업데이트되었습니다.")
        else:
            rospy.logwarn(f"상태 업데이트 실패: {response.status_code}, {response.text}")
    except requests.RequestException as e:
        rospy.logerr(f"상태 업데이트 중 오류 발생: {e}")

def listener():
    rospy.init_node('robot_connection_listener', anonymous=True)
    rospy.Subscriber("/rosout_agg", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
