import rospy
from std_msgs.msg import String
import json
import base64
import os
import subprocess

def receive_map(data):
    try:
        map_info = json.loads(data.data)
        map_data_base64 = map_info['map_data']
        file_extension = map_info['file_extension']
        metadata = map_info['metadata']

        map_data = base64.b64decode(map_data_base64)

        os.makedirs(os.path.expanduser('~/map'), exist_ok=True)

        file_path = os.path.expanduser(f'~/map/map.{file_extension}')
        yaml_path = os.path.expanduser('~/map/map.yaml')

        # 기존 파일 삭제
        for file in os.listdir(os.path.expanduser('~/map')):
            file_path_to_remove = os.path.expanduser(f'~/map/{file}')
            os.remove(file_path_to_remove)

        # 새로운 맵 파일 저장
        with open(file_path, 'wb') as f:
            f.write(map_data)

        rospy.loginfo(f"Map received and saved to {file_path}.")
        rospy.loginfo(f"Map file size: {len(map_data)} bytes")
        rospy.loginfo(f"Map file content (first 100 bytes): {map_data[:100]}")
        rospy.loginfo(f"Map file content (last 100 bytes): {map_data[-100:]}")

        # YAML 파일 저장
        with open(yaml_path, 'w') as yaml_file:
            yaml_file.write(metadata)

        # 저장된 맵 파일 다시 읽기 (확인용)
        with open(file_path, 'rb') as f:
            saved_map_data = f.read()
        rospy.loginfo(f"Saved map file size: {len(saved_map_data)} bytes")
        rospy.loginfo(f"Saved map file content (first 100 bytes): {saved_map_data[:100]}")
        rospy.loginfo(f"Saved map file content (last 100 bytes): {saved_map_data[-100:]}")

        # map_server 노드 실행
        subprocess.Popen(['rosrun', 'map_server', 'map_server', yaml_path])

    except Exception as e:
        rospy.logerr(f"Error receiving map: {e}")

if __name__ == "__main__":
    rospy.init_node('map_receiver', anonymous=True)
    rospy.Subscriber('/map_topic', String, receive_map)
    rospy.spin()


