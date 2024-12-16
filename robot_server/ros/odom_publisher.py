import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu  # 예시로 IMU 센서 데이터 사용

current_position = [0.0, 0.0]  # 로봇의 현재 위치를 저장할 변수

def imu_callback(data):
    global current_position
    # IMU 또는 로봇의 다른 센서 데이터를 이용해 로봇의 위치를 계산
    # 이 부분은 실제 센서나 로봇 제어 시스템에 따라 달라집니다.
    current_position[0] = data.linear_acceleration.x  # 예시로 IMU 데이터를 위치로 변환
    current_position[1] = data.linear_acceleration.y

def odom_callback(data):
    # /odom 토픽의 데이터를 터미널에 출력
    position = data.pose.pose.position
    rospy.loginfo(f"Odometry received - x: {position.x}, y: {position.y}")

def publisher():
    rospy.init_node('odom_publisher', anonymous=True)
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    rospy.Subscriber("/imu", Imu, imu_callback)  # IMU 데이터를 구독
    rospy.Subscriber("/odom", Odometry, odom_callback)  # /odom 토픽 구독하여 데이터 출력

    while not rospy.is_shutdown():
        # Odometry 메시지 생성
        odom = Odometry()

        # 실제 센서 데이터를 이용해 Odometry 메시지 작성
        odom.pose.pose.position.x = current_position[0]
        odom.pose.pose.position.y = current_position[1]
        # Orientation 데이터도 실제 데이터를 기반으로 설정해야 함 (예시로는 생략)
        
        # Odometry 메시지 발행
        rospy.loginfo(f"Publishing Odom: {odom.pose.pose.position.x}, {odom.pose.pose.position.y}")
        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
