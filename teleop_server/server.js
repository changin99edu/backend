const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const rosnodejs = require('rosnodejs');
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
    cors: {
        origin: '*',
        methods: ['GET', 'POST'],
        credentials: true
    }
});

// ROS 노드 초기화
rosnodejs.initNode('/teleop_node')
    .then((rosNode) => {
        // cmd_vel 퍼블리셔 설정
        const pub = rosNode.advertise('/cmd_vel', geometry_msgs.Twist);

        io.on('connection', (socket) => {
            console.log('New client connected');

            // 클라이언트에서 teleop 명령 수신
            socket.on('teleop', (data) => {
                // 데이터 유효성 검사
                if (data && data.command && typeof data.command.linear === 'number' && typeof data.command.angular === 'number') {
                    const twist = new geometry_msgs.Twist();
                    twist.linear.x = data.command.linear;  // 직선 속도
                    twist.angular.z = data.command.angular;  // 회전 속도
                    pub.publish(twist);  // ROS 퍼블리셔로 퍼블리시
                } else {
                    console.error('Invalid teleop data received:', data);
                }
            });

            // 클라이언트가 연결 해제되었을 때 로봇 정지 명령 전송
            socket.on('disconnect', () => {
                console.log('Client disconnected');

                // 연결이 끊겼을 때 로봇 정지
                const stopTwist = new geometry_msgs.Twist();
                stopTwist.linear.x = 0;
                stopTwist.angular.z = 0;
                pub.publish(stopTwist);
            });
        });

        // 서버 시작
        const PORT = process.env.PORT || 7003;
        server.listen(PORT, () => console.log(`Server running on port ${PORT}`));
    })
    .catch((error) => {
        console.error('Failed to initialize ROS node:', error);
    });
