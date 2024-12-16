require('dotenv').config();
const app = require('./app');
const express = require('express');
const WebSocket = require('ws');
const { exec } = require('child_process'); // Python 스크립트를 실행하기 위해 사용

const PORT = process.env.PORT;
const WS_PORT = 5050; // WebSocket 서버 포트

// 서버 실행 시 Python 스크립트 실행 (odom_publisher.py와 ros_odom_to_websocket.py)
exec('python3 src/ros/odom_publisher.py', (error, stdout, stderr) => {
  if (error) {
    console.error(`Error executing odom_publisher.py: ${error.message}`);
    return;
  }
  if (stderr) {
    console.error(`Error output: ${stderr}`);
    return;
  }
  console.log(`Odom Publisher Output: ${stdout}`);
});

exec('python3 src/ros/ros_odom_to_websocket.py', (error, stdout, stderr) => {
  if (error) {
    console.error(`Error executing ros_odom_to_websocket.py: ${error.message}`);
    return;
  }
  if (stderr) {
    console.error(`Error output: ${stderr}`);
    return;
  }
  console.log(`ROS Odom to WebSocket Output: ${stdout}`);
});

// Express 서버 실행
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

// WebSocket 서버 설정
const wss = new WebSocket.Server({ port: WS_PORT });

wss.on('connection', function connection(ws) {
  console.log('WebSocket connected');

  ws.on('message', function message(data) {
    // 여기서 로봇의 위치 데이터를 처리합니다.
    // 받은 데이터는 JSON 형식으로 변환하여 사용 가능합니다.
    //const robotData = JSON.parse(data);
    //console.log('Received robot data:', robotData);

    // 여기서 로봇의 좌표를 처리하거나 저장하는 로직을 추가할 수 있습니다.
  });

  ws.on('close', () => {
    console.log('WebSocket disconnected');
  });
});

module.exports = { wss };
