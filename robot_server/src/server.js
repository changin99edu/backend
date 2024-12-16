require('dotenv').config();
const WebSocket = require('ws');
const http = require('http');
const axios = require('axios');
const app = require('./app');
const cron = require('node-cron'); // Polling 작업을 위한 Cron
const { exec } = require('child_process'); // Python 스크립트를 실행하기 위해 사용
const Robot = require('../src/models/robot.model'); // 로봇 모델 가져오기 (MongoDB)
const WorkflowQueue = require('../src/models/workflowQueue.model'); // 워크플로우 모델 가져오기
const moveRobotToTask = require('../src/controllers/robot.controller').moveRobotToTask; // 이동 로직 가져오기

// 포트 설정
const PORT = process.env.PORT || 5559; // Express 서버 포트
const WS_PORT = process.env.WS_PORT || 7000; // WebSocket 포트

// Express 서버 실행
const server = http.createServer(app);
server.listen(PORT, () => {
  console.log(`HTTP Server is running on port ${PORT}`);
});

// WebSocket 서버 설정
const wss = new WebSocket.Server({ port: WS_PORT });
wss.on('connection', (ws) => {
  console.log(`WebSocket connected on port ${WS_PORT}`);

  ws.on('message', async (data) => {
    try {
      const robotData = JSON.parse(data);

      // 배터리 데이터 처리 로직
      if (robotData.type === 'battery' && robotData.battery) {
        const { percentage } = robotData.battery;

        if (robotData.namespace) {
          const updatedRobot = await Robot.findOneAndUpdate(
            { name: robotData.namespace },
            { 'status.battery': percentage },
            { new: true, runValidators: true }
          );

          if (updatedRobot) {
            console.log(`Battery percentage updated: ${updatedRobot.name} = ${percentage}%`);
          } else {
            console.warn(`Robot with namespace ${robotData.namespace} not found in database`);
          }
        }
      }

      // 좌표와 속도 데이터 처리 로직
      else if (robotData.namespace && robotData.x !== undefined && robotData.y !== undefined && robotData.speed !== undefined) {
        const updatedRobot = await Robot.findOneAndUpdate(
          { name: robotData.namespace },
          {
            'location.x': robotData.x,
            'location.y': robotData.y,
            'status.speed': robotData.speed
          },
          { new: true, runValidators: true }
        );

        if (updatedRobot) {
          console.log(`Robot updated: ${updatedRobot.name} - Coordinates [${updatedRobot.location.x}, ${updatedRobot.location.y}] - Speed: ${robotData.speed}`);
        } else {
          console.warn(`Robot with namespace ${robotData.namespace} not found in database`);
        }
      }
    } catch (error) {
      console.error('Error processing robot data:', error);
    }
  });

  ws.on('close', () => {
    console.log('WebSocket connection closed');
  });
});

// Python 스크립트 실행 함수
const executePythonScript = (scriptName) => {
  exec(`python3 ./ros/${scriptName}`, (error, stdout, stderr) => {
    if (error) {
      console.error(`Error executing ${scriptName}: ${error.message}`);
      return;
    }
    if (stderr) {
      console.error(`${scriptName} stderr: ${stderr}`);
      return;
    }
    console.log(`${scriptName} output: ${stdout}`);
  });
};

// Python 스크립트 실행
/*executePythonScript('odom_publisher.py');*/// maxbuffer 발생하는데 이 부분 유효하게 작동 중인 친구인지 몰라, 검증 귀찮,,, 주석처리 24.11.28. [#좌표#maxbuffer]
executePythonScript('ros_odom_to_websocket.py');
executePythonScript('robot_status.py');

const TASK_LOG_SERVER_URL = 'http://172.30.1.33:8080/task/task-logs'; // 작업 기록 API URL

// Polling 작업: 활성화된 로봇의 대기 작업을 주기적으로 처리
const processRobotWorkflows = async () => {
  try {
    const activeRobots = await Robot.find({ active: 1 }); // 활성화된 로봇만 검색

    for (const robot of activeRobots) {
      if (!robot.currentWorkflow || robot.currentWorkflow.node === null) {
        console.log(`Robot ${robot.name} has no active workflow. Attempting to assign a new one...`);

        const workflowQueue = await WorkflowQueue.findOne({ robotId: robot._id });
        if (workflowQueue && workflowQueue.workflows.length > 0) {
          const nextWorkflow = workflowQueue.workflows.shift();
          robot.currentWorkflow = {
            node: nextWorkflow.node,
            step: nextWorkflow.step,
            x: nextWorkflow.x,
            y: nextWorkflow.y,
            status: 'In Progress',
          };

          await robot.save();
          await workflowQueue.save();

          console.log(`Assigned new workflow to robot ${robot.name}:`, robot.currentWorkflow);

          // 작업 기록 서버에 작업 정보 저장
          try {
            const taskLogResponse = await axios.post(TASK_LOG_SERVER_URL, {
              robotName: robot.name,
              robotIp: robot.ip,
              nodeName: nextWorkflow.node,
              step: nextWorkflow.step,
              status: 'In Progress', // 초기 상태
            });
            console.log(`Task log created on external server:`, taskLogResponse.data);
          } catch (logError) {
            console.error(`Failed to log task for robot ${robot.name}:`, logError.response?.data || logError.message);
          }

          // 작업이 할당된 후 move-to-task 로직 실행
          const req = { body: { robotId: robot._id } };
          const res = {
            status: (code) => ({
              json: (data) => console.log(`moveRobotToTask response (${code}):`, data),
            }),
          };
          await moveRobotToTask(req, res);
        } else {
          console.log(`No workflows available for robot ${robot.name}`);
        }
      } else {
        /*console.log(`Robot ${robot.name} is already processing a workflow.`);*/
      }
    }
  } catch (error) {
    console.error('Error processing robot workflows:', error);
  }
};

// Polling 주기 설정 (5초마다 실행)
cron.schedule('*/5 * * * * *', async () => {
  await processRobotWorkflows();
});