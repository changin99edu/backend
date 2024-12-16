const mongoose = require('mongoose');
const Robot = require('../models/robot.model');
const axios = require('axios');
const { GridFSBucket } = require('mongodb');
const WebSocket = require('ws');
const Map = require('../models/map.model');
const rosnodejs = require('rosnodejs');
console.log('CMAKE_PREFIX_PATH:', process.env.CMAKE_PREFIX_PATH);
console.log('ROS_PACKAGE_PATH:', process.env.ROS_PACKAGE_PATH);

const exec = require('child_process').exec; // SSH를 통해 원격 명령어 실행
const WorkflowQueue = require('../models/workflowQueue.model');

let gfs;
mongoose.connection.once('open', () => {
  gfs = new GridFSBucket(mongoose.connection.db, { bucketName: 'maps' });
});

// 자신이 보유한 로봇 조회 (이름이 0이 아닌 로봇만)
exports.getRobots = async (req, res) => {
  try {
    const robots = await Robot.find({ userId: req.user.id, name: { $ne: '0' } });
    res.json(robots);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch robots');
  }
  
  
};

// 로봇 등록 및 ROS와 연동
exports.registerRobot = async (req, res) => {
  try {
    const { name, ip, model, speed, sshUser } = req.body; // sshUser 유지

    // 새로운 로봇 객체 생성
    const robot = new Robot({
      name,
      ip,
      model,
      userId: req.user.id,
      sshUser, // sshUser 필드 유지
      status: {
        speed: speed || 0 // 속도 값이 없을 경우 기본값 0으로 설정
      }
    });

    // 로봇 저장
    await robot.save();

    // WorkflowQueue 생성 - 로봇 ID 사용
    const workflowQueue = new WorkflowQueue({
      robotId: robot._id,
      workflows: [] // 초기에는 빈 workflows 배열로 설정
    });
    await workflowQueue.save();

    // 성공 응답
    res.status(200).json({ message: 'Robot registered successfully', robot });
  } catch (error) {
    console.error('Error registering robot:', error);
    res.status(500).json({ message: 'Error registering robot', error: error.message });
  }
};

// 로봇 업데이트
exports.updateRobot = async (req, res) => {
  try {
    const { name, ip, model, speed, description, active } = req.body; // 설명 및 active 필드 추가
    const { id } = req.params;

    // 로봇 정보 업데이트 (설명, 속도 및 active 값도 함께 업데이트)
    const robot = await Robot.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { 
        name, 
        ip, 
        model, 
        description, // 설명 필드 업데이트
        'status.speed': speed || 0,  // 속도 필드 업데이트
        active: active !== undefined ? active : 1  // active 필드 업데이트
      },
      { new: true, runValidators: true } // runValidators 옵션 추가
    );

    if (!robot) {
      return res.status(404).json({ message: 'Robot not found or not authorized' });
    }

    res.status(200).json({ message: 'Robot updated successfully', robot });
  } catch (error) {
    console.error('Error updating robot:', error);
    res.status(500).json({ message: 'Error updating robot', error: error.message });
  }
};


// 로봇에게 맵 전송
exports.sendMapToRobots = async (req, res) => {
  try {
    // ROS 노드 초기화
    await rosnodejs.initNode('/map_publisher_node');
    const nh = rosnodejs.nh;

    // 선택된 맵 정보 조회
    const monitoredMapResponse = await axios.get('http://172.30.1.30:5557/map/monitored', {
      headers: { Authorization: req.headers.authorization }
    });

    const monitoredMap = monitoredMapResponse.data;

    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }

    console.log(`Monitored map ID: ${monitoredMap._id}`);

    // 맵 파일 스트림 생성
    const mapFileResponse = await axios.get(`http://172.30.1.30:5557/map/file/${monitoredMap._id}`, {
      responseType: 'arraybuffer',
      headers: { Authorization: req.headers.authorization }
    });

    const mapData = Buffer.from(mapFileResponse.data, 'binary');
    console.log(`Map data size: ${mapData.length}`);

    // 메타데이터 파일 스트림 생성
    const metadataResponse = await axios.get(`http://172.30.1.30:5557/map/metadata/${monitoredMap._id}`, {
      responseType: 'text',
      headers: { Authorization: req.headers.authorization }
    });

    const metadata = metadataResponse.data;

    // 파일 확장자 추출
    const fileExtension = monitoredMap.filename.split('.').pop();

    // ROS 메시지 생성
    const std_msgs = rosnodejs.require('std_msgs').msg;
    const mapTopic = nh.advertise('/map_topic', std_msgs.String);
    const mapMsg = new std_msgs.String({
      data: JSON.stringify({
        map_data: mapData.toString('base64'),
        file_extension: fileExtension,
        metadata: metadata
      })
    });

    // ROS 토픽으로 맵 데이터 퍼블리시
    mapTopic.publish(mapMsg);
    console.log('Map data published to /map_topic');
    
    res.status(200).json({ message: 'Map sent to robots successfully' });
  } catch (error) {
    console.error('Error sending map to robots:', error);
    res.status(500).json({ message: 'Error sending map to robots', error: error.message });
  }
};

// 로봇 등록 해제
/*exports.unregisterRobot = async (req, res) => {
  try {
    const { id } = req.params;
    const robot = await Robot.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { name: '0' }, // 로봇 이름을 '0'으로 설정하여 등록 해제
      { new: true }
    );

    if (!robot) {
      return res.status(404).json({ message: 'Robot not found or not authorized' });
    }

    res.status(200).json({ message: 'Robot unregistered successfully', robot });
  } catch (error) {
    console.error('Error unregistering robot:', error);
    res.status(500).json({ message: 'Error unregistering robot', error: error.message });
  }
};*/
exports.unregisterRobot = async (req, res) => {
  try {
    const { id } = req.params;

    // 로봇을 찾고 삭제
    const robot = await Robot.findOneAndDelete({ _id: id, userId: req.user.id });

    if (!robot) {
      return res.status(404).json({ message: 'Robot not found or not authorized' });
    }

    // 해당 로봇의 워크플로우 큐 삭제
    await WorkflowQueue.deleteOne({ robotId: id });

    res.status(200).json({ message: 'Robot deleted successfully', robot });
  } catch (error) {
    console.error('Error deleting robot:', error);
    res.status(500).json({ message: 'Error deleting robot', error: error.message });
  }
};

exports.updateRobotPosition = async (req, res) => {
  try {
    const { namespace, x, y } = req.body;

    // 로깅 추가: 요청에서 받은 데이터 출력
    console.log(`Received update request for namespace: ${namespace}, x: ${x}, y: ${y}`);

    if (x === undefined || y === undefined) {
      return res.status(400).json({ message: 'x와 y 좌표가 필요합니다.' });
    }

    // 네임스페이스로 위치 업데이트
    const robot = await Robot.findOneAndUpdate(
      { name: namespace },  // 네임스페이스로 조회
      { 
        'location.x': x, 
        'location.y': y 
      },  // x, y 좌표 업데이트
      { new: true, runValidators: true }
    );

    // 로봇이 존재하는지 확인
    if (!robot) {
      console.warn(`No robot found with namespace: ${namespace}`);
      return res.status(404).json({ message: '해당 네임스페이스를 가진 로봇을 찾을 수 없습니다.' });
    }

    // 업데이트된 로봇 정보 출력
    console.log(`Robot with namespace ${namespace} updated with new position: x=${robot.location.x}, y=${robot.location.y}`);

    res.status(200).json({ message: '로봇 위치가 성공적으로 업데이트되었습니다.', robot });
  } catch (error) {
    console.error('로봇 위치 업데이트 중 오류 발생:', error);
    res.status(500).json({ message: '로봇 위치 업데이트 실패', error: error.message });
  }
};

exports.bringupRobot = async (req, res) => {
  try {
    const { robotIp } = req.params;
    const rosMasterUri = process.env.ROS_MASTER_URI || 'http://172.30.1.30:11311'; // 환경 변수에 ROS 마스터 URI 설정

    // 데이터베이스에서 로봇 정보 조회
    const robot = await Robot.findOne({ ip: robotIp });

    if (!robot) {
      return res.status(404).json({ message: '해당 IP를 가진 로봇을 찾을 수 없습니다.' });
    }

    const sshUser = robot.sshUser; // 데이터베이스에서 sshUser 가져오기

    // Bringup 명령어 구성
    const sshCommand = `
      ssh ${sshUser}@${robotIp} "source /opt/ros/noetic/setup.bash && \
      source ~/catkin_ws/devel/setup.bash && \
      export ROS_MASTER_URI=${rosMasterUri} && \
      export ROS_HOSTNAME=${robotIp} && \
      echo 'All environment variables set correctly'; \
      roslaunch turtlebot3_bringup turtlebot3_robot.launch"
    `;

    // SSH 명령 실행
    exec(sshCommand, (error, stdout, stderr) => {
      if (error) {
        console.error(`Error executing bringup via SSH: ${error.message}`);
        console.error(`stderr: ${stderr}`);
        return res.status(500).json({ message: 'Failed to execute bringup on the robot via SSH', error: error.message });
      }

      if (stderr) {
        console.warn(`SSH Command stderr: ${stderr}`);
      }

      console.log(`SSH Command Output: ${stdout}`);
      console.log(`Robot at ${robotIp} successfully brought up.`);
      res.status(200).json({ message: `Robot at ${robotIp} successfully brought up.` });
    });

  } catch (error) {
    console.error('Error bringing up the robot:', error);
    res.status(500).json({ message: 'Error bringing up the robot', error: error.message });
  }
};
exports.updateRobotBattery = async (req, res) => {
  try {
    const { namespace, percentage } = req.body;

    // 유효성 검사: 배터리 잔량 필드가 존재하는지 확인
    if (percentage === undefined) {
      return res.status(400).json({ message: 'percentage 값이 필요합니다.' });
    }

    // 네임스페이스로 배터리 상태 업데이트
    const robot = await Robot.findOneAndUpdate(
      { name: namespace },  // 네임스페이스를 로봇 이름으로 조회
      {
        'status.battery': percentage
      },  // 배터리 상태 (percentage 값으로 업데이트)
      { new: true, runValidators: true }
    );

    // 로봇이 존재하는지 확인
    if (!robot) {
      console.warn(`No robot found with namespace: ${namespace}`);
      return res.status(404).json({ message: '해당 네임스페이스를 가진 로봇을 찾을 수 없습니다.' });
    }

    // 업데이트된 로봇 정보 출력
    console.log(`Robot with namespace ${namespace} updated with new battery percentage: ${robot.status.battery}%`);

    res.status(200).json({ message: '로봇 배터리 상태가 성공적으로 업데이트되었습니다.', robot });
  } catch (error) {
    console.error('로봇 배터리 상태 업데이트 중 오류 발생:', error);
    res.status(500).json({ message: '로봇 배터리 상태 업데이트 실패', error: error.message });
  }
};

exports.updateRobotStatus = async (req, res) => {
  const { robot_ip, status } = req.body;

  try {
      // 로봇의 IP 주소로 로봇을 찾고 상태를 업데이트
      const robot = await Robot.findOneAndUpdate({ ip: robot_ip }, { 'status.state': status }, { new: true });

      if (!robot) {
          return res.status(404).json({ message: '로봇을 찾을 수 없습니다.' });
      }

      res.status(200).json({ message: '로봇 상태가 업데이트되었습니다.', robot });
  } catch (error) {
      console.error(error);
      res.status(500).json({ message: '로봇 상태 업데이트 중 오류가 발생했습니다.' });
  }
};
exports.toggleRobotActive = async (req, res) => {
  try {
    const { id } = req.params; // 로봇 ID를 URL 파라미터로 받음

    // 로봇 조회 및 현재 active 상태 반전
    const robot = await Robot.findOne({ _id: id, userId: req.user.id });

    if (!robot) {
      return res.status(404).json({ message: '해당 로봇을 찾을 수 없습니다.' });
    }

    // active 상태를 0에서 1 또는 1에서 0으로 토글
    robot.active = robot.active === 1 ? 0 : 1;

    // 변경된 active 상태 저장
    await robot.save();

    res.status(200).json({ message: '로봇 active 상태가 업데이트되었습니다.', active: robot.active });
  } catch (error) {
    console.error('로봇 active 상태 업데이트 중 오류 발생:', error);
    res.status(500).json({ message: '로봇 active 상태 업데이트 실패', error: error.message });
  }
};
exports.getActiveRobots = async (req, res) => {
  try {
    // 요청한 사용자 ID와 Active 상태가 1이며, 이름이 빈 문자열이 아닌 로봇만 조회
    const robots = await Robot.find({ userId: req.user.id, active: 1, name: { $ne: "0" } });
    res.json(robots);
  } catch (error) {
    console.error('Active 상태인 로봇 조회 중 오류 발생:', error);
    res.status(500).json({ message: 'Active 상태인 로봇 조회 중 오류가 발생했습니다.', error: error.message });
  }
};
exports.startRosbridgeWebsocket = (req, res) => {
  exec('roslaunch rosbridge_server rosbridge_websocket.launch', (error, stdout, stderr) => {
    if (error) {
      console.error(`Error starting rosbridge: ${error.message}`);
      return res.status(500).send('rosbridge 시작에 실패했습니다.');
    }
    res.send('rosbridge가 성공적으로 시작되었습니다.');
  });
};

exports.stopRosbridgeWebsocket = (req, res) => {
  exec('pkill -f rosbridge_websocket', (error, stdout, stderr) => {
    if (error) {
      console.error(`Error stopping rosbridge: ${error.message}`);
      return res.status(500).send('rosbridge 중지에 실패했습니다.');
    }
    res.send('rosbridge가 성공적으로 중지되었습니다.');
  });
};
exports.assignNextWorkflow = async (req, res) => {
  const { robotId } = req.body;

  // robotId 유효성 검사
  if (!robotId || !mongoose.Types.ObjectId.isValid(robotId)) {
    return res.status(400).json({ message: '유효한 robotId가 필요합니다.' });
  }

  try {
    // 로봇 문서 찾기
    const robot = await Robot.findById(robotId);
    if (!robot) {
      return res.status(404).json({ message: '로봇을 찾을 수 없습니다.' });
    }

    // currentWorkflow가 비어 있는지 확인
    if (robot.currentWorkflow.node !== null) {
      return res.status(400).json({ message: '현재 워크플로우가 비어 있지 않습니다.' });
    }

    // 워크플로우 큐 찾기
    const workflowQueue = await WorkflowQueue.findOne({ robotId });
    if (!workflowQueue || workflowQueue.workflows.length === 0) {
      return res.status(404).json({ message: '워크플로우 큐가 비어 있습니다.' });
    }

    // 첫 번째 워크플로우 가져오기
    const nextWorkflow = workflowQueue.workflows[0];

    // 로봇의 currentWorkflow에 할당
    robot.currentWorkflow = {
      node: nextWorkflow.node,
      step: nextWorkflow.step,
      x: nextWorkflow.x,
      y: nextWorkflow.y,
      status: 'In Progress'
    };
    await robot.save();

    // 큐에서 워크플로우 제거
    workflowQueue.workflows.shift();
    await workflowQueue.save();

    res.status(200).json({ message: '워크플로우가 성공적으로 할당되었습니다.', currentWorkflow: robot.currentWorkflow });
  } catch (error) {
    console.error('워크플로우 할당 중 오류 발생:', error);
    res.status(500).json({ message: '워크플로우 할당 중 오류가 발생했습니다.', error: error.message });
  }
};
let nh; // 전역 네임스페이스 핸들러
const publishers = {}; // 퍼블리셔 캐시

// 애플리케이션 초기화 시 ROS 노드 초기화
(async () => {
  try {
    await rosnodejs.initNode('/moveRobotToTask_node', { anonymous: true });
    nh = rosnodejs.nh; // 전역 네임스페이스 핸들러 저장
    console.log('ROS 노드 초기화 성공');
  } catch (error) {
    console.error('ROS 노드 초기화 실패:', error);
    process.exit(1); // 노드 초기화 실패 시 종료
  }
})();

// 퍼블리셔 생성 또는 캐시에서 가져오기
function getOrCreatePublisher(robotName) {
  const topicName = `/${robotName}/task_path`;
  if (!publishers[topicName]) {
    publishers[topicName] = nh.advertise(topicName, TaskPath);
    console.log(`퍼블리셔 생성: ${topicName}`);
  }
  return publishers[topicName];
}

const MAP_SERVER_URL = process.env.MAP_SERVER_URL || 'http://172.30.1.33:5557/map/monitored/nodes';
const SHORT_PATH_API_URL = process.env.SHORT_PATH_API_URL || 'http://172.30.1.33:5557/map/shortPaths/find';

const TaskPath = rosnodejs.require('custom_msgs').msg.TaskPath; // TaskPath.msg는 ROS에서 정의한 메시지 타입

exports.moveRobotToTask = async (req, res) => {
  const { robotId } = req.body;

  if (!robotId) {
    console.error('moveRobotToTask: robotId가 요청 본문에 없습니다.');
    return res.status(400).json({ error: 'robotId가 필요합니다.' });
  }

  console.log(`moveRobotToTask: 요청된 robotId = ${robotId}`);

  try {
    // 1. 로봇 정보 가져오기
    const robot = await Robot.findById(robotId);
    if (!robot) {
      console.error(`moveRobotToTask: robotId ${robotId}에 해당하는 로봇을 찾을 수 없습니다.`);
      return res.status(404).json({ error: '로봇을 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 로봇 정보 가져오기 성공. 로봇 이름: ${robot.name}`);

    // 2. 맵 서버에서 노드 데이터 가져오기
    console.log(`moveRobotToTask: MAP_SERVER_URL (${MAP_SERVER_URL})에 GET 요청을 보냅니다.`);
    const response = await axios.get(MAP_SERVER_URL);
    const { nodes } = response.data;

    if (!nodes || nodes.length === 0) {
      console.error('moveRobotToTask: 맵에 노드가 없습니다.');
      return res.status(404).json({ error: '맵에 노드가 없습니다.' });
    }

    console.log(`moveRobotToTask: 맵 서버에서 받은 노드 개수 = ${nodes.length}`);

    // 3. 로봇의 현재 위치에서 가장 가까운 노드 찾기 (출발 노드)
    const { x: currentX, y: currentY } = robot.location;
    let startNode = null;
    let minDistance = Infinity;

    nodes.forEach((node) => {
      const distance = Math.sqrt(
        Math.pow(node.x - currentX, 2) + Math.pow(node.y - currentY, 2)
      );
      if (distance < minDistance) {
        minDistance = distance;
        startNode = node;
      }
    });

    if (!startNode) {
      console.error('moveRobotToTask: 출발 노드를 찾을 수 없습니다.');
      return res.status(400).json({ error: '출발 노드를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 출발 노드 찾기 성공 - 이름: ${startNode.name}, x: ${startNode.x}, y: ${startNode.y}`);

    // 4. 로봇의 currentWorkflow에서 목적지 노드 정보 가져오기
    const { currentWorkflow } = robot;

    if (!currentWorkflow || !currentWorkflow.node) {
      console.error('moveRobotToTask: currentWorkflow 정보가 부족합니다.');
      return res.status(400).json({ error: 'currentWorkflow 정보가 부족합니다.' });
    }

    const { node: nodeName, step, x: workflowX, y: workflowY } = currentWorkflow;

    const endNode = nodes.find(
      (node) =>
        node.name === nodeName &&
        node.x === workflowX &&
        node.y === workflowY
    );

    if (!endNode) {
      console.error(`moveRobotToTask: 목적지 노드 (${nodeName}, x: ${workflowX}, y: ${workflowY})를 찾을 수 없습니다.`);
      return res.status(400).json({ error: '도착 노드를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 도착 노드 찾기 성공 - 이름: ${endNode.name}, x: ${endNode.x}, y: ${endNode.y}`);

    // 5. ShortPath 경로 API 호출로 경로 조회
    console.log('moveRobotToTask: ShortPath API 호출 중...');
    const shortPathResponse = await axios.get(SHORT_PATH_API_URL, {
      params: {
        startNodeName: startNode.name,
        startX: startNode.x,
        startY: startNode.y,
        endNodeName: endNode.name,
        endX: endNode.x,
        endY: endNode.y,
      },
    });

    const { path, totalDistance } = shortPathResponse.data;

    if (!path || path.length === 0) {
      console.error('moveRobotToTask: ShortPath API에서 저장된 경로를 찾을 수 없습니다.');
      return res.status(404).json({ error: '저장된 경로를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 경로 조회 성공 - 총 거리: ${totalDistance}, 경로: ${JSON.stringify(path)}`);

    // 6. ROS 메시지 발행
    console.log(`moveRobotToTask: ${robot.name} 로봇을 위한 ROS 메시지 발행 중...`);

    const pub = getOrCreatePublisher(robot.name);

    const taskPathMsg = new TaskPath();
    taskPathMsg.robotName = robot.name;
    taskPathMsg.currentWorkflowStep = step;
    taskPathMsg.path = path.map((point) => ({
      x: point.x,
      y: point.y,
      z: 0,
    }));

    pub.publish(taskPathMsg);

    console.log(`moveRobotToTask: ROS 메시지를 ${pub.getTopic()} 토픽으로 성공적으로 발행했습니다.`);

    // 7. 좌표값 및 step 반환
    const responseData = {
      robotName: robot.name,
      currentWorkflowStep: step, // 현재 작업 단계 반환
      path,
    };

    // 콘솔 로그로 반환 데이터를 출력
    console.log('moveRobotToTask: 최종 반환 데이터:', JSON.stringify(responseData, null, 2));

    res.status(200).json(responseData);
  } catch (error) {
    console.error('moveRobotToTask: 좌표 추출 중 오류:', error);
    res.status(500).json({ error: '좌표 추출 중 서버 오류 발생', details: error.message });
  }
};
/*
exports.moveRobotToTask = async (req, res) => {
  const { robotId } = req.body;

  if (!robotId) {
    console.error('moveRobotToTask: robotId가 요청 본문에 없습니다.');
    return res.status(400).json({ error: 'robotId가 필요합니다.' });
  }

  console.log(`moveRobotToTask: 요청된 robotId = ${robotId}`);

  try {
    // 1. 로봇 정보 가져오기
    const robot = await Robot.findById(robotId);
    if (!robot) {
      console.error(`moveRobotToTask: robotId ${robotId}에 해당하는 로봇을 찾을 수 없습니다.`);
      return res.status(404).json({ error: '로봇을 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 로봇 정보 가져오기 성공. 로봇 이름: ${robot.name}`);

    // 2. 맵 서버에서 노드 데이터 가져오기
    console.log(`moveRobotToTask: MAP_SERVER_URL (${MAP_SERVER_URL})에 GET 요청을 보냅니다.`);
    const response = await axios.get(MAP_SERVER_URL);
    const { nodes } = response.data;

    if (!nodes || nodes.length === 0) {
      console.error('moveRobotToTask: 맵에 노드가 없습니다.');
      return res.status(404).json({ error: '맵에 노드가 없습니다.' });
    }

    console.log(`moveRobotToTask: 맵 서버에서 받은 노드 개수 = ${nodes.length}`);

    // 3. 로봇의 현재 위치에서 가장 가까운 노드 찾기 (출발 노드)
    const { x: currentX, y: currentY } = robot.location;
    let startNode = null;
    let minDistance = Infinity;

    nodes.forEach((node) => {
      const distance = Math.sqrt(
        Math.pow(node.x - currentX, 2) + Math.pow(node.y - currentY, 2)
      );
      if (distance < minDistance) {
        minDistance = distance;
        startNode = node;
      }
    });

    if (!startNode) {
      console.error('moveRobotToTask: 출발 노드를 찾을 수 없습니다.');
      return res.status(400).json({ error: '출발 노드를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 출발 노드 찾기 성공 - 이름: ${startNode.name}, x: ${startNode.x}, y: ${startNode.y}`);

    // 4. 로봇의 currentWorkflow에서 목적지 노드 정보 가져오기
    const { currentWorkflow } = robot;

    if (!currentWorkflow || !currentWorkflow.node) {
      console.error('moveRobotToTask: currentWorkflow 정보가 부족합니다.');
      return res.status(400).json({ error: 'currentWorkflow 정보가 부족합니다.' });
    }

    const { node: nodeName, step, x: workflowX, y: workflowY } = currentWorkflow;

    const endNode = nodes.find(
      (node) =>
        node.name === nodeName &&
        node.x === workflowX &&
        node.y === workflowY
    );

    if (!endNode) {
      console.error(`moveRobotToTask: 목적지 노드 (${nodeName}, x: ${workflowX}, y: ${workflowY})를 찾을 수 없습니다.`);
      return res.status(400).json({ error: '도착 노드를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 도착 노드 찾기 성공 - 이름: ${endNode.name}, x: ${endNode.x}, y: ${endNode.y}`);

    // 5. ShortPath 경로 API 호출로 경로 조회
    console.log('moveRobotToTask: ShortPath API 호출 중...');
    const shortPathResponse = await axios.get(SHORT_PATH_API_URL, {
      params: {
        startNodeName: startNode.name,
        startX: startNode.x,
        startY: startNode.y,
        endNodeName: endNode.name,
        endX: endNode.x,
        endY: endNode.y,
      },
    });

    const { path, totalDistance } = shortPathResponse.data;

    if (!path || path.length === 0) {
      console.error('moveRobotToTask: ShortPath API에서 저장된 경로를 찾을 수 없습니다.');
      return res.status(404).json({ error: '저장된 경로를 찾을 수 없습니다.' });
    }

    console.log(`moveRobotToTask: 경로 조회 성공 - 총 거리: ${totalDistance}, 경로: ${JSON.stringify(path)}`);

    // 6. 좌표값 및 step 반환
    const responseData = {
      robotName: robot.name,
      currentWorkflowStep: step, // 현재 작업 단계 반환
      path,
    };

    // 콘솔 로그로 반환 데이터를 출력
    console.log('moveRobotToTask: 최종 반환 데이터:', JSON.stringify(responseData, null, 2));

    res.status(200).json(responseData);
  } catch (error) {
    console.error('moveRobotToTask: 좌표 추출 중 오류:', error);
    res.status(500).json({ error: '좌표 추출 중 서버 오류 발생', details: error.message });
  }
};
*/
exports.clearCurrentWorkflow = async (req, res) => {
  try {
      const { id } = req.params;

      // 로봇을 찾고 currentWorkflow를 기본값으로 업데이트
      const updatedRobot = await Robot.findByIdAndUpdate(
          id,
          {
              $set: {
                  currentWorkflow: {
                      node: null,
                      step: null,
                      x: 0,
                      y: 0,
                      status: 'Pending' // 기본값
                  }
              }
          },
          { new: true } // 업데이트된 문서를 반환
      );

      // 로봇이 존재하지 않을 경우
      if (!updatedRobot) {
          return res.status(404).json({ message: 'Robot not found' });
      }

      res.status(200).json({
          message: 'Current workflow cleared successfully',
          robot: updatedRobot
      });
  } catch (error) {
      console.error('Error clearing current workflow:', error);
      res.status(500).json({ message: 'An error occurred', error: error.message });
  }
};