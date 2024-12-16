const rosnodejs = require('rosnodejs');

// ROS 퍼블리셔 설정
async function initRosPublisher() {
  const rosNode = await rosnodejs.initNode('/task_assigner_node');
  const pub = rosNode.advertise('/robot/task', 'std_msgs/String');  // '/robot/task' 토픽에 퍼블리싱
  return pub;
}

// 작업 로그를 ROS로 퍼블리싱하는 함수
async function publishTaskLogToRos(taskLog) {
  try {
    const pub = await initRosPublisher();

    // 퍼블리시할 작업 로그 정보
    const taskLogMsg = { data: JSON.stringify(taskLog) };  // 작업 로그 정보를 문자열로 변환
    pub.publish(taskLogMsg);  // ROS 토픽에 퍼블리시

    console.log(`TaskLog published to ROS: ${taskLog.taskId}`);
  } catch (error) {
    console.error('Failed to publish task log to ROS:', error);
  }
}

module.exports = { publishTaskLogToRos };
