const rosnodejs = require('rosnodejs');
const TaskLog = require('../models/tasklog.model');  // TaskLog 스키마 가져오기

// ROS 서브스크라이버 설정
async function initRosSubscriber() {
  const rosNode = await rosnodejs.initNode('/task_status_listener_node');

  rosNode.subscribe('/robot/task_status', 'std_msgs/String', async (data) => {
    const taskStatus = JSON.parse(data.data);  // 로봇으로부터 받은 상태 정보
    const { taskId, status } = taskStatus;    // taskId와 status를 이용해 MongoDB 업데이트

    console.log('Received task status update from ROS:', taskStatus);

    try {
      // MongoDB에서 taskId를 기준으로 작업 로그를 찾고 상태를 업데이트
      const taskLog = await TaskLog.findById(taskId);
      if (taskLog) {
        taskLog.status = status;  // 상태 업데이트
        await taskLog.save();     // MongoDB에 저장
        console.log(`TaskLog ${taskId} updated to status: ${status}`);
      } else {
        console.log(`TaskLog with ID ${taskId} not found.`);
      }
    } catch (error) {
      console.error(`Failed to update task log status for ${taskId}:`, error);
    }
  });
}

module.exports = { initRosSubscriber };