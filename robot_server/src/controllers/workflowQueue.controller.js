const mongoose = require('mongoose');
const WorkflowQueue = require('../models/workflowQueue.model'); 
const Robot = require('../models/robot.model');


exports.addWorkflowToRobotQueue = async (req, res) => {
  const { robotId, node, step, x, y } = req.body;

  // 필수 값 검증
  if (!robotId || !node || !step || x === undefined || y === undefined) {
    return res.status(400).json({ message: '필수 필드 (robotId, node, step, x, y)가 필요합니다.' });
  }

  try {
    // 해당 로봇의 큐에 새로운 워크플로우 추가
    const updatedQueue = await WorkflowQueue.findOneAndUpdate(
      { robotId },
      { 
        $push: { 
          workflows: { 
            node, 
            step, 
            x, 
            y, 
            status: 'Pending',
            assignedAt: new Date()
          } 
        } 
      },
      { upsert: true, new: true } // 큐가 없으면 새로 생성
    );

    res.status(200).json({ message: '워크플로우가 성공적으로 추가되었습니다.', data: updatedQueue });
  } catch (error) {
    console.error('로봇 큐에 워크플로우 추가 중 오류 발생:', error);
    res.status(500).json({ message: '워크플로우 추가 중 오류가 발생했습니다.', error: error.message });
  }
};

exports.getWorkflowQueueByRobotId = async (req, res) => {
  const { robotId } = req.params; // URL 파라미터로 robotId 가져오기

  // Validate robotId format
  if (!mongoose.Types.ObjectId.isValid(robotId)) {
    return res.status(400).json({ message: '유효한 robotId가 아닙니다.' });
  }

  try {
    // Fetch the WorkflowQueue for the given robotId
    const workflowQueue = await WorkflowQueue.findOne({ robotId });

    if (!workflowQueue) {
      return res.status(404).json({ message: '지정된 robotId에 대한 워크플로우 큐를 찾을 수 없습니다.' });
    }

    // Fetch the Robot document to get currentWorkflow
    const robot = await Robot.findById(robotId).select('currentWorkflow');

    if (!robot) {
      return res.status(404).json({ message: '로봇을 찾을 수 없습니다.' });
    }

    const combinedWorkflows = [];

    // Check if currentWorkflow exists and is valid
    if (
      robot.currentWorkflow &&
      robot.currentWorkflow.node &&
      robot.currentWorkflow.step
    ) {
      combinedWorkflows.push({
        _id: 'currentWorkflow', // Assign a unique identifier or a specific flag
        node: robot.currentWorkflow.node,
        step: robot.currentWorkflow.step,
        x: robot.currentWorkflow.x,
        y: robot.currentWorkflow.y,
        status: robot.currentWorkflow.status,
        assignedAt: robot.currentWorkflow.assignedAt || null, // Include if available
        isCurrent: true, // Flag to identify currentWorkflow in the frontend
      });
    }

    // Append the remaining workflows from the queue
    combinedWorkflows.push(...workflowQueue.workflows);

    res.status(200).json({ workflows: combinedWorkflows });
  } catch (error) {
    console.error('워크플로우 큐를 가져오는 중 오류 발생:', error);
    res.status(500).json({ message: '워크플로우 큐를 가져오는 중 오류가 발생했습니다.', error: error.message });
  }
};

exports.removeWorkflowByIdFromRobotQueue = async (req, res) => {
  const { robotId, workflowId } = req.params; // URL 파라미터로 robotId와 workflowId 가져오기

  // robotId 및 workflowId 유효성 검사
  if (!mongoose.Types.ObjectId.isValid(robotId)) {
    return res.status(400).json({ message: '유효한 robotId가 아닙니다.' });
  }

  if (!mongoose.Types.ObjectId.isValid(workflowId)) {
    return res.status(400).json({ message: '유효한 workflowId가 아닙니다.' });
  }

  try {
    // 워크플로우 _id로 해당 워크플로우 삭제
    const updatedQueue = await WorkflowQueue.findOneAndUpdate(
      { robotId },
      { $pull: { workflows: { _id: workflowId } } },
      { new: true }
    );

    if (!updatedQueue) {
      return res.status(404).json({ message: '지정된 robotId에 대한 워크플로우 큐를 찾을 수 없습니다.' });
    }

    // 삭제된 후에도 workflows 배열을 확인하여 삭제 여부 확인
    const workflowExists = updatedQueue.workflows.id(workflowId);
    if (workflowExists) {
      return res.status(500).json({ message: '워크플로우 삭제에 실패했습니다.' });
    }

    res.status(200).json({ message: '워크플로우가 성공적으로 삭제되었습니다.', data: updatedQueue });
  } catch (error) {
    console.error('워크플로우 삭제 중 오류 발생:', error);
    res.status(500).json({ message: '워크플로우 삭제 중 오류가 발생했습니다.', error: error.message });
  }
};
