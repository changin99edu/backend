const mongoose = require('mongoose');
const Task = require('../models/task.model');
const TaskLog = require('../models/tasklog.model');

exports.createTask = async (req, res) => {
  try {
    const { name, description, taskType, workflow, mapId } = req.body;

    // 필수 값 확인 (작업 이름과 mapId 필수)
    if (!name || !mapId) {
      return res.status(400).json({ error: '작업 이름과 맵 ID는 필수 항목입니다.' });
    }

    // 새로운 작업 생성
    const newTask = new Task({
      name,
      description, // 작업 설명 (선택)
      taskType,    // 작업 유형 (선택)
      workflow,    // 작업 플로우 (선택)
      mapId,       // 모니터링 중인 맵 ID 추가
    });

    // 데이터베이스에 저장
    const savedTask = await newTask.save();
    res.status(201).json(savedTask);
  } catch (error) {
    console.error(error);
    res.status(500).json({ error: '작업을 생성하는 중 오류가 발생했습니다.' });
  }
};


// 작업 조회 API (전체 조회)
/*exports.getTasks = async (req, res) => {
  try {
    // 작업 이름이 '0'이 아닌 작업만 조회
    const tasks = await Task.find({ name: { $ne: '0' } });
    res.status(200).json(tasks);
  } catch (error) {
    console.error(error);
    res.status(500).json({ error: '작업을 조회하는 중 오류가 발생했습니다.' });
  }
};*/
exports.getTasks = async (req, res) => {
  try {
    const { mapId } = req.query; // 클라이언트에서 mapID 쿼리 파라미터를 받아옴
    if (!mapId) {
      return res.status(400).json({ error: 'mapID가 필요합니다.' });
    }

    // mapID와 일치하는 작업 중 name이 '0'이 아닌 것만 조회
    const tasks = await Task.find({ mapId, name: { $ne: '0' } });

    // 각 작업의 workflow 배열을 필터링하여 node가 "0"이 아닌 항목만 포함
    const filteredTasks = tasks.map(task => {
      const filteredWorkflow = task.workflow.filter(step => step.node !== '0');
      return { ...task.toObject(), workflow: filteredWorkflow };
    });

    res.status(200).json(filteredTasks);
  } catch (error) {
    console.error(error);
    res.status(500).json({ error: '작업을 조회하는 중 오류가 발생했습니다.' });
  }
};

exports.updateTask = async (req, res) => {
  try {
    const { id } = req.params; // 업데이트할 작업 ID
    const { name, description, taskType, status, workflow } = req.body;

    // 작업 업데이트
    const updatedTask = await Task.findByIdAndUpdate(
      id,
      { name, description, taskType, status, workflow },
      { new: true, runValidators: true } // 새 값 반환 및 유효성 검사 실행
    );

    // 작업을 찾지 못한 경우
    if (!updatedTask) {
      return res.status(404).json({ error: '작업을 찾을 수 없습니다.' });
    }

    // 업데이트된 작업 반환
    res.status(200).json(updatedTask);
  } catch (error) {
    console.error('작업 업데이트 중 오류 발생:', error);
    res.status(500).json({ error: '작업 업데이트 중 오류가 발생했습니다.' });
  }
};
exports.softDeleteTask = async (req, res) => {
  try {
    const { id } = req.params; // 삭제할 작업 ID

    // 작업 이름을 '0'으로 수정
    const deletedTask = await Task.findByIdAndUpdate(
      id,
      { name: '0' }, // 작업 이름을 '0'으로 설정
      { new: true } // 업데이트된 작업 반환
    );

    if (!deletedTask) {
      return res.status(404).json({ error: '작업을 찾을 수 없습니다.' });
    }

    res.status(200).json({ message: '작업이 삭제되었습니다.', task: deletedTask });
  } catch (error) {
    console.error('작업 삭제 중 오류 발생:', error);
    res.status(500).json({ error: '작업 삭제 중 오류가 발생했습니다.' });
  }
};
exports.getTaskById = async (req, res) => {
  const { id } = req.params;

  try {
      const task = await Task.findById(id);

      if (!task) {
          return res.status(404).json({ message: '작업을 찾을 수 없습니다.' });
      }

      // node 이름이 "0"이 아닌 workflow만 반환
      task.workflow = task.workflow.filter(step => step.node !== "0");
      res.json(task);
  } catch (error) {
      console.error('작업 조회 중 오류:', error);
      res.status(500).json({ message: '작업 조회 중 오류가 발생했습니다.' });
  }
};
exports.updateWorkflowNodeToZero = async (req, res) => {
  const { taskId, workflowId } = req.params;

  try {
      // 해당 작업을 찾고 워크플로우의 노드 이름을 "0"으로 변경
      const task = await Task.findOneAndUpdate(
          { _id: taskId, "workflow._id": workflowId },
          { $set: { "workflow.$.node": "0" } },
          { new: true }
      );

      if (!task) {
          return res.status(404).json({ message: '작업 또는 워크플로우를 찾을 수 없습니다.' });
      }

      res.status(200).json({ message: '워크플로우 노드 이름이 "0"으로 업데이트되었습니다.', task });
  } catch (error) {
      console.error('워크플로우 노드 이름 업데이트 중 오류 발생:', error);
      res.status(500).json({ message: '서버 오류가 발생했습니다.' });
  }
};


exports.createTaskLog = async (req, res) => {
  try {
    const { robotName, robotIp, nodeName, step, status } = req.body;

    // 필수 필드 체크
    if (!robotName || !robotIp || !nodeName || !step) {
      return res.status(400).json({ message: '모든 필수 정보를 입력해주세요.' });
    }

    // 새로운 TaskLog 생성
    const newTaskLog = new TaskLog({
      robotName,
      robotIp,
      nodeName,
      step,
      status, // 선택 사항, 기본값은 "In Progress"
    });

    // 데이터 저장
    await newTaskLog.save();

    res.status(201).json({
      message: '작업 로그가 성공적으로 생성되었습니다.',
      taskLog: newTaskLog,
    });
  } catch (error) {
    console.error('TaskLog 생성 오류:', error);
    res.status(500).json({ message: '작업 로그 생성 중 오류가 발생했습니다.', error });
  }
};

exports.getAllTaskLogs = async (req, res) => {
  try {
    const taskLogs = await TaskLog.find(); // 모든 작업 기록 조회
    res.status(200).json({
      message: '작업 기록 조회 성공',
      taskLogs,
    });
  } catch (error) {
    console.error('TaskLog 조회 오류:', error);
    res.status(500).json({
      message: '작업 기록 조회 중 오류가 발생했습니다.',
      error,
    });
  }
};