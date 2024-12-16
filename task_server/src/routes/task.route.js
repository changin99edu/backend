const express = require('express');
const router = express.Router();
const taskController = require('../controllers/task.controller');
const auth = require('../middlewares/auth').authenticate;

// 작업 생성 라우트 (auth 미들웨어 적용)
router.post('/tasks', auth, taskController.createTask);

// 작업 전체 조회 라우트 (auth 미들웨어 적용)
router.get('/tasks', auth, taskController.getTasks);
router.put('/tasks/:id', auth, taskController.updateTask);
router.put('/tasks/:id/delete', auth, taskController.softDeleteTask);
router.get('/tasks/:id', auth, taskController.getTaskById);
router.put('/tasks/:taskId/workflow/:workflowId/nodeToZero', auth, taskController.updateWorkflowNodeToZero);

router.post('/task-logs', taskController.createTaskLog);
router.get('/task-logs', auth, taskController.getAllTaskLogs);

module.exports = router;
