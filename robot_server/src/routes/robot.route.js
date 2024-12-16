const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const workflowQueueController = require('../controllers/workflowQueue.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/robots', auth, robotController.getRobots);
router.post('/register', auth, robotController.registerRobot);
router.put('/update/:id', auth, robotController.updateRobot);
router.post('/send_map', auth, robotController.sendMapToRobots);
/*router.put('/unregister/:id', auth, robotController.unregisterRobot);*/
router.delete('/robots/:id', auth, robotController.unregisterRobot);
router.put('/updatePosition', robotController.updateRobotPosition);
router.post('/bringup/:robotIp', auth, robotController.bringupRobot);
router.put('/battery/update', robotController.updateRobotBattery);
router.put('/update_status', robotController.updateRobotStatus);
/*router.put('/update_speed', robotController.updateRobotSpeed);*/
router.put('/toggleActive/:id', auth, robotController.toggleRobotActive);
router.get('/robots/active', auth, robotController.getActiveRobots);

router.post('/addWorkflow', workflowQueueController.addWorkflowToRobotQueue);
router.get('/robot/:robotId', workflowQueueController.getWorkflowQueueByRobotId);
router.delete('/:robotId/workflows/:workflowId', workflowQueueController.removeWorkflowByIdFromRobotQueue);
router.post('/assignNextWorkflow', auth, robotController.assignNextWorkflow);


router.post('/rosbridge/start', robotController.startRosbridgeWebsocket);
router.post('/rosbridge/stop',  robotController.stopRosbridgeWebsocket);

router.post('/move-to-task', auth, robotController.moveRobotToTask);
router.put('/robots/:id/clear-current-workflow', robotController.clearCurrentWorkflow);

module.exports = router;
