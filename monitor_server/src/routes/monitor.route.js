const WebSocket = require('ws');
const express = require('express');
const router = express.Router();
const monitorController = require('../controllers/monitor.controller');
const expressWs = require('express-ws');

expressWs(router);

router.ws('/', (ws, req) => {
  monitorController.monitorUpdates(ws, req);
});

module.exports = router;
