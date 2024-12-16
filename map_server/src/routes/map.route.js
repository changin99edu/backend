const express = require('express');
const router = express.Router();
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/maps', auth, mapController.getMaps);
router.post('/upload', auth, mapController.uploadMap);
router.put('/update/:id', auth, mapController.updateMap);
router.get('/monitored', auth, mapController.getMonitoredMap);
router.get('/file/:id', auth, mapController.getMonitoredMapFile);
router.get('/metadata/:id', auth, mapController.getMonitoredMapMetadata);
router.get('/monitored/file', auth, mapController.getCurrentMonitoredMapFile);
router.get('/monitored/metadata', auth, mapController.getCurrentMonitoredMapMetadata);
router.put('/delete/:id', auth, mapController.deleteMap);

router.post('/nodes', mapController.createNode);  // 노드 생성
router.get('/nodes/:mapId', mapController.getNodesByMap); // 특정 맵의 노드 조회
router.get('/nodes', mapController.getAllNodes);  // 노드 목록 조회
router.delete('/node/:nodeId', auth, mapController.deleteNodeCompletely);

router.post('/steps', auth, mapController.addStep); // 작업 추가
router.get('/steps', auth, mapController.getSteps); // 작업 목록 조회
router.put('/remove-step', mapController.removeStep);

router.post('/node/add-step', auth, mapController.addStepToNode); // Step을 Node에 추가
router.post('/node/connect', mapController.connectNodes);
router.delete('/:nodeId/connections/:connectionNodeId', mapController.removeConnectionFromNode);

router.post('/no-go-zones', mapController.createNoGoZone);
router.get('/no-go-zones/map/:mapId', mapController.getNoGoZonesByMap);
router.delete('/no-go-zones/:zoneId', mapController.deleteNoGoZone);

router.post('/calculate-paths', mapController.calculateAndStoreShortPaths);
router.delete('/shortpaths/map/:mapId', auth, mapController.deleteShortPathsByMapId);

router.get('/monitored/nodes', mapController.getMonitoredMapNodes);

router.get('/shortPaths/find', mapController.getPathByNodes);


module.exports = router;