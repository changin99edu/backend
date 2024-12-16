const mongoose = require('mongoose');
const { GridFSBucket } = require('mongodb');
const Map = require('../models/map.model');
const MapNode = require('../models/mapNode.model');
const StepList = require('../models/stepList.model'); 
const NoGoZone = require('../models/noGoZone.model');
const multer = require('multer');
const sharp = require('sharp'); // sharp 모듈 추가
const path = require('path');
const ShortPath = require('../models/shortPath.model');
const findShortestPath = require('../findShortestPath');

// Mongoose 설정
mongoose.set('useFindAndModify', false);

// GridFS 초기화
let gfs;
mongoose.connection.once('open', () => {
  gfs = new GridFSBucket(mongoose.connection.db, { bucketName: 'maps' });
});

// Multer 설정
const storage = multer.memoryStorage();
const upload = multer({ storage });

// 자신이 보유한 맵 조회 (이름이 '0'인 맵 제외)
exports.getMaps = async (req, res) => {
  try {
    const maps = await Map.find({ userId: req.user.id, name: { $ne: '0' } });
    res.json(maps);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch maps');
  }
};

// 맵 업로드
exports.uploadMap = [
  upload.fields([{ name: 'file', maxCount: 1 }, { name: 'yaml', maxCount: 1 }]), // 파일과 메타데이터 파일 모두 업로드
  async (req, res) => {
    try {
      const { name, description } = req.body;
      const mapFile = req.files.file[0];
      const yamlFile = req.files.yaml[0];

      if (!mapFile || !yamlFile) {
        return res.status(400).json({ message: 'Map file and YAML file are required' });
      }

      // 맵 파일 업로드
      const uploadStream = gfs.openUploadStream(mapFile.originalname, {
        contentType: mapFile.mimetype
      });

      let mapFileId;
      const uploadPromise = new Promise((resolve, reject) => {
        uploadStream.on('finish', (uploadedFile) => {
          mapFileId = uploadedFile._id;
          resolve();
        });
        uploadStream.on('error', reject);
        uploadStream.end(mapFile.buffer);
      });

      await uploadPromise;

      // YAML 파일 업로드
      const yamlUploadStream = gfs.openUploadStream(yamlFile.originalname, {
        contentType: yamlFile.mimetype
      });

      let yamlFileId;
      const yamlUploadPromise = new Promise((resolve, reject) => {
        yamlUploadStream.on('finish', (uploadedFile) => {
          yamlFileId = uploadedFile._id;
          resolve();
        });
        yamlUploadStream.on('error', reject);
        yamlUploadStream.end(yamlFile.buffer);
      });

      await yamlUploadPromise;

      const map = new Map({
        name,
        description,
        FileId: mapFileId,
        filename: mapFile.originalname,
        yamlFileId: yamlFileId, // YAML 파일 ID 저장
        userId: req.user.id
      });

      await map.save();
      res.status(201).json({ message: 'Map uploaded successfully', map });
    } catch (error) {
      console.error('Error uploading map:', error);
      res.status(500).json({ message: 'Error uploading map', error: error.message });
    }
  }
];

// 맵 업데이트
exports.updateMap = async (req, res) => {
  try {
    const { name, description, isMonitored } = req.body;
    const { id } = req.params;

    // 모든 맵의 isMonitored 필드를 false로 설정
    if (isMonitored) {
      await Map.updateMany({ userId: req.user.id }, { isMonitored: false });
    }

    // 선택된 맵의 isMonitored 필드를 true로 설정
    const map = await Map.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { name, description, isMonitored },
      { new: true, runValidators: true }
    );

    if (!map) {
      return res.status(404).json({ message: 'Map not found or not authorized' });
    }

    res.status(200).json({ message: 'Map updated successfully', map });
  } catch (error) {
    console.error('Error updating map:', error);
    res.status(500).json({ message: 'Error updating map', error: error.message });
  }
};

// 선택된 맵 조회
exports.getMonitoredMap = async (req, res) => {
  try {
    const monitoredMap = await Map.findOne({ userId: req.user.id, isMonitored: true, name: { $ne: '0' } });
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }
    res.status(200).json(monitoredMap);
  } catch (error) {
    console.error('Error fetching monitored map:', error);
    res.status(500).json({ message: 'Error fetching monitored map', error: error.message });
  }
};

// 선택된 맵 파일 다운로드
exports.getMonitoredMapFile = async (req, res) => {
  try {
    const monitoredMap = await Map.findById(req.params.id);
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }
    
    const fileId = monitoredMap.FileId;
    const downloadStream = gfs.openDownloadStream(fileId);

    downloadStream.on('error', (error) => {
      console.error('Error downloading map file:', error);
      res.status(500).json({ message: 'Error downloading map file', error: error.message });
    });

    downloadStream.pipe(res);
  } catch (error) {
    console.error('Error fetching monitored map file:', error);
    res.status(500).json({ message: 'Error fetching monitored map file', error: error.message });
  }
};

// 선택된 메타데이터 파일 다운로드
exports.getMonitoredMapMetadata = async (req, res) => {
  try {
    const monitoredMap = await Map.findById(req.params.id);
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map metadata found' });
    }

    const yamlFileId = monitoredMap.yamlFileId;
    const downloadStream = gfs.openDownloadStream(yamlFileId);

    downloadStream.on('error', (error) => {
      console.error('Error downloading map metadata file:', error);
      res.status(500).json({ message: 'Error downloading map metadata file', error: error.message });
    });

    downloadStream.pipe(res);
  } catch (error) {
    console.error('Error fetching monitored map metadata file:', error);
    res.status(500).json({ message: 'Error fetching monitored map metadata file', error: error.message });
  }
};

// 현재 모니터링 중인 맵 파일 다운로드 (PGM 파일을 PNG로 변환)
exports.getCurrentMonitoredMapFile = async (req, res) => {
  try {
    const monitoredMap = await Map.findOne({ userId: req.user.id, isMonitored: true });
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }

    const fileId = monitoredMap.FileId;
    const downloadStream = gfs.openDownloadStream(fileId);

    let chunks = [];

    downloadStream.on('data', (chunk) => {
      chunks.push(chunk);
    });

    downloadStream.on('end', async () => {
      const buffer = Buffer.concat(chunks);
      const extension = monitoredMap.filename.split('.').pop().toLowerCase();

      // PGM 파일일 경우 PNG로 변환
      if (extension === 'pgm') {
        try {
          const pngBuffer = await sharp(buffer).png().toBuffer(); // PGM을 PNG로 변환
          res.set('Content-Type', 'image/png'); // 응답의 콘텐츠 타입을 PNG로 설정
          res.send(pngBuffer); // 변환된 PNG 파일을 클라이언트로 전송
        } catch (error) {
          console.error('Error converting PGM to PNG:', error);
          res.status(500).json({ message: 'Error converting PGM to PNG', error: error.message });
        }
      } else {
        res.set('Content-Type', monitoredMap.mimetype); // 다른 파일 형식 처리
        res.send(buffer); // 변환 없이 원본 파일을 전송
      }
    });

    downloadStream.on('error', (error) => {
      console.error('Error downloading map file:', error);
      res.status(500).json({ message: 'Error downloading map file', error: error.message });
    });
  } catch (error) {
    console.error('Error fetching monitored map file:', error);
    res.status(500).json({ message: 'Error fetching monitored map file', error: error.message });
  }
};

// 현재 모니터링 중인 메타데이터 파일 다운로드
exports.getCurrentMonitoredMapMetadata = async (req, res) => {
  try {
    const monitoredMap = await Map.findOne({ userId: req.user.id, isMonitored: true });
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map metadata found' });
    }

    const yamlFileId = monitoredMap.yamlFileId;
    const downloadStream = gfs.openDownloadStream(yamlFileId);

    downloadStream.on('error', (error) => {
      console.error('Error downloading map metadata file:', error);
      res.status(500).json({ message: 'Error downloading map metadata file', error: error.message });
    });

    downloadStream.pipe(res);
  } catch (error) {
    console.error('Error fetching monitored map metadata file:', error);
    res.status(500).json({ message: 'Error fetching monitored map metadata file', error: error.message });
  }
};

// 맵 삭제 (이름을 '0'으로 변경)
exports.deleteMap = async (req, res) => {
  try {
    const { id } = req.params;

    // 사용자가 소유한 맵인지 확인하고 이름을 '0'으로 변경
    const map = await Map.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { name: '0' },
      { new: true }
    );

    if (!map) {
      return res.status(404).json({ message: 'Map not found or not authorized' });
    }

    res.status(200).json({ message: 'Map deleted successfully (name set to 0)', map });
  } catch (error) {
    console.error('Error deleting map:', error);
    res.status(500).json({ message: 'Error deleting map', error: error.message });
  }
};


// 노드 생성 API (맵과 연결)
exports.createNode = async (req, res) => {
  const { name, x, y, tasks, mapId } = req.body; // task 대신 tasks로 변경하여 배열을 받음
  try {
    const newNode = new MapNode({ name, x, y, tasks, mapId }); // 배열로 처리
    await newNode.save();
    res.status(201).json({ message: '노드가 생성되었습니다.', node: newNode });
  } catch (error) {
    res.status(500).json({ error: '노드 생성 중 오류가 발생했습니다.' });
  }
};

// 특정 맵에 속한 노드 목록 조회
exports.getNodesByMap = async (req, res) => {
  const { mapId } = req.params;

  try {
    // 이름이 '0'이 아닌 노드만 조회
    const nodes = await MapNode.find({ mapId, name: { $ne: '0' } });
    res.status(200).json(nodes);
  } catch (error) {
    console.error('노드 목록 조회 중 오류:', error);
    res.status(500).json({ error: '노드 목록 조회 중 오류가 발생했습니다.' });
  }
};

// 노드 전체 목록 조회
exports.getAllNodes = async (req, res) => {
  try {
    // 이름이 '0'이 아닌 노드만 조회
    const nodes = await MapNode.find({ name: { $ne: '0' } });
    res.status(200).json(nodes);
  } catch (error) {
    console.error('노드 목록 조회 중 오류:', error);
    res.status(500).json({ error: '노드 목록을 가져오는 중 오류가 발생했습니다.' });
  }
};


exports.deleteNodeCompletely = async (req, res) => {
  const { nodeId } = req.params;

  // 유효한 ObjectId인지 확인
  if (!mongoose.Types.ObjectId.isValid(nodeId)) {
    return res.status(400).json({ error: '유효한 nodeId가 필요합니다.' });
  }

  try {
    // Step 1: 노드 삭제
    const deletedNode = await MapNode.findByIdAndDelete(nodeId);
    if (!deletedNode) {
      return res.status(404).json({ error: '노드를 찾을 수 없습니다.' });
    }

    // Step 2: 다른 노드들의 connections 배열에서 해당 nodeId 제거
    await MapNode.updateMany(
      { 'connections.node': nodeId },
      { $pull: { connections: { node: nodeId } } }
    );

    // Step 3: ShortPath 데이터 삭제 (필요 시)
    await ShortPath.deleteMany({
      $or: [
        { startNode: nodeId },
        { endNode: nodeId },
        { waypoints: nodeId } // 웨이포인트에 nodeId가 포함된 경우
      ]
    });

    res.status(200).json({ message: '노드와 관련된 연결이 성공적으로 삭제되었습니다.' });
  } catch (error) {
    console.error('노드 삭제 중 오류 발생:', error);
    res.status(500).json({ error: '서버 에러: 노드 삭제에 실패했습니다.' });
  }
};


exports.addStep = async (req, res) => {
  const { name } = req.body;  // 클라이언트로부터 전달받은 작업 이름

  if (!name) {
    return res.status(400).json({ message: '작업 이름이 필요합니다.' });
  }

  try {
    // 새로운 작업 생성
    const newStep = new StepList({ name });
    await newStep.save();

    res.status(201).json({ message: '작업이 성공적으로 추가되었습니다.', step: newStep });
  } catch (error) {
    console.error('작업 추가 중 오류:', error);
    res.status(500).json({ message: '작업 추가 중 오류가 발생했습니다.' });
  }
};

// 작업 목록 조회
exports.getSteps = async (req, res) => {
  try {
    // 이름이 '0'이 아닌 작업 목록만 조회
    const steps = await StepList.find({ name: { $ne: '0' } }); // name이 '0'이 아닌 항목만 조회
    res.status(200).json(steps);
  } catch (error) {
    console.error('작업 목록 조회 중 오류:', error);
    res.status(500).json({ message: '작업 목록 조회 중 오류가 발생했습니다.' });
  }
};

exports.addStepToNode = async (req, res) => {
  const { nodeId, stepId } = req.body; // 요청에서 nodeId와 stepId를 받음

  try {
    // 해당 노드를 찾아서 tasks 배열에 stepId를 추가
    const node = await MapNode.findByIdAndUpdate(
      nodeId,
      { $push: { tasks: stepId } }, // tasks 배열에 stepId를 추가
      { new: true } // 업데이트된 문서를 반환
    );

    if (!node) {
      return res.status(404).json({ message: 'Node not found' });
    }

    res.status(200).json({ message: 'Step added to tasks successfully', node });
  } catch (error) {
    console.error('Error adding step to tasks:', error);
    res.status(500).json({ message: 'Error adding step to tasks', error });
  }
};
exports.removeStep = async (req, res) => {
  const { stepId } = req.body; // 요청에서 stepId를 받음

  try {
    // 해당 step의 이름을 '0'으로 변경하여 검색에서 제외
    const step = await StepList.findByIdAndUpdate(
      stepId,
      { name: '0' }, // name 필드를 '0'으로 업데이트
      { new: true } // 업데이트된 문서를 반환
    );

    if (!step) {
      return res.status(404).json({ message: 'Step not found' });
    }

    res.status(200).json({ message: 'Step name updated to 0 successfully', step });
  } catch (error) {
    console.error('Error updating step name:', error);
    res.status(500).json({ message: 'Error updating step name', error });
  }
};
/*exports.connectNodes = async (req, res) => {
  const { node1, node2 } = req.body;

  if (!node1 || !node2) {
    return res.status(400).json({ message: '노드 ID가 누락되었습니다.' });
  }

  try {
    // 첫 번째 노드에 두 번째 노드 추가
    await MapNode.findByIdAndUpdate(node1, {
      $addToSet: { connections: node2 } // 중복 방지를 위해 $addToSet 사용
    });

    // 두 번째 노드에 첫 번째 노드 추가
    await MapNode.findByIdAndUpdate(node2, {
      $addToSet: { connections: node1 }
    });

    res.status(200).json({ message: '노드가 성공적으로 연결되었습니다.' });
  } catch (error) {
    console.error('노드 연결 중 오류 발생:', error);
    res.status(500).json({ message: '노드 연결에 실패했습니다.', error });
  }
};*/
exports.connectNodes = async (req, res) => {
  const { node1, node2, waypoints = [] } = req.body;

  if (!node1 || !node2) {
    return res.status(400).json({ message: '노드 ID가 누락되었습니다.' });
  }

  try {
    // 노드 정보를 가져와 거리 계산
    const startNode = await MapNode.findById(node1);
    const endNode = await MapNode.findById(node2);

    if (!startNode || !endNode) {
      return res.status(404).json({ message: '노드를 찾을 수 없습니다.' });
    }

    // 웨이포인트를 포함한 총 거리 계산 함수
    const calculateTotalDistance = (points) => {
      let totalDistance = 0;
      for (let i = 0; i < points.length - 1; i++) {
        const dx = points[i + 1].x - points[i].x;
        const dy = points[i + 1].y - points[i].y;
        totalDistance += Math.sqrt(dx * dx + dy * dy);
      }
      return totalDistance;
    };

    // 시작 노드, 웨이포인트, 끝 노드를 모두 포함하는 배열 생성
    const allPoints = [{ x: startNode.x, y: startNode.y }, ...waypoints, { x: endNode.x, y: endNode.y }];
    const totalDistance = calculateTotalDistance(allPoints);

    // 첫 번째 노드에 두 번째 노드와 거리 및 웨이포인트 추가
    await MapNode.findByIdAndUpdate(node1, {
      $addToSet: { 
        connections: { 
          node: node2, 
          distance: totalDistance, // 웨이포인트를 포함한 거리 사용
          waypoints 
        } 
      }
    });

    // 두 번째 노드에 첫 번째 노드와 거리 및 웨이포인트 추가 (양방향 연결)
    await MapNode.findByIdAndUpdate(node2, {
      $addToSet: { 
        connections: { 
          node: node1, 
          distance: totalDistance, // 동일 거리 사용
          waypoints: waypoints.slice().reverse() // 반대 방향 경로
        } 
      }
    });

    res.status(200).json({ message: '노드가 성공적으로 연결되었습니다.', distance: totalDistance });
  } catch (error) {
    console.error('노드 연결 중 오류 발생:', error);
    res.status(500).json({ message: '노드 연결에 실패했습니다.', error });
  }
};

exports.removeConnectionFromNode = async (req, res) => {
  const { nodeId, connectionNodeId } = req.params;

  try {
    // nodeId에서 connectionNodeId와의 연결 제거
    await MapNode.findByIdAndUpdate(
      nodeId,
      { $pull: { connections: { node: connectionNodeId } } },
      { new: true }
    );

    // connectionNodeId에서 nodeId와의 연결 제거 (양방향 연결 제거)
    await MapNode.findByIdAndUpdate(
      connectionNodeId,
      { $pull: { connections: { node: nodeId } } },
      { new: true }
    );

    res.status(200).json({ message: '양쪽 노드 간의 연결이 성공적으로 해제되었습니다.' });
  } catch (error) {
    console.error('연결 해제 중 오류 발생:', error);
    res.status(500).json({ message: '연결 해제에 실패했습니다.' });
  }
};
exports.createNoGoZone = async (req, res) => {
  const { topLeft, bottomRight, mapId } = req.body;

  if (!topLeft || !bottomRight || !mapId) {
    return res.status(400).json({ message: 'topLeft, bottomRight, mapId가 모두 필요합니다.' });
  }

  try {
    const newNoGoZone = new NoGoZone({ topLeft, bottomRight, mapId });
    await newNoGoZone.save();

    res.status(201).json({ message: '금지구역이 성공적으로 생성되었습니다.', noGoZone: newNoGoZone });
  } catch (error) {
    console.error('금지구역 생성 중 오류:', error);
    res.status(500).json({ message: '금지구역 생성 중 오류가 발생했습니다.' });
  }
};
exports.getNoGoZonesByMap = async (req, res) => {
  const { mapId } = req.params;

  try {
    const noGoZones = await NoGoZone.find({ mapId });
    res.status(200).json(noGoZones);
  } catch (error) {
    console.error('금지구역 조회 중 오류:', error);
    res.status(500).json({ message: '금지구역 조회 중 오류가 발생했습니다.' });
  }
};
exports.deleteNoGoZone = async (req, res) => {
  const { zoneId } = req.params;

  try {
    // zoneId에 해당하는 금지 구역을 찾아서 삭제
    const deletedZone = await NoGoZone.findByIdAndDelete(zoneId);

    if (!deletedZone) {
      return res.status(404).json({ message: '금지 구역을 찾을 수 없습니다.' });
    }

    res.status(200).json({ message: '금지 구역이 성공적으로 삭제되었습니다.', deletedZone });
  } catch (error) {
    console.error('금지 구역 삭제 중 오류 발생:', error);
    res.status(500).json({ message: '금지 구역 삭제 중 오류가 발생했습니다.', error: error.message });
  }
};

exports.calculateAndStoreShortPaths = async (req, res) => {
  const { mapId } = req.body;

  if (!mapId || !mongoose.Types.ObjectId.isValid(mapId)) {
    return res.status(400).json({ error: '유효한 mapId가 필요합니다.' });
  }

  try {
    const nodes = await MapNode.find({ mapId }).populate('connections.node').exec();

    if (nodes.length < 1) {
      return res.status(400).json({ error: '노드가 충분하지 않습니다.' });
    }

    const nodeMap = {};
    nodes.forEach(node => {
      nodeMap[node._id.toString()] = node;
    });

    const results = [];

    for (let i = 0; i < nodes.length; i++) {
      const startNode = nodes[i];

      // 자기 자신에 대한 경로 저장
      const existingSelfPath = await ShortPath.findOne({
        startNode: startNode._id,
        endNode: startNode._id,
        mapId
      });

      if (!existingSelfPath) {
        const selfPathDoc = new ShortPath({
          startNode: startNode._id,
          startNodeInfo: { name: startNode.name, x: startNode.x, y: startNode.y },
          endNode: startNode._id,
          endNodeInfo: { name: startNode.name, x: startNode.x, y: startNode.y },
          path: [{ node: startNode._id, x: startNode.x, y: startNode.y }], // 경로는 자기 자신만 포함
          totalDistance: 0, // 거리는 0
          mapId
        });

        await selfPathDoc.save();
        results.push({
          start: startNode.name,
          end: startNode.name,
          totalDistance: 0
        });
        console.log(`자기 자신 경로 저장: ${startNode.name} -> ${startNode.name} (거리: 0)`);
      }

      // 다른 노드와의 경로 계산
      for (let j = i + 1; j < nodes.length; j++) {
        const endNode = nodes[j];

        // 순방향 경로 계산
        const shortestPathForward = findShortestPath(startNode._id, endNode._id, nodeMap);

        if (shortestPathForward) {
          const existingPathForward = await ShortPath.findOne({
            startNode: startNode._id,
            endNode: endNode._id,
            mapId
          });

          if (!existingPathForward) {
            const fullPathForward = [
              { node: startNode._id, x: startNode.x, y: startNode.y }, // 출발 노드 추가
              ...shortestPathForward.path
            ];

            const shortPathDoc = new ShortPath({
              startNode: startNode._id,
              startNodeInfo: { name: startNode.name, x: startNode.x, y: startNode.y },
              endNode: endNode._id,
              endNodeInfo: { name: endNode.name, x: endNode.x, y: endNode.y },
              path: fullPathForward,
              totalDistance: shortestPathForward.totalDistance,
              mapId
            });

            await shortPathDoc.save();
            results.push({
              start: startNode.name,
              end: endNode.name,
              totalDistance: shortestPathForward.totalDistance
            });
            console.log(`경로 저장: ${startNode.name} -> ${endNode.name} (${shortestPathForward.totalDistance})`);
          }
        }

        // 반대 방향 경로 계산
        const shortestPathBackward = findShortestPath(endNode._id, startNode._id, nodeMap);

        if (shortestPathBackward) {
          const existingPathBackward = await ShortPath.findOne({
            startNode: endNode._id,
            endNode: startNode._id,
            mapId
          });

          if (!existingPathBackward) {
            const fullPathBackward = [
              { node: endNode._id, x: endNode.x, y: endNode.y }, // 출발 노드 추가
              ...shortestPathBackward.path
            ];

            const shortPathDoc = new ShortPath({
              startNode: endNode._id,
              startNodeInfo: { name: endNode.name, x: endNode.x, y: endNode.y },
              endNode: startNode._id,
              endNodeInfo: { name: startNode.name, x: startNode.x, y: startNode.y },
              path: fullPathBackward,
              totalDistance: shortestPathBackward.totalDistance,
              mapId
            });

            await shortPathDoc.save();
            results.push({
              start: endNode.name,
              end: startNode.name,
              totalDistance: shortestPathBackward.totalDistance
            });
            console.log(`경로 저장: ${endNode.name} -> ${startNode.name} (${shortestPathBackward.totalDistance})`);
          }
        }
      }
    }

    res.status(200).json({ message: '경로 계산 및 저장 완료', results });
  } catch (error) {
    console.error('경로 계산 중 오류:', error);
    res.status(500).json({ error: '서버 에러', details: error.message });
  }
};

exports.deleteShortPathsByMapId = async (req, res) => {
  const { mapId } = req.params; // URL 파라미터로 mapId 받기

  if (!mongoose.Types.ObjectId.isValid(mapId)) {
    return res.status(400).json({ error: '유효한 mapId가 필요합니다.' });
  }

  try {
    const result = await ShortPath.deleteMany({ mapId });
    res.status(200).json({
      message: `mapId ${mapId}에 속한 ShortPath 문서가 성공적으로 삭제되었습니다.`,
      deletedCount: result.deletedCount
    });
  } catch (error) {
    console.error('ShortPath 삭제 중 오류:', error);
    res.status(500).json({
      error: '서버 에러: ShortPath 삭제에 실패했습니다.'
    });
  }
};

exports.getMonitoredMapNodes = async (req, res) => {
  try {
    // 모니터링 중인 맵 조회
    const monitoredMap = await Map.findOne({ isMonitored: true });
    if (!monitoredMap) {
      return res.status(404).json({ error: '모니터링 중인 맵이 없습니다.' });
    }

    // 해당 맵의 노드 가져오기
    const nodes = await MapNode.find({ mapId: monitoredMap._id });
    if (!nodes.length) {
      return res.status(404).json({ error: '모니터링 중인 맵에 노드가 없습니다.' });
    }

    res.status(200).json({
      map: {
        id: monitoredMap._id,
        name: monitoredMap.name,
        description: monitoredMap.description,
      },
      nodes,
    });
  } catch (error) {
    console.error('노드 데이터를 가져오는 중 오류:', error);
    res.status(500).json({ error: '서버 오류가 발생했습니다.', details: error.message });
  }
};
exports.getPathByNodes = async (req, res) => {
  const { startNodeName, startX, startY, endNodeName, endX, endY } = req.query;

  // 입력 값 검증
  if (!startNodeName || startX === undefined || startY === undefined || !endNodeName || endX === undefined || endY === undefined) {
    return res.status(400).json({ error: '출발 노드와 도착 노드의 이름 및 좌표 값이 필요합니다.' });
  }

  try {
    // ShortPath에서 출발 노드와 도착 노드를 기준으로 경로 조회
    const storedPath = await ShortPath.findOne({
      'startNodeInfo.name': startNodeName,
      'startNodeInfo.x': Number(startX),
      'startNodeInfo.y': Number(startY),
      'endNodeInfo.name': endNodeName,
      'endNodeInfo.x': Number(endX),
      'endNodeInfo.y': Number(endY),
    });

    // 경로가 없을 경우
    if (!storedPath) {
      return res.status(404).json({ error: '저장된 경로를 찾을 수 없습니다.' });
    }

    // 성공적으로 경로 반환
    res.status(200).json({
      message: '경로를 성공적으로 조회했습니다.',
      path: storedPath.path,
      totalDistance: storedPath.totalDistance,
      startNode: storedPath.startNodeInfo,
      endNode: storedPath.endNodeInfo,
    });
  } catch (error) {
    console.error('경로 조회 중 오류:', error);
    res.status(500).json({ error: '경로 조회 중 서버 오류가 발생했습니다.', details: error.message });
  }
};