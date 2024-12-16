// utils/findShortestPath.js

const findShortestPath = (startNodeId, endNodeId, nodeMap) => {
  // 디버깅: nodeMap 타입 확인
  console.log('Inside findShortestPath');
  console.log('nodeMap is an object:', typeof nodeMap === 'object'); // true이어야 함
  console.log('nodeMap has keys:', Object.keys(nodeMap).length > 0); // true이어야 함

  // Dijkstra 알고리즘을 사용하여 최단 경로 계산
  const distances = {}; // 노드 ID별 거리
  const previous = {};  // 노드 ID별 이전 노드
  const unvisited = new Set(Object.keys(nodeMap));

  // 초기화
  for (const nodeId in nodeMap) {
    distances[nodeId] = Infinity;
    previous[nodeId] = null;
  }
  distances[startNodeId.toString()] = 0;

  while (unvisited.size > 0) {
    // 현재 가장 가까운 노드 선택
    let currentNodeId = null;
    let minDistance = Infinity;
    for (const nodeId of unvisited) {
      const distance = distances[nodeId];
      if (distance < minDistance) {
        minDistance = distance;
        currentNodeId = nodeId;
      }
    }

    if (currentNodeId === endNodeId.toString() || minDistance === Infinity) {
      break; // 도착 노드에 도달하거나 더 이상 갈 수 없는 경우
    }

    unvisited.delete(currentNodeId);
    const currentNode = nodeMap[currentNodeId];

    // 인접 노드 탐색
    currentNode.connections.forEach(connection => {
      const neighborId = connection.node._id.toString();
      if (!unvisited.has(neighborId)) return;

      const alt = distances[currentNodeId] + connection.distance;
      if (alt < distances[neighborId]) {
        distances[neighborId] = alt;
        previous[neighborId] = currentNodeId;
      }
    });
  }

  // 최단 경로 재구성
  const path = [];
  let currentNodeId = endNodeId.toString();
  if (previous[currentNodeId] || currentNodeId === startNodeId.toString()) {
    while (currentNodeId) {
      path.unshift(currentNodeId);
      currentNodeId = previous[currentNodeId];
    }
  }

  if (path.length === 0) return null;

  // 총 거리 계산
  const totalDistance = distances[endNodeId.toString()];

  // 웨이포인트 정보 수집
  const fullPath = [];
  for (let i = 0; i < path.length - 1; i++) {
    const fromNode = nodeMap[path[i]];
    const toNode = nodeMap[path[i + 1]];

    // 연결된 경로 찾기
    const connection = fromNode.connections.find(conn => conn.node._id.toString() === toNode._id.toString());
    if (connection) {
      // 현재 노드에서 다음 노드로 가는 웨이포인트 추가
      connection.waypoints.forEach(wp => {
        fullPath.push({ x: wp.x, y: wp.y });
      });
      // 다음 노드의 좌표 추가
      fullPath.push({ x: toNode.x, y: toNode.y });
    }
  }

  return {
    path: fullPath, // 웨이포인트를 포함한 경로
    totalDistance
  };
};

module.exports = findShortestPath;
