const mongoose = require('mongoose');

const mapNodeSchema = new mongoose.Schema({
  name: { type: String, required: true },  // 노드 이름 또는 식별자
  x: { type: Number, required: true },     // X 좌표
  y: { type: Number, required: true },     // Y 좌표
  tasks: [{ type: String }],               // 할당된 작업 (선택적)
  connections: [
    {
      node: { type: mongoose.Schema.Types.ObjectId, ref: 'MapNode', required: true },  // 연결된 다른 노드
      distance: { type: Number, default: 0 },  // 노드 간 거리 (가중치)
      waypoints: [
        {
          x: { type: Number, required: true },  // 웨이포인트의 X 좌표
          y: { type: Number, required: true }   // 웨이포인트의 Y 좌표
        }
      ]
    }
  ],
  mapId: { type: mongoose.Schema.Types.ObjectId, ref: 'Map', required: true },  // 해당 노드가 속한 맵 ID
  createdAt: { type: Date, default: Date.now },  // 노드 생성 시간
});

module.exports = mongoose.model('MapNode', mapNodeSchema);
