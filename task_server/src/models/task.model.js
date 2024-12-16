const mongoose = require('mongoose');

const taskSchema = new mongoose.Schema({
  name: { type: String, required: true }, // 작업 이름 (필수)
  description: { type: String, default: '' }, // 작업 설명 (선택)
  taskType: { type: String, default: 'A' }, // 작업 유형 (선택)
  status: { 
    type: String, 
    enum: ['Pending', 'In Progress', 'Completed', 'Failed', 'Queued'], 
    default: 'Pending' 
  }, // 작업 상태
  createdAt: { type: Date, default: Date.now }, // 작업 생성 시간
  mapId: { type: mongoose.Schema.Types.ObjectId, ref: 'Map', required: true }, // 특정 맵과 연결 (필수)
  workflow: [{ 
    node: { type: String, default: '' }, // 노드 또는 단계 이름 (선택)
    step: { type: String, default: '' }, // 노드에서 수행할 작업 (선택)
    x: { type: Number, default: 0 }, // 기본값 설정
    y: { type: Number, default: 0 }, // 기본값 설정
  }], // 작업 플로우 정보 (선택)
});

module.exports = mongoose.model('Task', taskSchema);
