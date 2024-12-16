const mongoose = require('mongoose');
const Schema = mongoose.Schema;

const taskLogSchema = new Schema({
  robotName: { type: String, required: true },  // 작업을 수행한 로봇의 이름
  robotIp: { type: String, required: true },  // 작업을 수행한 로봇의 IP
  nodeName: { type: String, required: true },  // 작업 대상 노드의 이름
  step: { type: String, required: true },  // 노드에서의 작업 단계
  timestamp: { type: Date, default: Date.now },  // 작업 기록 시간
  status: { 
    type: String, 
    enum: ['In Progress', 'Done', 'Error','Cancle'], 
    default: 'In Progress' 
  },  // 작업 상태 (진행 중, 완료, 오류)
});

module.exports = mongoose.model('TaskLog', taskLogSchema);
