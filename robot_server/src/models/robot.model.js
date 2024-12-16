const mongoose = require('mongoose');

const robotSchema = new mongoose.Schema({
  name: { type: String, required: true },
  ip: { type: String, required: true },
  model: { type: String, required: true },
  sshUser: { type: String, required: true },
  userId: { type: mongoose.Schema.Types.ObjectId, ref: 'Account', required: true },
  description: { type: String, default: '설명없음' },
  location: {
    x: { type: Number, default: 0.0},
    y: { type: Number, default: 0.0 }
  },
  status: {                         
    battery: { type: Number, default: 0 },  
    state: { type: String, default: 'None' }, 
    speed: { type: Number, default: 0.0 }       
  },
  currentWorkflow: {
        node: { type: String, default: null },  // 현재 작업 노드
        step: { type: String, default: null },  // 현재 작업 단계
        x: { type: Number, default: 0 }, // 기본값 설정
        y: { type: Number, default: 0 }, // 기본값 설정
        status: { 
          type: String, 
          enum: ['Pending', 'In Progress', 'Completed'], 
          default: 'In Progress' 
        }
    },
  active: { type: Number, default: 0 }, // Active 상태 (0 또는 1)
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Robot', robotSchema);

