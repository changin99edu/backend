const mongoose = require('mongoose');

const workflowQueueSchema = new mongoose.Schema({
    robotId: { type: mongoose.Schema.Types.ObjectId, ref: 'Robot', required: true, unique: true }, // 로봇마다 하나의 큐
    workflows: [
        {
            node: { type: String, required: true },  // 노드 이름 또는 ID
            step: { type: String, required: true },  // 스텝 이름 또는 ID
            x: { type: Number, default: 0 }, // 기본값 설정
            y: { type: Number, default: 0 }, // 기본값 설정
            status: { 
                type: String, 
                enum: ['Pending', 'In Progress', 'Completed'], 
                default: 'Pending' 
            },  // 각 워크플로우 단계의 상태
            assignedAt: { type: Date, default: Date.now } // 할당된 시간
        }
    ],
    createdAt: { type: Date, default: Date.now }
});

const WorkflowQueue = mongoose.model('WorkflowQueue', workflowQueueSchema);

module.exports = WorkflowQueue;