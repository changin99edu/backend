const mongoose = require('mongoose');

const noGoZoneSchema = new mongoose.Schema({
    topLeft: {
        x: { type: Number, required: true }, // 사각형 좌상단의 x 좌표
        y: { type: Number, required: true }, // 사각형 좌상단의 y 좌표
    },
    bottomRight: {
        x: { type: Number, required: true }, // 사각형 우하단의 x 좌표
        y: { type: Number, required: true }, // 사각형 우하단의 y 좌표
    },
    mapId: {
        type: mongoose.Schema.Types.ObjectId, // 특정 맵과 연결
        ref: 'Map', // Map 모델을 참조
        required: true,
    },
});

module.exports = mongoose.model('NoGoZone', noGoZoneSchema);