// models/ShortPath.js

const mongoose = require('mongoose');

const waypointSchema = new mongoose.Schema({
  x: { type: Number, required: true },
  y: { type: Number, required: true }
}, { _id: false });

const shortPathSchema = new mongoose.Schema({
  startNode: { type: mongoose.Schema.Types.ObjectId, ref: 'MapNode', required: true },
  startNodeInfo: {
    name: { type: String, required: true },
    x: { type: Number, required: true },
    y: { type: Number, required: true }
  },
  endNode: { type: mongoose.Schema.Types.ObjectId, ref: 'MapNode', required: true },
  endNodeInfo: {
    name: { type: String, required: true },
    x: { type: Number, required: true },
    y: { type: Number, required: true }
  },
  path: [waypointSchema], // 최단 경로를 이루는 웨이포인트 배열
  totalDistance: { type: Number, required: true },
  mapId: { type: mongoose.Schema.Types.ObjectId, ref: 'Map', required: true },
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('ShortPath', shortPathSchema);
