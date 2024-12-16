const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
  userId: { type: mongoose.Schema.Types.ObjectId, ref: 'User', required: true },
  name: { type: String, required: true },
  description: { type: String, required: true },
  isMonitored: { type: Boolean, default: false },
  filename: { type: String, required: true },
  FileId: { type: mongoose.Schema.Types.ObjectId, required: true },
  yamlFileId: { type: mongoose.Schema.Types.ObjectId, required: true } // YAML 파일 ID 추가
});

const Map = mongoose.model('Map', mapSchema);

module.exports = Map;

