const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
  userId: { type: mongoose.Schema.Types.ObjectId, ref: 'User', required: true },
  name: { type: String, required: true },
  description: { type: String, required: true },
  isMonitored: { type: Boolean, default: false },
  filename: { type: String, required: true },
  yamlFile: { type: String, required: true }, // YAML 파일명 추가
  FileId: { type: mongoose.Schema.Types.ObjectId, required: true }
});

const Map = mongoose.model('Map', mapSchema);

module.exports = Map;







