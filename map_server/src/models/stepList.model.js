const mongoose = require('mongoose');

const taskSchema = new mongoose.Schema({
  name: { type: String, required: true },  // 작업 이름 (필수)
});

module.exports = mongoose.model('Task', taskSchema);
