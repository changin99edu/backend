const mongoose = require('mongoose');

const robotSchema = new mongoose.Schema({
  name: { type: String, required: true },
  ip: { type: String, required: true },
  model: { type: String, required: true },
  userId: { type: mongoose.Schema.Types.ObjectId, ref: 'Account', required: true },
  location: {
    x: { type: Number, default: 0 },
    y: { type: Number, default: 0 }
  },
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Robot', robotSchema);
