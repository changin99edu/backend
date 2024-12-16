const mongoose = require('mongoose');   // mongoose 패키지를 로드

// accountSchema 정의
const accountSchema = new mongoose.Schema({
    email: { type: String, required: true, unique: true }, // 이메일 (아이디로 사용)
    password: { type: String, required: true }, // 패스워드
    name: { type: String, required: true }, // 사용자명
    phone: { type: String, required: true }, // 휴대전화 번호
    status: { type: String, enum: ['pending', 'active'], default: 'pending' }, // 계정 상태 대기, 활성화 (기본 값 대기)
    selectedMapId: { type: mongoose.Schema.Types.ObjectId, default: null, ref: 'Map' }, // 선택된 맵 ID 필드 추가
    createdAt: { type: Date, default: Date.now }, // 계정 생성 시간
});

module.exports = mongoose.model('Account', accountSchema);