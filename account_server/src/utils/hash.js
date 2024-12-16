const bcrypt = require('bcryptjs'); // bcryptjs 패키지 로드

// 비밀번호를 해시하는 함수
exports.hashPassword = async (password) => {
    return await bcrypt.hash(password, 10);
};

// 비밀번호를 검증하는 함수
exports.comparePassword = async (password, hashedPassword) => {
    return await bcrypt.compare(password, hashedPassword);
};