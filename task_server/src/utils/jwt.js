const jwt = require('jsonwebtoken');

exports.generateToken = (payload, expiresIn) => {
    return jwt.sign(payload, process.env.JWT_SECRET_KEY, { expiresIn });
};

exports.verifyToken = (token) => {
    try {
        return jwt.verify(token, process.env.JWT_SECRET_KEY);
    } catch (error) {
        return null;
    }
};
