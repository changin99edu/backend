const jwt = require('jsonwebtoken');
const axios = require('axios');

exports.authenticate = async (req, res, next) => {
  try {
    const authHeader = req.headers['authorization'];
    if (!authHeader) {
      return res.status(401).json({ message: 'Access denied. No token provided.' });
    }

    const token = authHeader.replace('Bearer ', '');
    const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
    req.user = decoded;

    // account_server에서 사용자 정보 확인
    const response = await axios.get(`http://localhost:5555/account/user/${req.user.id}`, {
      headers: { Authorization: `Bearer ${token}` }
    });

    if (response.status !== 200) {
      return res.status(401).json({ message: 'Invalid token.' });
    }

    req.user = response.data;
    req.user.id = req.user._id; // 추가된 부분

    next();
  } catch (error) {
    console.error('Error in authenticate middleware:', error);
    res.status(401).json({ message: 'Invalid token.' });
  }
};


