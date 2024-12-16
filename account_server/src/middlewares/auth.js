const jwt = require('jsonwebtoken');
const Account = require('../models/account.model');

exports.authenticate = async (req, res, next) => {
  const token = req.headers['authorization']?.split(' ')[1];

  if (!token) {
    return res.status(401).json({ message: 'Unauthorized' });
  }

  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
    req.user = await Account.findById(decoded.id);
    if (!req.user) {
      return res.status(401).json({ message: 'Unauthorized' });
    }
    next();
  } catch (error) {
    res.status(401).json({ message: 'Unauthorized' });
  }
};

