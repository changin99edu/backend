require('dotenv').config();
const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');
const bodyParser = require('body-parser');
const cookieParser = require('cookie-parser');

const taskRoutes = require('./routes/task.route');  // Task 관련 라우터
const { initRosSubscriber } = require('./ros/rosSubscriber');  // ROS 서브스크라이버 통합

const app = express();
const MONGODB_URI = process.env.MONGODB_URI;
const allowedOrigins = process.env.FRONT_URI.split(',');

// CORS 설정
app.use(cors({
  origin: (origin, callback) => {
    if (!origin || allowedOrigins.indexOf(origin) !== -1) {
      callback(null, true);
    } else {
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true  // 쿠키를 포함한 요청을 허용
}));

// 요청 로깅 미들웨어
app.use((req, res, next) => {
  console.log(`${req.method} ${req.url}`);
  next();
});

// Body parser와 Cookie parser 설정
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));
app.use(cookieParser());

// MongoDB 연결
mongoose.connect(MONGODB_URI, { useNewUrlParser: true, useUnifiedTopology: true })
  .then(() => console.log('MongoDB 연결 성공'))
  .catch(err => console.log('MongoDB 연결 실패:', err));

// Task API 라우터 적용
app.use('/task', taskRoutes);  // 작업(Task) 관련 라우터

// ROS 서브스크라이버 초기화
initRosSubscriber();

// 에러 처리 미들웨어
app.use((err, req, res, next) => {
  console.error('Unexpected error:', err.stack);
  res.status(500).send('서버에 오류가 발생했습니다!');
});

module.exports = app;
