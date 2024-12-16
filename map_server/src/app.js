// dotenv 패키지를 로드하고 .env 파일의 환경 변수를 process.env 객체에 로드
require('dotenv').config();

const express = require('express'); // express 패키지를 로드, 애플리케이션 생성
const mongoose = require('mongoose'); // mongoose 패키지를 로드
const cors = require('cors'); // Cors 패키지 로드
const bodyParser = require('body-parser'); // body-parser 패키지를 로드, 요청 본문을 파싱
const cookieParser = require('cookie-parser'); // cookie-parser 패키지를 로드, 쿠키 파싱

// Routes 모듈을 로드, 관련 api
const mapRoutes = require('./routes/map.route');


const app = express();
// .env 파일의 주소 로드
const MONGODB_URI = process.env.MONGODB_URI;
const allowedOrigins = process.env.FRONT_URI.split(',');

// CORS 설정
app.use(cors({
  origin: (origin, callback) => {
    // 웹 브라우저에서 origin이 없는 경우 (예: Postman, curl)은 허용합니다.
    if (!origin || allowedOrigins.indexOf(origin) !== -1) {
      callback(null, true);
    } else {
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true
}));

// 요청 로그 출력
app.use((req, res, next) => {
  console.log(`${req.method} ${req.url}`);
  next();
});

app.use(bodyParser.json()); // 요청 본문 json으로 파싱
app.use(bodyParser.urlencoded({ extended: true }));
app.use(cookieParser()); // 쿠키 파싱

mongoose.connect(MONGODB_URI, { useNewUrlParser: true, useUnifiedTopology: true })
  .then(() => console.log('MongoDB connected')) // 연결이 성공하면 콘솔에 메시지를 출력합니다.
  .catch(err => console.log(err)); // 연결이 실패하면 콘솔에 오류 메시지를 출력합니다.

// '/map/...' 경로로 들어오는 요청은 ...Routes 모듈에서
app.use('/map', mapRoutes);

// 오류 핸들러
app.use((err, req, res, next) => {
  console.error('Unexpected error:', err.stack);
  res.status(500).send('Something broke!');
});

// app 모듈 내보내기
module.exports = app;
