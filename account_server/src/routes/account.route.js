const express = require('express'); // express 패키지 로드
const router = express.Router(); 
const accountController = require('../controllers/account.controller'); // /controllers/account.controller 로드
const auth = require('../middlewares/auth').authenticate; 

// 회원가입 함수
router.post('/register', accountController.register);  
// login함수
router.post('/login', accountController.login); 
// logout함수
router.post('/logout', auth, accountController.logout); 
// 사용자 정보 함수
router.get('/userinfo', auth, accountController.getUserInfo);
// 사용자 정보를 가져오는 라우트
router.get('/user/:id', auth, accountController.getUserById);
// PUT 사용자 정보 업데이트
router.put('/user/:id', accountController.updateUser);

module.exports = router;