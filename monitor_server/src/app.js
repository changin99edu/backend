require('dotenv').config();
const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');
const bodyParser = require('body-parser');
const cookieParser = require('cookie-parser');
const expressWs = require('express-ws');

const app = express();
expressWs(app);

const MONGODB_URI = process.env.MONGODB_URI;
const allowedOrigins = process.env.FRONT_URI.split(',');

console.log("Allowed Origins:", allowedOrigins);

app.use(cors({
  origin: (origin, callback) => {
    if (!origin || allowedOrigins.indexOf(origin) !== -1 || origin === `http://172.30.1.33:${process.env.PORT}`) {
      callback(null, true);
    } else {
      console.error(`Not allowed by CORS: ${origin}`);
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true
}));

app.use((req, res, next) => {
  console.log(`${req.method} ${req.url}`);
  next();
});

app.use(bodyParser.json());
app.use(cookieParser());

mongoose.connect(MONGODB_URI, { useNewUrlParser: true, useUnifiedTopology: true })
    .then(() => console.log('MongoDB connected'))
    .catch(err => console.log(err));

const monitorRoutes = require('./routes/monitor.route');
app.use('/monitor', monitorRoutes);

app.use((err, req, res, next) => {
  console.error('Unexpected error:', err.stack);
  res.status(500).send('Something broke!');
});

module.exports = app;

