const Robot = require('../models/robot.model');

let clients = [];
let robotPositions = {};

exports.monitorUpdates = (ws, req) => {
  clients.push(ws);
  console.log('WebSocket connected');

  ws.on('message', (message) => {
    try {
      const data = JSON.parse(message);
      if (data.x !== undefined && data.y !== undefined) {
        robotPositions[ws.id] = data;
        console.log('Updated robot position:', data);
      }
    } catch (error) {
      console.error('Error processing message:', error);
    }
  });

  ws.on('close', () => {
    clients = clients.filter(client => client !== ws);
    delete robotPositions[ws.id];
    console.log('WebSocket disconnected');
  });

  setInterval(() => {
    clients.forEach(client => {
      if (client.readyState === 1) {
        client.send(JSON.stringify(Object.values(robotPositions)));
      }
    });
  }, 1000);
};

async function getRobotPositions() {
  try {
    const robots = await Robot.find().select('name location');
    return robots;
  } catch (error) {
    console.error('Error fetching robot positions:', error);
    return [];
  }
}
