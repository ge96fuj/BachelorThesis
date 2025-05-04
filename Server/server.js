const net = require('net');
const { TrafficLight } = require('./core/TrafficLight.js');
const TrafficGroup = require('./core/TrafficGroup.js');
const lightConfig = require('./config/lightConfig.js');
require('./services/api.server.js');
// require('/Users/skanderjneyeh/Documents/Latency_Packet_loss_rep/Server/services/api.server.js');

require('./services/mqtt.service.js');

// SERVER CONFIG 
// IP = '192.168.0.103';
IP = '10.181.241.34';
PORT = 12345;
global.lights = {};
global.trafficGroupsList = [];

//default values if there is no duration in Config
const DEFAULT_DURATIONS = {
  red: 2000,
  yellow: 2000,
  green: 10000
};

const server = net.createServer((socket) => {
  console.log('Client connected: ', socket.remoteAddress);
  socket.setKeepAlive(true, 100);

  let resReceived = false;
  let cnt = 0;

  
  let handshakeTimer = null;
  // client has 3 tries to send 0x60 response
  const sendHandshake = () => {
    if (resReceived) return;

    if (cnt >= 3) {
      console.warn(`${socket.remoteAddress} do not send 0x60`);
      socket.close();
      return;
    }

    socket.write(Buffer.from([0x20]));
    console.log(`Sent 0x20 , ${socket.remoteAddress} (attempt ${cnt + 1})`);

    cnt++;
    //retry evry 3 sec
    handshakeTimer = setTimeout(sendHandshake, 3000);
  };

  sendHandshake();

  socket.on('data', (data) => {
    console.log('Received message:', data.toString());

    try {
      const { command } = JSON.parse(data);

      if (command === 60 && !resReceived) {
        resReceived = true;
        clearTimeout(handshakeTimer);
        console.log(`${socket.remoteAddress} sent 0x60 `);
}

    } catch (err) {
      console.warn(`Json error ${socket.remoteAddress}:`, data.toString());
      return;
    }

    handleReceivedMessage(data.toString(), socket);
  });
// make the socket as null if client disconnect
  const delSock = () => {
    clearTimeout(handshakeTimer);
    console.log(`${socket.remoteAddress} disconnects`);

    const light = Object.values(global.lights).find(l => l.socket === socket);
    if (light) {
      console.log(`${light.id} disconnects`);
      light.socket = null;
      light.status = null;
    }
  };

  socket.on('end', delSock);
  socket.on('close', delSock);
  socket.on('error', (err) => {
  console.error(`Err ,${err.message},  ${socket.remoteAddress}: `);
  delSock();
  });
});

server.listen(PORT, IP, () => {
  console.log('TCP server ON');
});

function handleReceivedMessage(data, socket) {
  try {
    if (!data.trim()) return console.log("Empty msg");

    const { command } = JSON.parse(data);
    switch (command) {
      case 60:
        addNewTrafficLight(JSON.parse(data), Object.keys(data).length, socket);
        break;
      case 90:
        break;
      default:
        console.log(`invalid command ${command}`);
    }
  } catch (error) {
    socket.end();
    console.log('Err ', error.message);
  }
}

function addNewTrafficLight(parsedData, dataLength, socket) {
  console.log('Adding Traffic Light');
  const { lightID } = parsedData;
  if (!lightID) {
    return console.log("no Light id in msg");
  }
  const light = global.lights[lightID];
  if (!light) {
    return console.log(` ${lightID} not in Config`);
  }

  light.socket = socket;
  light.status = 0x05;
  light.goBlink();

  console.log(`${lightID} added`);
}

//initialization of Traffic lights and groups based on lightConfig
function initConfig() {
  const groupedIDs = {};
  global.lights = {};
  global.trafficGroupsList = [];
  global.launchedGroups = new Set();
  const usedIDs = new Set();

  for (const { id, localization_x, localization_y, group, durations } of lightConfig) {
    if (usedIDs.has(id)) {
      throw new Error(`Duplicate ID  "${id}" , group "${group}"`);
    }
    const lightDurations = durations || DEFAULT_DURATIONS;
    global.lights[id] = new TrafficLight(id, localization_x, localization_y, undefined, null, lightDurations);
    console.log(` ${id} created `, lightDurations);
    usedIDs.add(id);
    if (!groupedIDs[group]) groupedIDs[group] = [];
    groupedIDs[group].push(id);
  }

  for (const [groupName, lightIDs] of Object.entries(groupedIDs)) {
    const group = new TrafficGroup(groupName, lightIDs);
    global.trafficGroupsList.push(group);
  }

  console.log("Lights initialized:", Object.keys(global.lights));
  console.log("Groups initialized:", global.trafficGroupsList.map(g => g.name));
}

global.launchedGroups = new Set();

//Launch the ready Groups
async function checkGrps() {
  console.log("Running checkAndLaunch function");
  for (const group of global.trafficGroupsList) {
    const groupName = group.name;

    if (global.launchedGroups.has(groupName)) continue;

    if (group.isReady()) {
      global.launchedGroups.add(groupName);
      group.goAllRed();
      console.log(`Group ${groupName} is ready. Starting cycle`);
      group.runCycle();
    } else {
      console.log(`Group ${groupName} not ready...`);
    }
  }
}

initConfig();

setInterval(() => {
  const allGroupsLaunched = global.trafficGroupsList.length === global.launchedGroups.size;
  if (!allGroupsLaunched) {
    checkGrps().catch(console.error);
  }
}, 3000);
