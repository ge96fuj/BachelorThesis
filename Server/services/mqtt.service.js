/* 
Connect to MQTT Broker and publish the status of all available traffic lights .
*/

var mqtt = require('mqtt');
const { TrafficLightStatus } = require('../core/TrafficLight');

// Connect to  MQTT sv
const mqttClient = mqtt.connect('mqtt://10.181.241.34:1883');

mqttClient.on('connect', () => {
  console.log(' Connected to MQTT ');
});

mqttClient.on('error', (err) => {
  console.error(' Failed to connect to MQTT Sv:', err);
});

// Change enum val to Strings
var statusMap = {
  [TrafficLightStatus.RED]: "RED",
  [TrafficLightStatus.YELLOW_TO_G]: "YELLOW", //red to green
  [TrafficLightStatus.GREEN]: "GREEN",
  [TrafficLightStatus.YELLOW_TO_R]: "YELLOW", //green to red
  [TrafficLightStatus.UNKNOWN]: "UNKNOWN",
  [TrafficLightStatus.BLINK]: "BLINK"

};

// Publishes current traffic light statuses 
function publishTrafficLightStat() {


  if (!global.lights) return;

  for (const [id, light] of Object.entries(global.lights)) {
    
    const payload = JSON.stringify({
      status: statusMap[light.status] || "UNKNOWN",
      timestamp: Date.now()
    });

    mqttClient.publish(`detection/traffic_lights/${id}`, payload);
    
    mqttClient.publish(`detection/status/${id}`, statusMap[light.status]); // for the simulator .
   



    // console.log(` Publishing in  detection/status/${id}` , payload);
 }
   }



// Freq of publishing
setInterval(publishTrafficLightStat, 1000);

module.exports = mqttClient;


