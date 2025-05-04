const express = require('express');
const app = express();
const port = 3000;
const path = require('path');


/* 
HTTP Requests to trigger interrupt . Change a specific light to green - until the group recieves a reset . 
example of use (assuming server is launched on localhost and port 3000)
http://localhost:3000/changeGreen/A2 
http://localhost:3000/reset/A2 
api return :
-404 when the request is invalid / not possible 
-400 when the request is valid but there some conditions that are not met (such as triggering an interrupt to a group where the interrupt is active )
-200 when the request is valid
*/
app.use(express.json());


app.get('/changeGreen/:id', (req, res) => {
  const id = req.params.id;
  const light = global.lights && global.lights[id];
  if (!light) return res.status(404).json({ error: 'invalid id' });;
  // Find the group that contains this light
  const group = global.trafficGroupsList.find(g => g.lightIDs.includes(id));
  if (!group) return res.status(404).json({ error: 'invalid group for this id' });;
  if (group.interrupt.active) {return res.status(400).json({ error: 'group is interrupted already' });}
 // active the interrupt in the group 
  group.interrupt = {active: true,targetID: id};
  res.json({ message: `interrupt done ` });
  ;});

//Reset can be done either with id of the light or with the name of the group

app.get('/reset/:id', (req, res) => {
  const id = req.params.id;
  const light = global.lights[id];
  if (!light) return res.status(404).json({ error: 'invalid id' });;
  // Find the group that contains this light
  const group = global.trafficGroupsList.find(g => g.lightIDs.includes(id));
  if (!group) return res.status(404).json({ error: 'invalid group for this id' });
  if (!group.interrupt.active) {return res.status(400).json({ error: 'group is interrupted already' });}
  group.interrupt = {active: false,targetID: null};
  res.json({ message: `reset done ` });});


// Stop the interrupt with reset 
app.get('/reset/:group', (req, res) => {
  const groupName = req.params.group;
  const group = global.trafficGroupsList.find(g => g.name === groupName);
  if (!group) {return res.status(404).json({ error: 'invalid id'  });;}
  if (!group.interrupt.active) {return res.status(400).json({ error: 'group is  not interrupted' });}
  group.interrupt = {active: false,targetID: null};
  res.json({ message: `reset done ` });});


// Start the sv
app.listen(port, () => {
  console.log(`API server running at http://localhost:${port}`);
});
