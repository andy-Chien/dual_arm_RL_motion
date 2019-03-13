#!/usr/bin/env node
'use strict';

var express = require('express');
var fs      = require('fs');
var path    = require('path');
var app     = module.exports = express();
var https   = require('https');           // must to use https for mic. permission
var server  = https.createServer({
  key: fs.readFileSync('server.key'),
  cert: fs.readFileSync('server.cert')
},app);
server.listen(8080);
var io = require('socket.io').listen(server);

/** ROS */
/** using rosnodejs, ros distribution: kinetic
 *  See: [rosnodejs](http://wiki.ros.org/rosnodejs/overview)
 * 
 *  Why not use rosbridge_server?
 *  There are some problem with multiple websocket on UI
 */
const rosnodejs = require('rosnodejs');
const AssistantState = rosnodejs.require('strategy').srv.AssistantState;
var serviceClient;

rosnodejs.initNode('index')
.then((rosNode) => {
  /** ROS Service Server */
  /** pass request string that want robot to say
   *  to web ui through socket, then send to ibm.
   *  request: std_msgs/String , response: std_msgs/Bool
   */
  const LetRobotSay = (req, resp) => {
    console.log(req.info);
    io.emit("say", JSON.stringify(req));
    resp.success = true;
    return resp;
  };
  let service = rosNode.advertiseService('/let_robot_say', 'nodejs_pkg/LetRobotSay', LetRobotSay);

  /** ROS Service Client */
  /** call strategy 'custom_server' with request 'state'
   *  call state 0 when initial
   */
  serviceClient = rosNode.serviceClient('/assistant_service','strategy/AssistantState');
  rosNode.waitForService(serviceClient.getService(), 2000)
    .then((available) => {
      if (available) {
        const request = new AssistantState.Request();
        request.state = 0;
        serviceClient.call(request).then((resp) => {
          console.log('Service response ' + JSON.stringify(resp));
        });
      } else {
        console.log('Service not available');
      }
    });
});
/** NodeJS */
app.get('/', function (req, res) {
  app.use(express.static(path.join(__dirname, '../web')));
  app.use(express.static(path.join(__dirname, '..')));

  res.sendFile(path.join(__dirname + '/../web/demo03.html'));
});
/** Socket.io */
io.on('connect', (socket) => {
  console.log('A user connected.');

  io.emit("news", "Hello from Socket.io server");

  /** Pass data from web ui to ROS strategy thru service */
  socket.on('message', (data) => {
    console.log('Call service with: '+data);
    const request = new AssistantState.Request();
    request.state = data;

    serviceClient.call(request).then((resp) => {
      console.log('Service response ' + JSON.stringify(resp));
      io.emit("news", JSON.stringify(resp));
    });
  });
  socket.on('error', (error) => {
    console.log("Socket.io error occured: " + error);
  });
  socket.on('disconnect', () => {
    console.log("A user go out");
  });
});