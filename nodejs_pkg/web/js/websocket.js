ws = new WebSocket("wss://shengruchatbot1.mybluemix.net/ws/sound");
var heartbeat_msg = '--heartbeat--', heartbeat_interval = null, missed_heartbeats = 0;
var output = document.getElementById('output');

function __log(e, data) {
  log.innerHTML += "\n" + e + " " + (data || '');
}
function __mlog(e, data) {
  let tmp = mlog.innerHTML;
  mlog.innerHTML = e + " " + (data || '') + "\n" + tmp;
}
/** Socket.io pass robot state to socket server */
var socket = io.connect('https://'+location.host, {secure: true});
var paySW = 0;
socket.on('news', function(m) {
  if (IsJsonString(m)) {
    let j = JSON.parse(m);
    if (j.success) {
      /** Robot Speaking State Machine */
      if (j.info.toLowerCase() == "here are your meals") {
        paySW = 1;
        output.src = "sounds/here_are_your_meals.wav"
        output.play();
        __log('Pass payment 4 to socket server');
        __mlog('Strategy Requesting...');
        socket.emit('message', 4);
      }else if (j.info.toLowerCase() == "payment process complete") {
        output.src = "sounds/payment_process_complete.wav"
        output.play();
        switch (paySW) {
          case 0:
            PlasticPay('img/1.jpg', 'NEET', 'Sprite', 200);
            break;
          case 1:
            PlasticPay('img/2.jpg', 'Shengru', 'Lunch Box', 2000);
            paySW = 0;
            break;
        }
      }else if (j.info.toLowerCase() == "payment failed") {
        output.src = "sounds/payment_failed_please_try_again.wav"
        output.play();
        __log('Pass payment 4 to socket server');
        __mlog('Strategy Requesting...');
        socket.emit('message', 4);
      }else {
        Sayit(j.info);
      }
    }
  }else {
    console.log('[Socket.IO Connected]: '+m);
    __log('[Socket.IO Connected]');
    __mlog('[Socket.IO Connected]');
  }
});
socket.on('say', function(m) {
  let j = JSON.parse(m);
  console.log('Let Robot says: '+j.info);
  __log('Let Robot says: '+j.info);
  Sayit(j.info);
});

ws.onopen = function(){
  __log('[Assistant Connected]');
  __mlog('[Assistant Connected]');
  /** Ping/Pong signal for keeping websocket alive */
  if (heartbeat_interval === null) {
    missed_heartbeats = 0;
    heartbeat_interval = setInterval(function() {
      try {
        missed_heartbeats++;
        if (missed_heartbeats >= 3)
          throw new Error("Too many missed heartbeats.");
        ws.send(heartbeat_msg);
      } catch(e) {
        clearInterval(heartbeat_interval);
        heartbeat_interval = null;
        console.warn("Closing connection. Reason: " + e.message);
        ws.close();
      }
    }, 5000);
  }
};
ws.onerror = function(){ 
  console.log("Websocket error");
  __log('[Assistant Connected Error]');
  __mlog('[Assistant Connected Error]');
};
ws.onclose = function () {
  __log('[Assistant Disconnected]');
  __mlog('[Assistant Disconnected]');
}
var state, record;
ws.onmessage = function(evt){
  if (IsJsonString(evt.data)) {
    var j = JSON.parse(evt.data)
    if (j.role === "State Machine") {
      state = j.state;
      record = j.record;
      console.log("State Machine: "+state+", "+record);
    }
    return;
  }
  /* Ping/Pong signal for keeping websocket alive */
  if (evt.data === heartbeat_msg) {
    /* reset the counter for missed heartbeats */
    console.log('pong : '+evt.data);
    missed_heartbeats = 0;
    return;
  }
  if (evt.data[2] != "m") {
    // console.log(evt+" , "+evt.data);
    output.src = window.URL.createObjectURL(evt.data);
    output.play();
    __log('play audio');
    output.onended = function() {
      if (record) {
        PlayBeepSound(1, 2);
        document.getElementById('recordBtn').click();
      }else {
        __log('Pass '+state+' to socket server');
        __mlog('Strategy Requesting...');
        socket.emit('message', state);
        state = 0;
      }
    }
  }
};

function IsJsonString (json) {
  var str = json.toString();
  try {
    JSON.parse(str);
  }catch (e) {
    return false;
  }
  return true;
}
function Sayit (string) {
  string = '--Says:--'+string; // Add check code
  ws.send(string);
  console.log("Let robot says: "+string);
  __log("Let robot says: "+string);
  __mlog('TTS Requesting...');
}

function SendOrder() {
  __mlog('Strategy Requesting...');
  __log('Pass 3 to socket server');
  socket.emit('message', 3);
}