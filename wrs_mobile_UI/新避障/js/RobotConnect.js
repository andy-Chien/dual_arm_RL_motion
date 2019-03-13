//ROS_connect
var light;
var light = document.getElementById("Light");


if (typeof(Storage) !== "undefined") {
    if (localStorage.getItem("IP") != null) {
        document.getElementById("RobotIP").value = localStorage.getItem("IP");
    } else {
        document.getElementById("RobotIP").value = "localhost";
        localStorage.IP = "localhost";
    }
    if (localStorage.getItem("Host") != null) {
        document.getElementById("RobotHost").value = localStorage.getItem("Host");
    } else {
        document.getElementById("RobotHost").value = "9090";
        localStorage.Host = "9090"
    }
} else {
    console.log('Sorry, your browser does not support Web Storage...');
}


//Robot_connnet
var ros = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP").value + ':' + document.getElementById("RobotHost").value
});

//confirm_connect
ros.on('connection', function() {
    console.log('Robot1 Connected to websocket server.');
    light = "connected";
    document.getElementById("Light").src = "img/light1.png";
    CheckIP[0] = 1;
});
ros.on('error', function(error) {
    console.log('Robot1 Error connecting to websocket server:');
    light = "disconnected";
    document.getElementById("Light").src = "img/light2.png";
    CheckIP[0] = 0;
});
ros.on('close', function() {
    console.log('Robot1 Connection to websocket server closed.');
    light = "disconnected";
    document.getElementById("Light").src = "img/light2.png";
    CheckIP[0] = 0;
});


//When Connect botton is pressed
function RobotConnect() {
    var IP = document.getElementById("RobotIP").value;
    if (IP != '') {
        localStorage.IP = IP;
    } else {
        if (localStorage.getItem("IP") != null) {
            IP = localStorage.getItem("IP");
        } else {
            IP = "localhost";
            localStorage.IP = "localhost";
        }
    }
    
    
    var Host = document.getElementById("RobotHost").value;
    if (Host != '') {
        localStorage.Host = Host;
    } else {
        if (localStorage.getItem("Host") != null) {
            Host = localStorage.getItem("Host");
        } else {
            Host = "9090";
            localStorage.Host = "9090";
        }
    }
   

    ros = new ROSLIB.Ros({
        url: 'ws://' + IP + ':' + Host
    });

    //confirm_connect
    ros.on('connection', function() {
        console.log('Robot1 Connected to websocket server.');
        light = "connected";
        document.getElementById("Light").src = "img/light1.png"
        CheckIP[0] = 1;
    });
    ros.on('error', function(error) {
        console.log('Robot1 Error connecting to websocket server:');
        light = "disconnected";
        document.getElementById("Light").src = "img/light2.png"
        CheckIP[0] = 0;
    });
    ros.on('close', function() {
        console.log('Robot1 Connection to websocket server closed.');
        light = "disconnected";
        document.getElementById("Light").src = "img/light2.png"
        CheckIP[0] = 0;
    });

    //SetParamRobotNum();
    //up();
}

function RobotCloseConnect() {
    ros.close();
}
