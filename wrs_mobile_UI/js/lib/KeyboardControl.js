
window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

var keys = [];
var start;

function KeyboardState(state) {
    start = state;
}

document.getElementById("manual_keyboard").addEventListener("click", function(){
    var checked = document.getElementById("manual_keyboard_checked").checked;
    if(checked){
        this.style.cssText="color:#efbf67;";
        KeyboardState(checked);
    }else{
        this.style.cssText="color:#000000;";
        KeyboardState(checked);
    }
});

function keysdown(e) {
    if (start == true) {
        var vec3;
        var speed = document.getElementById("SpeedInput").value;
        keys[e.keyCode] = true;

        //RobotControl
        if (keys[87] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[87] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[83] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[83] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[87]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: parseFloat(speed),
                z: 0
            });
            Robot_Vel(vec3);
           // Robot_Vel(vec3);
        } else if (keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed),
                y: 0,
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[83]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: -parseFloat(speed),
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed),
                y: 0,
                z: 0
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[69]) {
            if (speed > 30)
                speed = speed * 0.5;
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: -parseFloat(speed)
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        } else if (keys[81]) {
            if (speed > 30)
                speed = speed * 0.5;
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: parseFloat(speed)
            });
            Robot_Vel(vec3);
            //Robot_Vel(vec3);
        }
        
    }
}

function releasebutton() {
    // var vec3 = new ROSLIB.Message({
    //     x: 0,
    //     y: 0,
    //     z: 0
    // });
    // Robot_Vel(vec3);
    // Robot_Vel(vec3);
    Robot_Stop();
    Robot_Stop();
}

function keyuped(e) {
    if (start) {
        if (keys[81] == true) releasebutton();
        else if (keys[69] == true) releasebutton();
        else if (keys[87] == true) releasebutton();
        else if (keys[65] == true) releasebutton();
        else if (keys[83] == true) releasebutton();
        else if (keys[68] == true) releasebutton();
        keys[e.keyCode] = false;
    }
}
