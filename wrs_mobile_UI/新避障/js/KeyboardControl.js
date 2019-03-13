window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

$(function($) {
    $('#KeyboardControl').famultibutton();
    $("#KeyboardControl").click(function() {
        if (this.value)
            KeyboardState(0);
        else
            KeyboardState(1);
    });
});

function KeyboardState(state) {
    obj = document.getElementById("KeyboardControl");
    obj.value = state;
    KeyboardStart = state;
}

function keysdown(e) {
    if (KeyboardStart == true) {
        var vec3;
        keys[e.keyCode] = true;

        //direction
        if (keys[38] && keys[39]) { //右前
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[38] && keys[37]) { //左前
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[40] && keys[39]) { //右後
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[40] && keys[37]) { //左後
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[38]) { //前
            vec3 = new ROSLIB.Message({
                x: 0,
                y: parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[39]) { //右
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[40]) { //後
            vec3 = new ROSLIB.Message({
                x: 0,
                y: -parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
        } else if (keys[37]) { //左
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
        }
        // else if (keys[69]) {
        //     if (speed > 30)
        //         speed = speed * 0.5;
        //     vec3 = new ROSLIB.Message({
        //         x: 0,
        //         y: 0,
        //         z: -parseFloat(speed)
        //     });
        //     PublishTopicCmdVel(vec3);
        //     PublishTopicCmdVel(vec3);
        // } else if (keys[81]) {
        //     if (speed > 30)
        //         speed = speed * 0.5;
        //     vec3 = new ROSLIB.Message({
        //         x: 0,
        //         y: 0,
        //         z: parseFloat(speed)
        //     });
        //     PublishTopicCmdVel(vec3);
        //     PublishTopicCmdVel(vec3);
        // }

        //SwitchRobot
        if (keys[80]) {
            ParamAvoidGo(0);
            StrategyStop();
        } else if (keys[79]) {
            ParamAvoidGo(1);
        }
    }
}

function releasebutton() {

}

function keyuped(e) {
    if (KeyboardStart) {
        if (keys[81] == true) releasebutton();
        else if (keys[69] == true) releasebutton();
        else if (keys[87] == true) releasebutton();
        else if (keys[65] == true) releasebutton();
        else if (keys[83] == true) releasebutton();
        else if (keys[68] == true) releasebutton();
        keys[e.keyCode] = false;
    }
}