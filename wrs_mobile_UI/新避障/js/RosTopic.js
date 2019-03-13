/*========================================================*/
//MotionRemote
var Remote = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/remote',
    messageType: 'std_msgs/Bool'
});

function RemoteSwitch(state) {
    var check;
    if (state) {
        RemoteState = true;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    } else {
        RemoteState = false;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    }
    if (CheckIP[0] == 1)
        Remote.publish(check);
}
/*========================================================*/
//old GameState  
//Please New User delete or change Ros Parameter 

var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});

function OldGameState() {
    setTimeout(PublishTopicGameState, 0, 1);
    setTimeout(PublishTopicGameState, 100, 1);
    setTimeout(PublishTopicGameState, 200, 1);
    setTimeout(PublishTopicGameState, 300, 1);
    setTimeout(PublishTopicGameState, 400, 1);
    setTimeout(PublishTopicGameState, 500, 1);
}

function PublishTopicGameState(state) {
    console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    if (CheckIP[0] == 1)
        GameState.publish(gameState);
}

/*========================================================*/
//vector
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});

function StrategyStop() {
    setTimeout(StandBy, 0);
    setTimeout(StandBy, 100);
    setTimeout(StandBy, 200);
    setTimeout(StandBy, 300);
    setTimeout(StandBy, 400);
}

function StandBy() {
    var twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    console.log(twist);
    cmdVel.publish(twist);
}

function PublishTopicCmdVel(vec3) {
    var twist = new ROSLIB.Message({
        linear: {
            x: vec3.x,
            y: vec3.y,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: vec3.z
        }
    });
    if (RemoteState) {
        console.log(twist);
        cmdVel.publish(twist);
    }
}

/*========================================================*/
//IsSimulator
var TopicIsSimulator = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});

function SimulatorFalse() {
    setTimeout(OpenSimulator, 0, false);
    setTimeout(OpenSimulator, 100, false);
    setTimeout(OpenSimulator, 200, false);
    setTimeout(OpenSimulator, 300, false);
    setTimeout(OpenSimulator, 400, false);
}

function OpenSimulator(checked) {
    var temp;
    console.log("IsSimulator", checked);
    if (checked == true) {
        temp = new ROSLIB.Message({
            data: 1
        });
        TopicIsSimulator.publish(temp);
    } else {
        temp = new ROSLIB.Message({
            data: 0
        });
        TopicIsSimulator.publish(temp);
    }
}