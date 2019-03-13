/* node list

    motion node
        pub:
            /motion/remote
            /motion/cmd_vel
        sub:

    stratgy node
        pub:
        sub:
    scan_black node
        pub:
        sub:
    qrcode node
        pub:
        sub
*/
/*========================================================*/
/*
    Motion node
 */
/*--------------------------------------------------------*/
//MotionRemote
function Init_Manual(){
    var msg = new ROSLIB.Message({
        data: false
    });
    topicRemote.Pub(msg);
}

function Manual_button(){
    var obj = document.getElementById("manual_button");
    var checked = document.getElementById("manual_button_checked").checked;

    if(checked){
        obj.style.cssText="color:#efbf67;";
        var msg = new ROSLIB.Message({
            data: true
        });
    }else{
        obj.style.cssText="color:#000000;";
        var msg = new ROSLIB.Message({
            data: false
        });
    }
    topicRemote.Pub(msg);
    // console.log(checked);
}
/*--------------------------------------------------------*/
// cmd_vel

function Robot_Vel(vec) {
    var checked = document.getElementById("manual_button_checked").checked;
    var msg = new ROSLIB.Message({
        linear: {
            x: vec.y,
            y: -vec.x,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: vec.z
        }
    });
    if(checked){
        console.log(msg.linear.x,msg.linear.y,msg.angular.z);
        topicCmdvel.Pub(msg);
    }else{
        console.log("don't start up Manual!!!!")
    }
    
}

function Robot_Stop() {
    var msg = new ROSLIB.Message({
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
    topicCmdvel.Pub(msg);
}
/*========================================================*/
/*
    Strategy node
 */
/*--------------------------------------------------------*/
// start
document.getElementsByName("strategy_state")[0].addEventListener("click", function(){
    console.log('Start');
    var msg = new ROSLIB.Message({
        data: true
    });
    topicStart.Pub(msg)
});
document.getElementsByName("strategy_state")[1].addEventListener("click", function(){
    console.log('Stop');
    var msg = new ROSLIB.Message({
        data: false
    });
    topicStart.Pub(msg)
});
/*--------------------------------------------------------*/
// behavior
document.getElementsByName("strategy_button")[0].addEventListener("click", function(){
    console.log('INIT');
    var msg = new ROSLIB.Message({
        data: 10
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[1].addEventListener("click", function(){
    console.log('MOBILE_ROBOT');
    var msg = new ROSLIB.Message({
        data: 0
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[2].addEventListener("click", function(){
    console.log('platform');
    var msg = new ROSLIB.Message({
        data: 2
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[3].addEventListener("click", function(){
    console.log('NEXT_POINT');
    var msg = new ROSLIB.Message({
        data: 3
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[4].addEventListener("click", function(){
    console.log('DELIVERY');
    var msg = new ROSLIB.Message({
        data: 11
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[5].addEventListener("click", function(){
    console.log('ORDER');
    var msg = new ROSLIB.Message({
        data: 12
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[6].addEventListener("click", function(){
    console.log('Home');
    var msg = new ROSLIB.Message({
        data: 4
    });
    topicBehavior.Pub(msg);
});
document.getElementsByName("strategy_button")[7].addEventListener("click", function(){
    console.log('MANUAL');
    var msg = new ROSLIB.Message({
        data: 5
    });
    topicBehavior.Pub(msg);
});

/*========================================================*/
/*
    scan_black node
 */
/*--------------------------------------------------------*/
/* param */

// middleY
document.getElementsByName("ScanElement")[0].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanMiddleY.Pub(msg);
});
document.getElementsByName("ScanElement")[1].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanMiddleY.Pub(msg);
});

// range
document.getElementsByName("ScanElement")[2].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanRange.Pub(msg);
});
document.getElementsByName("ScanElement")[3].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanRange.Pub(msg);
});

// threshold
document.getElementsByName("ScanElement")[4].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanThreshold.Pub(msg);
});
document.getElementsByName("ScanElement")[5].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanThreshold.Pub(msg);
});

// weight
document.getElementsByName("ScanElement")[6].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanWeight.Pub(msg);
});
document.getElementsByName("ScanElement")[7].addEventListener("input", function(){
    var msg = new ROSLIB.Message({
        data: parseInt(this.value)
    });
    topicScanWeight.Pub(msg);
});

/*--------------------------------------------------------*/
/* save */

document.getElementById("scan_save").addEventListener("click", function(){
    console.log("SCAN SAVE")
    var msg = new ROSLIB.Message({
        data: true
    });
    topicScanSave.Pub(msg);
});

document.getElementById("strategy_save").addEventListener("click", function(){
    console.log("STRATEGY SAVE")
    Set_Strategy_Param();
    var msg = new ROSLIB.Message({
        data: true
    });
    topicStrategySave.Pub(msg);
});
