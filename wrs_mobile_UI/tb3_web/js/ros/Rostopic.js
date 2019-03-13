//===================================================================
//HSV mode
//color
var TopicColor = new ROSLIB.Topic({
    ros: ros,
    name: '/tb3/color',
    messageType: '/vision/color'
});

function Change_HSVmode() {
    var mode = parseInt(document.getElementById('HSVSelect').value);
    switch (mode) {
        case 0:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = RedBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = RedBox[i];
            }
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = BlueBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = BlueBox[i];
            }
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = YellowBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = YellowBox[i];
            }
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = WhiteBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = WhiteBox[i];
            }
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                document.getElementsByName('HSVElement')[i].value = BlackBox[i];
                document.getElementsByName('HSVLabelElement')[i].innerText = BlackBox[i];
            }
            break;
    }
    Open_Camera('mask');
}

function Set_HSV() {
    var mode = parseInt(document.getElementById('HSVSelect').value);
    switch (mode) {
        case 0:
            for (var i = 0; i < 6; i++) {
                RedBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            if (Topic_HSV_Flag)
                console.log('Topic RedHSV = ' + RedBox);
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
                BlueBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            if (Topic_HSV_Flag)
                console.log('Topic BlueHSV = ' + BlueBox);
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
                YellowBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            if (Topic_HSV_Flag)
                console.log('Topic YellowHSV = ' + YellowBox);
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                WhiteBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            if (Topic_HSV_Flag)
                console.log('Topic WhiteHSV = ' + WhiteBox);
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                BlackBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            if (Topic_HSV_Flag)
                console.log('Topic BlackHSV = ' + BlackBox);
            break;
    }
    Pub_Color(mode);
}

function Pub_Color(mode) {
    var Color = new ROSLIB.Message({
        ColorMode: mode,
        RedHSVBox: RedBox,
        BlueHSVBox: BlueBox,
        YellowHSVBox: YellowBox,
        WhiteHSVBox: WhiteBox,
        BlackHSVBox: BlackBox,
    });
    TopicColor.publish(Color);
}
//===================================================================
var TopicSave = new ROSLIB.Topic({
    ros: ros,
    name: '/tb3/save',
    messageType: 'std_msgs/Int32'
});

function Pub_Save() {
    var save = new ROSLIB.Message({
        data: 1
    });
    TopicSave.publish(save);
}

//===================================================================
var TopicVideoMode = new ROSLIB.Topic({
    ros: ros,
    name: '/tb3/VideoMode',
    messageType: 'std_msgs/Int32'
});

function Pub_VideoMode(num) {
    var videomode = new ROSLIB.Message({
        data: num
    });
    TopicVideoMode.publish(videomode);
}