function Init() {
    Init_HSV();
    Init_VisionParam();
}

function Init_HSV() {
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
}

function Init_VisionParam() {
    for (var i = 0; i < VisionParam.length; i++) {
        document.getElementsByName('VisionParamElement')[i].value = VisionParam[i];
        document.getElementsByName('VisionParamLabelElement')[i].innerText = VisionParam[i];
    }
}