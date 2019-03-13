/*========================================================*/
/*
    scan_black node
 */
/*--------------------------------------------------------*/

const Init_ScanBlack_Param = async () => {
    try {
        const param = await paramScanBlack.Get();
        var obj = document.getElementsByName("ScanElement");
        if (param != null) {
            for (var item in param) {
                switch (item) {
                    case "middleY":
                        obj[0].value = parseInt(param.middleY);
                        obj[1].value = parseInt(param.middleY);
                        break;
                    case "range":
                        obj[2].value = parseInt(param.range);
                        obj[3].value = parseInt(param.range);
                        break;
                    case "threshold":
                        obj[4].value = parseInt(param.threshold);
                        obj[5].value = parseInt(param.threshold);
                        break;
                    case "weight":
                        obj[6].value = parseInt(param.weight);
                        obj[7].value = parseInt(param.weight);
                        break;
                    default:
                        break;
                }
            }
        } else {
            console.log("SCAN_PARAM : default");
        }
    } catch (err) {
        console.log(err);
    }
};

/*========================================================*/
/*
    Strategy node
 */
/*--------------------------------------------------------*/

const Init_Strategy_Param = async () => {
    try {
        const param = await paramStrategy.Get();
        var obj = document.getElementsByName("StrategyElement");
        if (param != null) {
            for (var item in param) {
                switch (item) {
                    case "minVel":
                        obj[0].value = parseFloat(param.minVel);
                        break;
                    case "velYaw":
                        obj[1].value = parseFloat(param.velYaw);
                        break;
                    case "rotateYaw":
                        obj[2].value = parseFloat(param.rotateYaw);
                        break;
                    case "crossTime":
                        obj[3].value = parseFloat(param.crossTime);
                        break;
                    case "errorRotate0":
                        obj[4].value = parseFloat(param.errorRotate0);
                        break;
                    case "errorRotate90":
                        obj[5].value = parseFloat(param.errorRotate90);
                        break;
                    case "rotateSlowAng":
                        obj[6].value = parseFloat(param.rotateSlowAng);
                        break;
                    case "errorAng":
                        obj[7].value = parseFloat(param.errorAng);
                        break;
                    case "errorMoibledis":
                        obj[8].value = parseFloat(param.errorMoibledis);
                        break;
                    case "errorMoibleAng":
                        obj[9].value = parseFloat(param.errorMoibleAng);
                        break;
                    case "errorCorrectionDis":
                        obj[10].value = parseFloat(param.errorCorrectionDis);
                        break;
                    default:
                        break;
                }
            }
        } else {
            console.log("Strategy_PARAM : default");
        }
    } catch (err) {
        console.log(err);
    }
};

function Set_Strategy_Param() {
    var obj = document.getElementsByName("StrategyElement");

    for (var i = 0; i < obj.length; i++) {
        switch (i) {
            case 0:
                paramMinVel.Set(parseFloat(obj[i].value));
                break;
            case 1:
                paramVelYaw.Set(parseFloat(obj[i].value));
                break;
            case 2:
                paramRotateYaw.Set(parseFloat(obj[i].value));
                break;
            case 3:
                paramCrossTime.Set(parseFloat(obj[i].value));
                break;
            case 4:
                paramErrorRotate0.Set(parseFloat(obj[i].value));
                break;
            case 5:
                paramErrorRotate90.Set(parseFloat(obj[i].value));
                break;
            case 6:
                paramRotateSlowAng.Set(parseFloat(obj[i].value));
                break;
            case 7:
                paramErrorAng.Set(parseFloat(obj[i].value));
                break;
            case 8:
                paramErrorMoibledis.Set(parseFloat(obj[i].value));
                break;
            case 9:
                paramErrorMoibleAng.Set(parseFloat(obj[i].value));
                break;
            case 10:
                paramErrorCorrectionDis.Set(parseFloat(obj[i].value));
                break;
            default:
                break;
        }
    }
}