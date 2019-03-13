function Init(){
    console.log("init");
    joystick_main();

    setTimeout(function(){
        Init_Manual();
    },1000);

    setTimeout(function(){
        Init_ScanBlack_Param();
    },500);

    setTimeout(function(){
        Init_Strategy_Param();
    },500);
}