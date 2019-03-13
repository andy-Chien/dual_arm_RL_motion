//ROS_Connect
var CheckIP = [0, 0, 0];
var RobNum = [0, 1, 2];

//Ros_CheckParam
var CheckGetParm = 0;

//Arbitrary
var obj;

//Remote_state
var RemoteState = 0;

//Speed
var speed = 30

//Keyboard
var keys = [];
var KeyboardStart = 0; //off 0 start 1

//SageSwitch
var SafeSwitchStart = 0; //off 0 start 1

//Joystick
var joystick_Area = document.getElementById('Joystick');
var joystick_canvas = document.getElementById("Joystick_Canvas");
var joy_ctx = joystick_canvas.getContext("2d");
var Round_r = 105; //var Round_r = 60;
var mouse_click = 0;
var logButton = -1;
var windowWidth = window.innerWidth;
var windowHeight = window.innerHeight;
var windowWidthToHeight = windowWidth / windowHeight;
var joystickcenter = {
    x: joystick_Area.offsetWidth / 2,
    y: joystick_Area.offsetHeight / 2
};
var joystick_V = {
    x: 0,
    y: 0,
    ang: 0
};