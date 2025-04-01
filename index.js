//<!-- Goals:
//1) Input for a robot name that defines the topic/robot name you are monitoring
//2) A connect button to initiate the rosbridge connection (192.168.8.104 port 9012)
//3) Follow the path of a robot, I want it to make dots on a map or the path 
//4) The robot will move while a controller is controlling its movements
//--> 

console.clear();
console.log('JS LOADED')

const ip = '192.168.8.104';
//const ip = '127.0.0.1'; //  SIMULATOR IP ADDRESS
const port = 9012;

const connectStat = document.getElementById('connectStat');
const connectBtn = document.getElementById('connectBtn');
const inputVal = document.getElementById('inputVal');
const title = document.getElementById('title');
const intensityDisplay = document.getElementById('intensityDisplay');

// define the function and change the HTML content of the connection
const changeConnection = function (){
  const name = inputVal.value; // this takes value of input
  console.log("BUTTON PRESSED - CONNECTION MADE")
  console.log(name)
  // Get the new connection word and change colors 
  connectStat.style.color = 'green';
  connectStat.textContent = 'Connected';
  title.textContent = name;

  // 1. make a ros instance
  const ros = new ROSLIB.Ros({
    url: `ws://${ip}:${port}`
  });
  
  //2. connect to our rosbridge
  ros.on('connection', ()=>{
    console.log('CONNECTED');
    console.log(`ip: ${ip}`);
  });

  // make a topic for the battery
  battTopic = new ROSLIB.Topic({
    ros: ros,
    name: `/${name}/battery_state`,
    messageType:'sensor_msgs/BatteryState'
    
  });
  // make a subscriber to the battery
  battTopic.subscribe((msg)=>{
      maxVolts = 16.8;
      voltPercent = (msg['voltage'].toFixed(3) / maxVolts) * 100;
      battVolt.textContent = `${voltPercent.toFixed(1)} %`;
  });

  // make a topic for the IR sensor
  // topic for the intensity
  intenseTopic = new ROSLIB.Topic({
      ros: ros,
      name: `${name}/ir_intensity`,
      messageType:'irobot_create_msgs/IrIntensityVector'
      
  }); 
  
  // subscribe to intensity of IR sensor
  intenseTopic.subscribe((ir_msg)=> {
      let ir_readings = ir_msg['readings'].map((r)=>['value']);
      //console.log(ir_msg);
      let max_value = 3000;
      const ir_0 = Math.round((ir_msg.readings[0].value / max_value) * 100);
      const ir_6 = Math.round((ir_msg.readings[6].value / max_value) * 100);
      console.log(ir_0, ir_6)
  });
  // make a topic for the cmd_vel to move to robot
  cmdvelTopic = new ROSLIB.Topic({
    ros: ros,
    name: `/${name}/cmd_vel`,
    messageType:'geometry_msgs/Twist'
    
  });

  // make a publisher function
  function moveFwd() {
    console.log('FWD BUTTON PRESSED');
    // publish message 
    twistMsg = {
      linear:{
        x: 0.5
      }
    }
    cmdvelTopic.publish(twistMsg)
  }

  // make a publisher function
  function turnLeft() {
    console.log('LEFT BUTTON PRESSED');
    // publish message 
    twistMsg = {
      angular:{
        z: 0.5
      }
    }
    cmdvelTopic.publish(twistMsg)
  }

  // make a publisher function
  function turnRight() {
    console.log('RIGHT BUTTON PRESSED');
    // publish message 
    twistMsg = {
      angular:{
        z: -0.5
      }
    }
    cmdvelTopic.publish(twistMsg)
  }

  // button values for directions
  const valX = document.getElementById('valX');
  const fwd = document.getElementById('up');
  const left = document.getElementById('left');
  const right = document.getElementById('right');
  // change the values when the button is pressed
  fwd.addEventListener('click', moveFwd);
  left.addEventListener('click', turnLeft);
  right.addEventListener('click', turnRight);

// make a topic for the cmd_vel to move to robot
  trackOdomTopic = new ROSLIB.Topic({
      ros: ros,
      name: `/${name}/odom`,
      messageType:'nav_msgs/Odometry'
      
    });

  // subscribe to the odom
  trackOdomTopic.subscribe((msg)=>{
    //console.log(msg)
    const position = msg.pose.pose.position; // dictionary in dictionary
    const orientation = msg.pose.pose.orientation;
    
    const y_q = orientation.y;
    const z_q = orientation.z;
    const w_q = orientation.w;

    const yaw = Math.atan2(2 * (w_q * z_q + y_q * 0), 1 - 2 * (y_q * y_q + z_q * z_q));

    valX.textContent = position.x.toFixed(3); // the x value in odom
    valY.textContent = position.y.toFixed(3); // the x value in odom
    
    valZ.textContent = yaw.toFixed(3); // the x value in odom

    // move the robot on the screen
    const container = document.getElementById('trackContainer');
    const contWidth = container.offsetWidth;
    const contHeight = container.offsetHeight;
    const scale = 50;

    let newX = scale * position.x;
    let newY = scale * position.y;
    // Ensure the robot stays within the container's bounds
    newX = Math.min(Math.max(newX, 0), contWidth - robotEl.offsetWidth); // Limit the x position
    newY = Math.min(Math.max(newY, 0), contHeight - robotEl.offsetHeight); // Limit the y position

    robotEl.style.transform = `translate(${newX}px, ${newY}px)`;
  });

  // starting the timere function
  let time_ms = 0; // initial time in ms
  let timer; // interval ID
  let isRunning = false; // tracks if the timer is running 
  const timerEl = document.getElementById('timerEl');
  const stopBtn = document.getElementById('pauseBtn');
  const resetBtn = document.getElementById('resetBtn');
  // function to update the timer display 
  function updateTimerDisplay() {
    const minutes = Math.floor(time_ms / (1000 * 60));
    const seconds = Math.floor((time_ms % (1000 * 60)) / 1000);
    const ms = time_ms % 1000;
    timerEl.textContent = `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}:${ms.toString().padStart(3, '0')}`;
  }
  // start timer function 
  startBtn.addEventListener('click', ()=>{
    console.log('START TIMER');
    isRunning = true;
    // Intervals of time 
      timer = setInterval(()=>{
        //increase time 
        time_ms += 10;
        // units of minutes and seconds
        updateTimerDisplay(); 
      }, 10)
  });

  stopBtn.addEventListener('click', () => {
    if (isRunning) {
      console.log('PAUSE TIMER');
      clearInterval(timer); // stop the timer 
      isRunning = false;
    }
  });

  resetBtn.addEventListener('click', () =>{
    console.log('RESET TIMER');
    clearInterval(timer); // stop the timer 
    time_ms = 0; // reset to 0 
    isRunning = false; 
    updateTimerDisplay();
  })

}
connectBtn.addEventListener('click', changeConnection);

