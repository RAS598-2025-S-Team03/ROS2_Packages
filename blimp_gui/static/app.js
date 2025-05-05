let ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

// Publishers
const modePub = new ROSLIB.Topic({
  ros: ros,
  name: '/mode_select',
  messageType: 'std_msgs/String'
});

const velPub = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist'
});

function sendCmd(linear, angular) {
  const twist = new ROSLIB.Message({
    linear: { x: linear, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angular }
  });
  velPub.publish(twist);
}

document.getElementById('modeToggle').addEventListener('change', (e) => {
  const msg = new ROSLIB.Message({ data: e.target.checked ? "autonomous" : "manual" });
  modePub.publish(msg);
});

function switchCamera(which) {
  let topic = which === "left" ? "/left_cam/image_raw" : "/right_cam/image_raw";
  document.getElementById('camFeed').src = `http://localhost:8080/stream?topic=${topic}`;
}

// Subscriptions
function subscribeSensor(topicName, elementId) {
  const topic = new ROSLIB.Topic({ ros, name: topicName, messageType: 'sensor_msgs/msg/Imu' });
  topic.subscribe((msg) => {
    document.getElementById(elementId).textContent = JSON.stringify(msg);
  });
}

subscribeSensor('/imu', 'imu');
subscribeSensor('/altimeter', 'alt');
subscribeSensor('/pressure', 'pressure');

