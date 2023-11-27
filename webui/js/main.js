// Create ros object to communicate over your Rosbridge connection
const ros = new ROSLIB.Ros({ url: "ws://10.10.20.152:9090" });

// When the Rosbridge server connects, fill the span with id "status" with "successful"
ros.on("connection", () => {
  // document.getElementById("status").innerHTML = "successful";
});

// When the Rosbridge server experiences an error, fill the "status" span with the returned error
ros.on("error", (error) => {
  // document.getElementById("status").innerHTML = `errored out (${error})`;
});

// When the Rosbridge server shuts down, fill the "status" span with "closed"
ros.on("close", () => {
  // document.getElementById("status").innerHTML = "closed";
});

// Create a listener for /my_topic
const my_topic_listener = new ROSLIB.Topic({
  ros,
  name: "/diff_cont/cmd_vel_unstamped",
  messageType: "geometry_msgs/msg/Twist",
});

// When we receive a message on /my_topic, add its data as a list item to the "messages" ul
my_topic_listener.subscribe((message) => {
  setEye(message);
});
