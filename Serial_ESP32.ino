#include <WiFi.h>
#include <WebServer.h>
#include <ddsm_ctrl.h>
#include <math.h>

// DDSM setup
#define DDSM_RX 18
#define DDSM_TX 19
#define WHEEL_RADIUS 0.0355  // meters (71mm diameter)
#define WHEEL_BASE 0.25      // meters

// Odometry state
float x = 0.0;
float y = 0.0;
float theta = 0.0;
unsigned long lastTime = 0;

DDSM_CTRL dc;

// Global motor speeds
int fr = 0, br = 0, fl = 0, bl = 0;

unsigned long lastPrint = 0;
WebServer server(80);

// Command buffer
String command = "";

// Motor speed
int speed_val = 250;  // 1000 = ~100 RPM depending on config

// HTML for web UI
const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>DDSM Car</title></head>
<body style="text-align:center;">
<h1>SLAM MOBILE ROS</h1>
<button onclick="send('F')">Forward</button><br><br>
<button onclick="send('L')">Left</button>
<button onclick="send('S')">Stop</button>
<button onclick="send('R')">Right</button><br><br>
<button onclick="send('B')">Backward</button>
<script>
function send(dir) {
  console.log("Sending: " + dir);
  fetch("/cmd?dir=" + dir)
    .then(res => res.text())
    .then(txt => console.log("Response: " + txt))
    .catch(err => console.error("Error:", err));
}
</script>
</body>
</html>
)rawliteral";

// Motor control logic
void moveMotors(String dir) {
  if (dir == "F") {           // Forward
    fr = -speed_val; br = -speed_val;
    fl = speed_val;  bl = speed_val;
  } else if (dir == "B") {    // Backward
    fr = speed_val;  br = speed_val;
    fl = -speed_val; bl = -speed_val;
  } else if (dir == "R") {    // Turn Left
    fr = speed_val;  br = speed_val;
    fl = speed_val;  bl = speed_val;
  } else if (dir == "L") {    // Turn Right
    fr = -speed_val; br = -speed_val;
    fl = -speed_val; bl = -speed_val;
  } else if (dir == "S") {    // Stop
    fr = br = fl = bl = 0;
  }

  dc.ddsm_ctrl(1, fr, 2);  // Front Right
  dc.ddsm_ctrl(2, br, 2);  // Back Right
  dc.ddsm_ctrl(3, fl, 2);  // Front Left
  dc.ddsm_ctrl(4, bl, 2);  // Back Left
}

void handleCommand() {
  if (server.hasArg("dir")) {
    command = server.arg("dir");
    Serial.print("Received Command: ");
    Serial.println(command);
    moveMotors(command);
  }
  server.send(200, "text/plain", "OK");
}

void handleRoot() {
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;
  dc.set_ddsm_type(210);
  dc.clear_ddsm_buffer();

  // Set up WiFi AP
  WiFi.softAP("SLAM_MOBILE_ros", "12345678");
  Serial.println("AP Started. Connect to WiFi: SLAM_MOBILE_ros, Pass: 12345678");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Setup web routes
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();
  Serial.println("Web server started.");

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // in seconds
  lastTime = now;

  // Read each motor's RPM
  dc.ddsm_ctrl(1, fr, 2); delay(1); float rpm_fr = dc.speed_data;
  dc.ddsm_ctrl(2, br, 2); delay(1); float rpm_br = dc.speed_data;
  dc.ddsm_ctrl(3, fl, 2); delay(1); float rpm_fl = dc.speed_data;
  dc.ddsm_ctrl(4, bl, 2); delay(1); float rpm_bl = dc.speed_data;

  // Convert RPM to linear velocity
  float v_fr = 2 * PI * WHEEL_RADIUS * rpm_fr / 600.0;
  float v_br = 2 * PI * WHEEL_RADIUS * rpm_br / 600.0;
  float v_fl = 2 * PI * WHEEL_RADIUS * rpm_fl / 600.0;
  float v_bl = 2 * PI * WHEEL_RADIUS * rpm_bl / 600.0;

  // Right and left side average velocities
  float v_right = (v_fr + v_br) / 2.0;
  float v_left  = (v_fl + v_bl) / 2.0;

  // Robot velocities
  float vx = (v_left - v_right) / 2.0;
  float omega = (v_right + v_left) / WHEEL_BASE;

  // Odometry integration
  x += vx * cos(theta) * dt;
  y += vx * sin(theta) * dt;
  theta += omega * dt;

  // Print odometry
  if (now - lastPrint > 1000) {
    Serial.printf("x: %.2f m, y: %.2f m, θ: %.2f rad\n", x, y, theta);
    lastPrint = now;
  }

  server.handleClient();
}
