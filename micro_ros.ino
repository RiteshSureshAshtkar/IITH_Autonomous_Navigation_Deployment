#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <ddsm_ctrl.h>
#include <time.h>
#include <sys/time.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <sensor_msgs/msg/joint_state.h>

// WiFi credentials (ONLY for NTP sync, then disconnect)
char ssid[] = "IITH-Guest-PWD-IITH@2024";
char password[] = "IITH@2024";

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Motor configuration
DDSM_CTRL dc;
#define DDSM_RX 18
#define DDSM_TX 19
#define NUM_MOTORS 4
const uint8_t motor_ids[NUM_MOTORS] = {1, 2, 3, 4}; // FR, BR, FL, BL

// Robot parameters - CORRECTED for proper SLAM integration
const float WHEEL_RADIUS = 0.0355;  // meters (71mm diameter)  
const float WHEEL_BASE = 0.155;      // meters (distance between left and right wheels)
const float TRACK_WIDTH = 0.25;    // meters (same as wheel base for differential drive)

// Motor control parameters
const int DEFAULT_SPEED = 250;
const float MAX_LINEAR_SPEED = 5.0;
const float MAX_ANGULAR_SPEED = 5.0;

// Odometry variables
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
unsigned long last_odom_time = 0;

// Velocity tracking for proper odometry
float current_linear_x = 0.0;
float current_angular_z = 0.0;

// Motor command variables
int fl = 0, fr = 0, bl = 0, br = 0;
float target_linear_x = 0.0;
float target_angular_z = 0.0;
unsigned long last_cmd_time = 0;

// Safety timeout
const unsigned long CMD_TIMEOUT_MS = 1000;

enum micro_ros_state_t {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// FIXED: Time synchronization with IST timezone
bool time_sync_complete = false;
bool wifi_disconnected_after_ntp = false;
const char* ntp_server = "pool.ntp.org";
const long  gmtOffset_sec = 19800; // IST is UTC+5:30 (5.5 * 3600 = 19800 seconds)
const int   daylightOffset_sec = 0;
int64_t time_offset_ns = 0; // Offset from system time to ROS time

// ROS2 entities
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t odom_timer;
rcl_timer_t tf_timer;
rclc_executor_t executor;

// Publishers and subscribers
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
rcl_publisher_t joint_state_publisher;

// Messages
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform_stamped;
sensor_msgs__msg__JointState joint_state_msg;

// FIXED: Properly typed joint state arrays
rosidl_runtime_c__String joint_names[4];
double joint_positions[4] = {0.0, 0.0, 0.0, 0.0};
double joint_velocities[4] = {0.0, 0.0, 0.0, 0.0};

// Performance monitoring
unsigned long loop_count = 0;
unsigned long last_performance_report = 0;
const unsigned long PERFORMANCE_REPORT_INTERVAL = 10000;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// FIXED: Function declarations
void error_loop();
void destroy_entities();
bool create_entities();
int64_t get_ros_time_ns();
void sync_system_time();
void verify_time_sync();
void setup_message_frames();
void setup_wifi_for_ntp();
void setup_motors();
void cmd_vel_callback(const void * msgin);
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void tf_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void control_motors();
void update_odometry();
void publish_odometry();
void publish_transform();
void publish_joint_states();

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&tf_publisher, &node);
  rcl_publisher_fini(&joint_state_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&odom_timer);
  rcl_timer_fini(&tf_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Create micro-ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 0);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // Create node
  rclc_node_init_default(&node, "ddsm_differential_drive", "", &support);

  // Create subscription
  rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  // Create publishers
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom");

  rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf");

  rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states");

  // Create timers
  rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(50),   // 20Hz control
    control_timer_callback);

  rclc_timer_init_default(
    &odom_timer,
    &support,
    RCL_MS_TO_NS(100),  // 10Hz odometry
    odom_timer_callback);

  rclc_timer_init_default(
    &tf_timer,
    &support,
    RCL_MS_TO_NS(100),  // 10Hz transform
    tf_timer_callback);

  // Create executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &control_timer);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_timer(&executor, &tf_timer);
  
  // Synchronize time with the agent
  rmw_uros_sync_session(1000);

  return true;
}

// FIXED: Consistent ROS time generation
int64_t get_ros_time_ns() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // Convert to nanoseconds with proper epoch time
  int64_t current_time_ns = ((int64_t)tv.tv_sec * 1000000000LL) + ((int64_t)tv.tv_usec * 1000LL);
  
  return current_time_ns;
}

// FIXED: Improved time synchronization with IST timezone and forced disconnect
void sync_system_time() {
  if (!time_sync_complete && WiFi.status() == WL_CONNECTED) {
    Serial.println("Synchronizing system time with NTP (IST timezone)...");
    
    // Configure NTP with IST timezone offset
    configTime(gmtOffset_sec, daylightOffset_sec, ntp_server, "time.nist.gov", "in.pool.ntp.org");
    
    struct tm timeinfo;
    int attempts = 0;
    while (!getLocalTime(&timeinfo) && attempts < 20) {
      Serial.print(".");
      delay(500);
      attempts++;
    }
    
    if (getLocalTime(&timeinfo)) {
      time_sync_complete = true;
      Serial.println("\n✓ System time synchronized successfully with IST timezone");
      Serial.printf("Current IST time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                   timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      
      // Calculate initial time offset for consistency
      struct timeval tv;
      gettimeofday(&tv, NULL);
      time_offset_ns = ((int64_t)tv.tv_sec * 1000000000LL) + ((int64_t)tv.tv_usec * 1000LL);
      
      Serial.printf("Time sync complete. System uptime: %lu ms\n", millis());
    } else {
      Serial.println("\n⚠ NTP sync failed, using system time");
      time_sync_complete = true; // Continue with system time
      
      // Set a basic time offset
      time_offset_ns = (int64_t)millis() * 1000000LL;
    }
    
    // FIXED: Force disconnect WiFi after time sync and prevent reconnection
    Serial.println("Disconnecting WiFi after NTP sync - will use SERIAL micro-ROS only");
    WiFi.disconnect(true);  // true = turn off WiFi
    WiFi.mode(WIFI_OFF);    // Turn off WiFi completely
    delay(1000);
    wifi_disconnected_after_ntp = true;
    Serial.println("✓ WiFi completely disabled - Serial micro-ROS transport only");
  }
}

// FIXED: Add time synchronization check function
void verify_time_sync() {
  if (time_sync_complete) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      Serial.printf("Time check - IST: %02d:%02d:%02d, ROS time: %lld ns\n",
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, get_ros_time_ns());
    }
  }
}

// FIXED: Proper message frame initialization with correct string setup
void setup_message_frames() {
  // Odometry message setup
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
  
  // Transform message setup
  tf_msg.transforms.data = &transform_stamped;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;
  
  transform_stamped.header.frame_id.data = (char*)"odom";
  transform_stamped.header.frame_id.size = strlen("odom");
  transform_stamped.header.frame_id.capacity = transform_stamped.header.frame_id.size + 1;
  
  transform_stamped.child_frame_id.data = (char*)"base_link";
  transform_stamped.child_frame_id.size = strlen("base_link");
  transform_stamped.child_frame_id.capacity = transform_stamped.child_frame_id.size + 1;
  
  // FIXED: Properly initialize joint name strings
  joint_names[0].data = (char*)"front_left_wheel_joint";
  joint_names[0].size = strlen("front_left_wheel_joint");
  joint_names[0].capacity = joint_names[0].size + 1;
  
  joint_names[1].data = (char*)"front_right_wheel_joint";
  joint_names[1].size = strlen("front_right_wheel_joint");
  joint_names[1].capacity = joint_names[1].size + 1;
  
  joint_names[2].data = (char*)"back_left_wheel_joint";
  joint_names[2].size = strlen("back_left_wheel_joint");
  joint_names[2].capacity = joint_names[2].size + 1;
  
  joint_names[3].data = (char*)"back_right_wheel_joint";
  joint_names[3].size = strlen("back_right_wheel_joint");
  joint_names[3].capacity = joint_names[3].size + 1;
  
  // FIXED: Assign properly typed arrays to joint state message
  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = 4;
  joint_state_msg.name.capacity = 4;
  
  joint_state_msg.position.data = joint_positions;
  joint_state_msg.position.size = 4;
  joint_state_msg.position.capacity = 4;
  
  joint_state_msg.velocity.data = joint_velocities;
  joint_state_msg.velocity.size = 4;
  joint_state_msg.velocity.capacity = 4;
}

void setup_wifi_for_ntp() {
  Serial.println("Connecting to WiFi ONLY for NTP time synchronization...");
  Serial.println("WiFi will be disconnected after time sync - using SERIAL micro-ROS only");
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int connection_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && connection_attempts < 30) {
    delay(500);
    Serial.print(".");
    connection_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("✓ Connected to WiFi: %s (for NTP time sync only)\n", ssid);
    Serial.printf("IP address: %s (temporary)\n", WiFi.localIP().toString().c_str());
    Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\n⚠ Failed to connect to WiFi for NTP!");
    Serial.println("Will use system time without NTP sync");
    // Still set time_sync_complete to continue with serial micro-ROS
    time_sync_complete = true;
    wifi_disconnected_after_ntp = true;
  }
}

void setup_motors() {
  Serial1.begin(115200, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;
  dc.set_ddsm_type(210);
  dc.clear_ddsm_buffer();
  
  delay(500);
  Serial.println("Initializing DDSM motors...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.printf("Setting up motor ID: %d\n", motor_ids[i]);
    dc.ddsm_change_mode(motor_ids[i], 2);  // Speed control mode
    delay(200);
    dc.ddsm_ctrl(motor_ids[i], 0, 2);      // Stop motor
    delay(100);
  }
  
  Serial.println("✓ DDSM motors initialized successfully");
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  delay(2000);
  Serial.println("Starting SERIAL-ONLY micro-ROS DDSM Differential Drive...");
  Serial.println("WiFi will be used ONLY for NTP time sync, then disconnected");
  
  setup_motors();
  setup_wifi_for_ntp();
  sync_system_time();
  
  // Verify time synchronization
  delay(1000);
  verify_time_sync();
  
  // FIXED: Initialize micro-ROS with SERIAL transport only
  Serial.println("Initializing micro-ROS with SERIAL transport...");
  set_microros_transports();  // This sets up SERIAL transport
  delay(3000);
  
  // Initialize state
  state = WAITING_AGENT;
  
  // FIXED: Properly initialize message frames and strings
  setup_message_frames();
  
  // Initialize time tracking
  last_odom_time = millis();
  last_performance_report = millis();
  
  Serial.println("✓ SERIAL-ONLY micro-ROS DDSM Differential Drive initialized successfully");
  Serial.println("Transport: SERIAL micro-ROS (WiFi disabled)");
  Serial.println("Publishing on topics: /odom, /tf, /joint_states");
  Serial.println("Subscribing to: /cmd_vel");
  Serial.println("Connect via: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0");
  Serial.println("Time synchronization: IST timezone configured");
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      RCSOFTCHECK(rmw_uros_ping_agent(100, 1));
      if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
        Serial.println("Serial micro-ROS agent available. Creating entities...");
        state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == AGENT_CONNECTED) {
        Serial.println("✓ Connected to SERIAL micro-ROS agent!");
        Serial.println("Publishing on topics: /odom, /tf, /joint_states");
        Serial.println("Subscribing to: /cmd_vel");
        Serial.println("Transport: SERIAL (WiFi disabled)");
        last_odom_time = millis();
        last_performance_report = millis();
      } else {
        Serial.println("Failed to create entities. Retrying...");
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        Serial.println("Serial micro-ROS agent disconnected. Destroying entities...");
        destroy_entities();
        state = AGENT_DISCONNECTED;
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("Attempting to reconnect to SERIAL micro-ROS agent...");
      destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }

  // Performance monitoring
  if (state == AGENT_CONNECTED) {
    loop_count++;
    unsigned long current_millis = millis();
    if (current_millis - last_performance_report > PERFORMANCE_REPORT_INTERVAL) {
      float loops_per_second = (float)loop_count / (PERFORMANCE_REPORT_INTERVAL / 1000.0);
      Serial.printf("Performance: %.1f loops/sec, Free heap: %d bytes\n", 
                    loops_per_second, ESP.getFreeHeap());
      Serial.printf("Robot pose: x=%.3f y=%.3f θ=%.3f°\n", 
                    robot_x, robot_y, robot_theta * 180.0 / PI);
      Serial.printf("Transport: SERIAL micro-ROS (WiFi: %s)\n", 
                    wifi_disconnected_after_ntp ? "DISABLED" : "ENABLED");
      
      // Verify time synchronization periodically
      verify_time_sync();
      
      loop_count = 0;
      last_performance_report = current_millis;
    }
  }
  
  delay(1);
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  target_linear_x = msg->linear.x;
  target_angular_z = msg->angular.z;
  
  // Constrain velocities to safe limits
  target_linear_x = constrain(target_linear_x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
  target_angular_z = constrain(target_angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
  
  last_cmd_time = millis();
  
  // Visual feedback
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  // Debug output
  static unsigned long last_cmd_debug = 0;
  if (millis() - last_cmd_debug > 1000) {
    Serial.printf("CMD: linear=%.3f angular=%.3f (via SERIAL)\n", target_linear_x, target_angular_z);
    last_cmd_debug = millis();
  }
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // Safety timeout check
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
      target_linear_x = 0.0;
      target_angular_z = 0.0;
    }
    control_motors();
  }
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    update_odometry();
    publish_odometry();
    publish_joint_states();
  }
}

void tf_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    publish_transform();
  }
}

// FIXED: Using corrected motor control logic
void control_motors() {
  // Calculate wheel velocities using differential drive kinematics
   float left_wheel_velocity = target_linear_x - (target_angular_z * WHEEL_BASE / 2.0);
  float right_wheel_velocity = target_linear_x + (target_angular_z * WHEEL_BASE / 2.0);
  
  // Convert to motor commands using corrected mapping from second .ino file
  float speed_scale = DEFAULT_SPEED / MAX_LINEAR_SPEED;
  
  // CORRECTED: Motor control logic from working .ino file
  if (target_linear_x > 0 && abs(target_angular_z) < 0.1) {
    // Forward movement
    fr = -DEFAULT_SPEED; br = -DEFAULT_SPEED;
    fl = DEFAULT_SPEED;  bl = DEFAULT_SPEED;
  } else if (target_linear_x < 0 && abs(target_angular_z) < 0.1) {
    // Backward movement
    fr = DEFAULT_SPEED;  br = DEFAULT_SPEED;
    fl = -DEFAULT_SPEED; bl = -DEFAULT_SPEED;
  } else if (target_angular_z > 0.1 && abs(target_linear_x) < 0.1) {
    // Turn Left (counterclockwise)
    fr = -DEFAULT_SPEED; br = -DEFAULT_SPEED;
    fl = -DEFAULT_SPEED; bl = -DEFAULT_SPEED;
  } else if (target_angular_z < -0.1 && abs(target_linear_x) < 0.1) {
    // Turn Right (clockwise)
    fr = DEFAULT_SPEED;  br = DEFAULT_SPEED;
    fl = DEFAULT_SPEED;  bl = DEFAULT_SPEED;
  } else if (abs(target_linear_x) < 0.1 && abs(target_angular_z) < 0.1) {
    // Stop
    fr = br = fl = bl = 0;
  } else {
    // Combined movement - use differential drive kinematics
    fl = left_wheel_velocity * speed_scale;
    bl = left_wheel_velocity * speed_scale;
    fr = -right_wheel_velocity * speed_scale;
    br = -right_wheel_velocity * speed_scale;
  }
  
  // Constrain motor commands
  fr = constrain(fr, -400, 400);
  br = constrain(br, -400, 400);
  fl = constrain(fl, -400, 400);
  bl = constrain(bl, -400, 400);
  
  // Send commands to motors with correct ID mapping
  dc.ddsm_ctrl(motor_ids[0], fl, 2);  // Motor ID 1 -> Front Right
  dc.ddsm_ctrl(motor_ids[1], bl, 2);  // Motor ID 2 -> Back Right
  dc.ddsm_ctrl(motor_ids[2], fr, 2);  // Motor ID 3 -> Front Left
  dc.ddsm_ctrl(motor_ids[3], br, 2);  // Motor ID 4 -> Back Left
  
  // Store current commands for odometry
  current_linear_x = target_linear_x;
  current_angular_z = target_angular_z;
}



static float prev_x = 0.0;
static float prev_y = 0.0;
static float prev_theta = 0.0;
static unsigned long prev_time = 0;

void update_odometry() {
  unsigned long current_time = millis();
  float dt = (current_time - last_odom_time) / 1000.0;
  
  if (dt <= 0.05 || last_odom_time == 0) {
    last_odom_time = current_time;
    return;
  }
  
  // More efficient motor reading
  dc.clear_ddsm_buffer();
  
  // Batch read motor RPMs
  float rpm_fl, rpm_fr, rpm_bl, rpm_br;
  dc.ddsm_ctrl(motor_ids[1], fl, 2); rpm_fl = dc.speed_data;
  dc.ddsm_ctrl(motor_ids[3], fr, 2); rpm_fr = dc.speed_data;
  dc.ddsm_ctrl(motor_ids[0], bl, 2); rpm_bl = dc.speed_data;
  dc.ddsm_ctrl(motor_ids[2], br, 2); rpm_br = dc.speed_data;
  
  // Convert RPM to velocity with improved formula
  const float rpm_to_vel = 2 * PI * WHEEL_RADIUS / 60.0;
  float v_fl = rpm_fl * rpm_to_vel;
  float v_fr = -rpm_fr * rpm_to_vel;
  float v_bl = rpm_bl * rpm_to_vel;
  float v_br = -rpm_br * rpm_to_vel;
  
  // Average velocities
  float v_left = (v_fl + v_bl) * 0.5;
  float v_right = (v_fr + v_br) * 0.5;
  
  // Robot linear velocity (for position integration)
  float linear_velocity = (v_left + v_right) * 0.5;
  
  // Update position using linear velocity
  float cos_theta = cos(robot_theta);
  float sin_theta = sin(robot_theta);
  
  float delta_x = linear_velocity * cos_theta * dt;
  float delta_y = linear_velocity * sin_theta * dt;
  
  robot_x += delta_x;
  robot_y += delta_y;
  
  // Calculate angular velocity from position change (like your ROS2 node)
  float angular_velocity = 0.0;
  if (prev_time > 0 && dt > 0) {
    float dx = robot_x - prev_x;
    float dy = robot_y - prev_y;
    float dtheta = robot_theta - prev_theta;
    
    // Normalize dtheta to handle wraparound
    while (dtheta > PI) dtheta -= 2.0 * PI;
    while (dtheta < -PI) dtheta += 2.0 * PI;
    
    angular_velocity = dtheta / dt;
    
    // Apply noise filtering to angular velocity
    if (abs(angular_velocity) < 0.05) {  // ~2.9 degrees/s threshold
      angular_velocity = 0.0;
    }
    
    // Additional check: if position change is very small, assume stationary
    float position_change = sqrt(dx*dx + dy*dy);
    if (position_change < 0.01) {  // Less than 1cm movement
      angular_velocity = 0.0;
    }
  }
  
  // Update theta using filtered angular velocity
  float delta_theta = angular_velocity * dt;
  robot_theta += delta_theta;
  
  // Normalize theta
  while (robot_theta > PI) robot_theta -= 2.0 * PI;
  while (robot_theta < -PI) robot_theta += 2.0 * PI;
  
  // Store previous values for next iteration
  prev_x = robot_x;
  prev_y = robot_y;
  prev_theta = robot_theta;
  prev_time = current_time;
  last_odom_time = current_time;
  
  // Less frequent debug output
  if (current_time % 10000 < 100) { // Every 10 seconds
    Serial.print("Odom: x=");
    Serial.print(robot_x, 3);
    Serial.print("m y=");
    Serial.print(robot_y, 3);
    Serial.print("m θ=");
    Serial.print(robot_theta, 3);
    Serial.print("rad v=");
    Serial.print(linear_velocity, 2);
    Serial.print("m/s ω=");
    Serial.print(angular_velocity, 2);
    Serial.println("rad/s");
  }

  
  // FIXED: Update joint positions for visualization (using double type)
  joint_positions[0] += v_fl * dt / WHEEL_RADIUS; // FL wheel
  joint_positions[1] += v_fr * dt / WHEEL_RADIUS; // FR wheel  
  joint_positions[2] += v_bl * dt / WHEEL_RADIUS; // BL wheel
  joint_positions[3] += v_br * dt / WHEEL_RADIUS; // BR wheel
  
  // FIXED: Update joint velocities (using double type)
  joint_velocities[0] = v_fl / WHEEL_RADIUS; // FL wheel
  joint_velocities[1] = v_fr / WHEEL_RADIUS; // FR wheel
  joint_velocities[2] = v_bl / WHEEL_RADIUS; // BL wheel
  joint_velocities[3] = v_br / WHEEL_RADIUS; // BR wheel
  
  
  
}

// FIXED: Improved odometry publishing with proper covariance and consistent timestamps
void publish_odometry() {
  // Use synchronized ROS time with consistent timestamp generation
  int64_t time_ns = get_ros_time_ns();
  odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
  
  // Position
  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.position.z = 0.0;
  
  // FIXED: Proper quaternion calculation
  float half_yaw = robot_theta * 0.5;
  odom_msg.pose.pose.orientation.w = cos(half_yaw);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(half_yaw);
  
  // FIXED: Use current command velocities for twist (more accurate for SLAM)
  odom_msg.twist.twist.linear.x = current_linear_x;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = current_angular_z;
  
  // FIXED: SLAM-optimized covariance matrices
  memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
  memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
  
  
  // Conservative pose covariance for SLAM (36 elements, 6x6 matrix)
  odom_msg.pose.covariance[0] = 0.005;   // x-x
  odom_msg.pose.covariance[7] = 0.005;   // y-y
  odom_msg.pose.covariance[14] = 1e6;    // z-z (not used, high uncertainty)
  odom_msg.pose.covariance[21] = 1e6;    // roll-roll (not used)
  odom_msg.pose.covariance[28] = 1e6;    // pitch-pitch (not used)
  odom_msg.pose.covariance[35] = 0.02;   // yaw-yaw
  
  // Twist covariance
  odom_msg.twist.covariance[0] = 0.001;  // vx-vx
  odom_msg.twist.covariance[7] = 1e6;    // vy-vy (not used)
  odom_msg.twist.covariance[14] = 1e6;   // vz-vz (not used)
  odom_msg.twist.covariance[21] = 1e6;   // wx-wx (not used)
  odom_msg.twist.covariance[28] = 1e6;   // wy-wy (not used)
  odom_msg.twist.covariance[35] = 0.01;  // wz-wz
  
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}



// FIXED: Synchronized transform publishing
void publish_transform() {
  // Use same timestamp as odometry for perfect synchronization
  int64_t time_ns = get_ros_time_ns();
  transform_stamped.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
  transform_stamped.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
  
  // Transform translation
  transform_stamped.transform.translation.x = robot_x;
  transform_stamped.transform.translation.y = robot_y;
  transform_stamped.transform.translation.z = 0.0;
  
  // Transform rotation (quaternion)
  float half_yaw = robot_theta * 0.5;
  transform_stamped.transform.rotation.w = cos(half_yaw);
  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = sin(half_yaw);
  
  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
}

// FIXED: Joint state publishing with proper header
void publish_joint_states() {
  // Use same timestamp for synchronization
  int64_t time_ns = get_ros_time_ns();
  joint_state_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
  joint_state_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
  
  // Joint state data is already updated in update_odometry()
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
}
