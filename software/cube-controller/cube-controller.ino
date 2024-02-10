// Toggle over-the-air (WiFi) programming functionality (0 = disabled, 1 = enabled)
#define OTA 1

// Toggle between use of nonlinear or linear controller (0 = linear, 1 = nonlinear)
#define USE_NONLINEAR_CONTROLLER 1

// TickTwo provides ESP32-compatible timers
#include <TickTwo.h>

#if OTA
  // Libraries related to wireless functionality
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

// Cube controller library
#include "src/cube-controller.h"

#if OTA
  // WiFi settings
  const char* wifi_ssid = "YOUR_SSID";
  const char* wifi_password = "YOUR_PASSWORD";
#endif

// Instantiation of objects
Motor motor1(M1_ENABLE, M1_CURRENT), motor2(M2_ENABLE, M2_CURRENT), motor3(M3_ENABLE, M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA, IMU_SCL);
AttitudeWheelController cont(USE_NONLINEAR_CONTROLLER);
AttitudeTrajectory att_tra;

// Run cube controller at the frequency as specified in parameter file
void controller();
TickTwo timer(controller, dt_us, 0, MICROS_MICROS);

// Status LED control: LED is either solid or blinking at 2.5 Hz
void control_led();
TickTwo timer_led(control_led, 200, 0, MILLIS);

#if OTA
  // Handle OTA flashing requests every 5 seconds
  void handle_ota();
  TickTwo timer_ota(handle_ota, 5000, 0, MILLIS);
#endif

// Quaternion and angle error
float qe0, qe1, qe2, qe3;
float phi;
float phi_lim = phi_min;

// Trajectory initialization flag
bool flag_tra = false;

// LED status when blinking
bool led_status = false;

// Security flags
bool flag_arm = false;
bool flag_terminate = false;

// Torques
float tau_1 = 0, tau_2 = 0, tau_3 = 0;

void setup() {
  // Open serial connection
  Serial.begin(921600);
  Serial.println("This is the ESP32 cube controller.");

  #if OTA
    // Set up WiFi connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    unsigned int nAttempts = 0;
    while (WiFi.waitForConnectResult() != WL_CONNECTED && nAttempts < 3) {
      delay(1000);
      nAttempts++;
    }

    // If WiFi connection failed, indicate this using six yellow flashes
    if(WiFi.waitForConnectResult() != WL_CONNECTED) {
      for(int i = 0; i < 5; i++) {
        neopixelWrite(RGB_BUILTIN, 255, 255, 0);
        delay(250);
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        delay(250);
      }
    }

    // Set hostname
    ArduinoOTA.setHostname("Cube ESP32");

    // Print WiFi status
    Serial.print("Connected to WiFi (local IP is ");
    Serial.print(WiFi.localIP());
    Serial.println(").");

    // Set up OTA flashing using ArduinoOTA
    ArduinoOTA.begin();
  #endif

  // Delay for one second to allow Maxon ESCON drivers to fully initialize
  delay(1000);

  // Motor setup (also spins each motor in positive direction very briefly)
  motor1.init();
  motor2.init();
  motor3.init();

  // Wheel estimator initialization (also calibrates the hall sensor for each wheel)
  whe_est_1.init();
  whe_est_2.init();
  whe_est_3.init();

  // Attitude estimator initialization (also calibrates the gyroscope)
  att_est.init();

  // Start controller and LED timers
  timer.start();
  timer_led.start();

  #if OTA
    // OTA handler timer
    timer_ota.start();
  #endif
}

void loop() {
  // Update controller timer
  timer.update();
  timer_led.update();

  #if OTA
    // Handle OTA updates
    timer_ota.update();
  #endif
}

#if OTA
  void handle_ota(){
    ArduinoOTA.handle();
  }
#endif

void controller() {
  // Estimate wheel velocities
  whe_est_1.estimate(tau_1);
  whe_est_2.estimate(tau_2);
  whe_est_3.estimate(tau_3);

  // Estimate cube attitude
  att_est.estimate();

  // Calculate rotation quaternion error
  qe0 = att_est.q0 * att_tra.qr0 + att_est.q1 * att_tra.qr1 + att_est.q2 * att_tra.qr2 + att_est.q3 * att_tra.qr3;
  qe1 = att_est.q0 * att_tra.qr1 - att_est.q1 * att_tra.qr0 - att_est.q2 * att_tra.qr3 + att_est.q3 * att_tra.qr2;
  qe2 = att_est.q0 * att_tra.qr2 - att_est.q2 * att_tra.qr0 + att_est.q1 * att_tra.qr3 - att_est.q3 * att_tra.qr1;
  qe3 = att_est.q0 * att_tra.qr3 - att_est.q1 * att_tra.qr2 + att_est.q2 * att_tra.qr1 - att_est.q3 * att_tra.qr0;

  // Normalize rotation quaternion error (real part only since we don't need the rest)
  qe0 /= sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);

  // Calculate error angle
  phi = 2.0 * acos(qe0);

  // Controller enable and disable logic. The controller only activates once its orientation is such that the error with
  // respect to the desired orientation is smaller than phi_min. As soon as this is reached, the error tolerance phi_lim
  // is increased to phi_max. If this error is exceeded, the controller is disabled until the chip is reset.
  if(abs(phi) <= phi_lim && !flag_terminate) {
    // Controller is active
    flag_arm = true;

    // Increase error range to the maximum limit
    phi_lim = phi_max;

    // Generate trajectory
    // Comment out the generate() method if the cube is balancing on a side instead of a corner
    if(!flag_tra) {
      flag_tra = true;
      att_tra.init();
    }
    att_tra.generate();

    // Controller calculates motor torques based on cube and wheel states
    cont.control(att_tra.qr0, att_tra.qr1, att_tra.qr2, att_tra.qr3, att_est.q0, att_est.q1, att_est.q2, att_est.q3,
      att_tra.omega_r_x, att_tra.omega_r_y, att_tra.omega_r_z, att_est.omega_x, att_est.omega_y, att_est.omega_z, 
      att_tra.alpha_r_x, att_tra.alpha_r_y, att_tra.alpha_r_z, whe_est_1.theta_w, whe_est_2.theta_w, 
      whe_est_3.theta_w, whe_est_1.omega_w, whe_est_2.omega_w, whe_est_3.omega_w);

    // Get motor torques from controller
    tau_1 = cont.tau_1;
    tau_2 = cont.tau_2;
    tau_3 = cont.tau_3;
  } else {
    // Outside safe limits: reset motor torques to zero
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;

    // Disarm mechanism
    if(flag_arm) {
      flag_arm = false;
      flag_terminate = true;
    }
  }

  // Apply torques to motors
  // When balancing on the x side, comment out motor 2 and 3. When balancing on the y side, comment out 1 and 3.
  motor1.set_torque(tau_1);
  motor2.set_torque(tau_2);
  motor3.set_torque(tau_3);
}

void control_led() {
  if(flag_arm) {
    if(abs(phi) <= phi_lim - 10 * pi / 180) {
      // Cube is not close to error limit: solid green LED
      neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    } else {
      // Cube is close to error limit: alternate between red and green. Cube should be manually put to rest.
      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
      }
      led_status = !led_status;
    }
  } else {
    if(flag_tra) {
      // A trajectory was already generated so the cube must now be halted due to a limit error
      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      }
      led_status = !led_status;
    } else {
      // Show a solid red LED to indicate readiness to start
      neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }
  }
}
