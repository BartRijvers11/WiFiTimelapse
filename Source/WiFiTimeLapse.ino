/*
 * New software: Connect IO0 with GND & press reset / power-up. Disable (external) terminal program and upload from Arduino IDE
 * For ESP-WROOM-32 choose FIreBeetle-ESP32, see https://techtutorialsx.com/2017/06/05/esp-wroom-32-uploading-a-program-with-arduino-ide/
 * Based on WiFiAccessPoint example in Arduino IDE
 * https://randomnerdtutorials.com/esp32-servo-motor-web-server-arduino-ide/
*/

#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <TinyStepper_28BYJ_48.h> // Stepper motor driver
#include <Adafruit_SSD1306.h> // Display driver 128x64 pixels
#include <Fonts/FreeSans9pt7b.h>

const GFXfont *font = &FreeSans9pt7b;

// TODO: Brownout verhelpen met juiste capacitor
// TODO: test WiFi control invloed op rotatie: snel roteren met knijper en veel netwerk verkeer, wifi logoff / logon: kijken of migreren van motor movement taak naar andere CPU helpt
// TODO: FUTURE fast movement between images when using external trigger, Canon 6D Mk2 does not use external trigger in 

typedef enum {
  DISPLAY_Splash = 0,
  DISPLAY_WiFiLogin,        // AP SSID & Password
  DISPLAY_WiFiShowIP,       // Connected to AP, show IP
  DISPLAY_PotSetupDegrees,  // Setup rotation degrees using pot meter
  DISPLAY_PotSetupDuration, // Setup duration using pot meter
  DISPLAY_PotSetupAccel,    // Setup accelleration using pot meter
  DISPLAY_PotWaitStart,     // All setup, wait to start
  DISPLAY_WiFiSetup,        // User is setting up the timelapse via web interface
  DISPLAY_ShowWaitStatus,   // Show pre-rotation delay status
  DISPLAY_ShowStatus1,      // Show 1st status screen
  DISPLAY_ShowStatus2,      // Show 2nd status screen
  DISPLAY_ShowStatus3,      // Show 3rd status screen
  DISPLAY_ShowStatus4,      // Show 4th status screen
  DISPLAY_Number            // Placeholer for number of display states (keep last)
} display_state_t;

#define DISPLAY_HOME_X  0   // Start coordinate for text of FreeSans9pt font
#define DISPLAY_HOME_Y  14
#define DISPLAY_WIFI_X  119 // Start coordinate of WiFi sign in display
#define DISPLAY_WIFI_Y  8
#define DISPLAY_STATUS_REFRESH 300
#define DISPLAY_STATUS_TIME_PER_SCREEN (3600 / DISPLAY_STATUS_REFRESH)

#define TASK_KICK_WATCHDOG()  vTaskDelay(portTICK_PERIOD_MS);  // Delay not desired but use here to kick task watchdog (in idle callback), portTICK_PERIOD_MS is the smallest delay time to handle this
#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#define OLED_RESET -1

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Runtime stuff
typedef struct {
  float tripodSteps; // in steps
  float motorSpeed; // in steps per sec
  float motorAccelleration; // in steps per sec^2
} motor_runtime_t;

typedef struct {
  float tripodPulley; // Relation of diameter tripod / pulley
  int tripodDegrees; // degrees default
  int rotationDuration; // seconds, default half an hour
  int motorAccelSeconds; // Accellerate in number of seconds
  float stepperSpeed; // steps per second in manual mode (using STEPS_PER_REVOLUTION is full round in 1 second)
  int cameraInterval; // Interval at which the camera takes pictures for the timelapse
  int playbackFPS; // normal playback framerate
  float tripodDiameter; // The entered tripod diam. For (re-) setup purpose, the tripodPulley member is derived from this and used in app
  float pulleyDiameter; // The entered pulley diam. For (re-) setup purpose, the tripodPulley member is derived from this and used in app
  int preDelay; // Delay in seconds to wait before rotation
} settings_t;

#define EEPROM_SIZE sizeof(settings)

const int MOTOR_IN1_PIN = 26;
const int MOTOR_IN2_PIN = 18;
const int MOTOR_IN3_PIN = 19;
const int MOTOR_IN4_PIN = 23;
const int ANALOG_POT_PIN = 34;  // Analog input pin on SVP label of WROOM mini module

const int STEPS_PER_REVOLUTION = 2048; // Applies to my stepper motor

// Set these to your desired credentials.
const char *ssid = "Timelapse";
const char *password = "L@pseT!me";
WiFiServer server(80); // Port 80 is for http standard
IPAddress myIP;
bool wifiStarted = false; // If WiFi AP is initialized, only used at startup
bool wifiConnected = false; // If a client is connected or not
bool wifiPrevConnected = false; // for display updating
String header; // Variable to store the HTTP request

TinyStepper_28BYJ_48 stepper;
Adafruit_SSD1306 display(OLED_RESET);

long webGetKey; // for unique Get request handling
char appMode = 'a'; // apply / idle, 'm' for manual control, 's' for started, 'o' for overspeed
// long stepperTotalRotation; // Calculated at start
//int cameraPictures; // Calculated at start: number of pictures the camera will take
int lastClientDataTick = 0; // Timestamp of last data exhange with client
long startTick; // timestamp the timelapse is started (for pre-delay timing)
long finishedTick; // timestamp the motor stopped moving

settings_t settings = { 2, 180, (30 * 60), 30, 512.0, 3, 25, 0, 0, 0 };
display_state_t displayState = DISPLAY_Splash;
display_state_t displayPrevState = DISPLAY_Number; // force upate 1st run
int displayRefreshTick = 0, displaySameStatus = 0;

int potAverage = 0; // number of ADC reads for averaging
int potValue = 0; // (cumulative) value from ADC
int potSetting = 0, potPrevSetting = 0, potSameSetting = 0; // The setting represented by the potmeter value, needs to be the same for some time to commit to

void setup()
{
  delay(500); // prevent brownout issue
  //pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Wifi AP Stepper Timelapse " + String(__DATE__) + " " + String(__TIME__));
  Serial.println("Initialize display...");
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setTextSize(1);
  display.setFont(font);
  display.setTextColor(WHITE);
  handleDisplay();
 
  Serial.println("Configuring stepper & load EEPROM...");
  stepper.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, settings);
  delay(50); // prevent brownout issue
  
  Serial.println("Configuring access point...");
  WiFi.onEvent(onWiFiAPStarted, SYSTEM_EVENT_AP_START);
  WiFi.onEvent(onWiFiStationConnected, SYSTEM_EVENT_AP_STACONNECTED);
  WiFi.onEvent(onWiFiStationDisconnected, SYSTEM_EVENT_AP_STADISCONNECTED);
  WiFi.softAP(ssid, password);

  randomSeed(analogRead(1)); // use noise of unconnected A1 as seed TODO check connected
  webGetKey = random(INT_MAX);

  // Need to wait for started event now, which can take some 100 ms, give max 200 ms, it is kinda crucial to have WiFi
  delay(300);
  for (int i = 0; ((i < 100 / portTICK_PERIOD_MS) && (!wifiStarted)); ++i) {
    delay(portTICK_PERIOD_MS);
  }
 
  myIP = WiFi.softAPIP();
  Serial.println("AP IP address: " + myIP.toString());
  server.begin();

  // Handle motor movements in a background task
  xTaskCreate(taskMotor, "stepperTask", 10000, NULL, 1, NULL);
  Serial.println("Server started"); 

  // SHow on screen that the client can connect now
  displayState = DISPLAY_WiFiLogin;
  handleDisplay();
}

void taskMotor( void * parameter )
{
  int i = 0;
  while(1) {
    if (!stepper.motionComplete()) {
      stepper.processMovement();
      if (((i++) % 200) == 0) {
        TASK_KICK_WATCHDOG();
      }
      else {
        finishedTick = millis();
      }
    }
    else {
      vTaskDelay(50 / portTICK_PERIOD_MS); // motor is not moving, check again some time later (give priority to main task)
    }
  }
}

void loop()
{
  WiFiClient client = server.available();   // Listen for incoming clients

  // Handle setup
  if (wifiConnected && client) {            // If a new client connects
    if (displayState < DISPLAY_WiFiSetup) {
      displayState = DISPLAY_WiFiSetup;
    }
    //digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("New Client.");
    lastClientDataTick = millis();
    String currentLine = ""; 
    while (client.connected() && wifiConnected) {
      if ((client.available()) &&    // if there's bytes to read from the client, handle it
          (!handleClientData(client, currentLine))) { // webpage updated, reconnect with client
        Serial.println("Client handled");
        break;
      }
      if ((!wifiConnected) || (millis() - lastClientDataTick > 40000)) { // WiFi disconnection or no data exchanged for more than 40 seconds
        Serial.println("Client disconnected (WiFi " + String(wifiConnected ? "Connected" : "Disconnected") + String(millis() - lastClientDataTick > 40000) ? " Data Timeout" : "");
        break;
      }
      handleMotorDisplayChanges();
    }
    
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    //digitalWrite(LED_BUILTIN, LOW);
  }
  else {
    if (displayState < DISPLAY_WiFiSetup) {
      handleSetupByPot();
    }
  }

  handleMotorDisplayChanges();
  
  TASK_KICK_WATCHDOG();
}

void handleMotorDisplayChanges()
{
  // Handle motor state changes
  if ((appMode == 'm') || (appMode == 's')) { // Automatic or manual motor control
    if ((getFinishedSeconds() >= 10) && // motor has stopped more than 10 seconds ago
        (isMotorEnabled())) { // motor is still energized
      stepper.disableMotor();
      Serial.println("Motor idle for > 10s: disable outputs");
    }
  }
  else if (appMode == 'd') {
    int preDelay = millis() - startTick;
    if (preDelay / 1000 >= settings.preDelay) { // Pre-delay expired
      Serial.println("Pre-delay " + String(settings.preDelay) + " has been spent");
      appMode = 's';
      driveStepperAuto();
    }
  }

  // Handle display updates
  if (!handleDisplay() && (wifiPrevConnected != wifiConnected)) { // display did not refresh the screen this call but the WiFi connected changed so update the icon
    drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
    display.display();
  }
  wifiPrevConnected = wifiConnected;
}

bool handleClientData(WiFiClient client, String currentLine)
{
  char c = client.read();             // read a byte, then
  //Serial.write(c);                    // print it out the serial monitor
  header += c;
  if (c == '\n') {                    // if the byte is a newline character
    lastClientDataTick = millis();
    // if the current line is blank, you got two newline characters in a row.
    // that's the end of the client HTTP request, so send a response:
    if (currentLine.length() == 0) {
      if ((appMode == 's') || (appMode == 'd')) { // Motor started in auto mode or pre-delay, show status page (with auto-refresh)
        sendStatusWebPage(client);
      }
      else if (appMode == 't') { 
        sendTripodSetupWebPage(client);
      }
      else {// Show control page
        sendControlWebPage(client);
      }   
      // The HTTP response ends with another blank line
      client.println();
      // Break out of the while loop
      return false;
    } else { // if you got a newline, then clear currentLine
      currentLine = "";
    }
  } else if (c != '\r') {  // if you got anything else but a carriage return character,
    currentLine += c;      // add it to the end of the currentLine
    lastClientDataTick = millis();
  }
  else { // Full request received
    Serial.println(header);
    if (parseParams(header)) {
      Serial.println("App Mode "+ String(appMode));
      header="";
      lastClientDataTick = millis();
    }
  }
  return true;
}          

void handleSetupByPot()
{
  // assuming we have a portTICK_PERIOD_MS delay per loop (at least)
  if (potAverage < 200 / portTICK_PERIOD_MS) { // get some average readings
    potValue += analogRead(ANALOG_POT_PIN); 
    potAverage++;
    return;
  }
  potValue /= (200 / portTICK_PERIOD_MS); 
  potAverage = 0; // next call will fill average values again
  Serial.print("ADC " + String(potValue));

  if (potValue < 100) {
    Serial.print(" Stop");
    potSetting = 0;
    potPrevSetting = potSetting;
    potSameSetting += (200 / portTICK_PERIOD_MS);
    if (potSameSetting > 10000 / portTICK_PERIOD_MS) { // some time the same value
      switch (displayState) { // traverse one state back
        default:
        case DISPLAY_PotSetupDegrees:
          if (wifiConnected) {
            displayState = DISPLAY_WiFiShowIP;
          }
          else {
            displayState = DISPLAY_WiFiLogin;
          }
          displayPrevState = DISPLAY_PotSetupDegrees;
          break;
        case DISPLAY_PotSetupDuration:
          displayState = DISPLAY_PotSetupDegrees;
          break;
        case DISPLAY_PotSetupAccel:
          displayState = DISPLAY_PotSetupDuration;
          break;
        case DISPLAY_PotWaitStart:
          displayState = DISPLAY_PotSetupAccel;
          break;
      }
      potSameSetting = 0;
    }
  }
  else {
    Serial.print(" ");
    if (displayState <= DISPLAY_PotSetupDegrees)
    {
      displayState = DISPLAY_PotSetupDegrees;
    }
    switch (displayState) {
      case DISPLAY_PotSetupDegrees:
        if (potValue < 3000) {
          potSetting = map(potValue, 100, 3000, 1, 18) * 10;
        }
        else {
          potSetting = map(potValue, 3000, 4095, 6, 12) * 30;
        }
        //Serial.print("ADC " + String(potValue));
        Serial.print(String(potSetting) + " degrees, same "+ String(potSameSetting));
        break;
      case DISPLAY_PotSetupDuration:
        if (potValue < 3000) {
          potSetting = map(potValue, 100, 2500, 1, 30); // minute
        }
        else {
          potSetting = map(potValue, 2500, 4095, 30, 60); // minute
        }
        potSetting *= 60; // convert to seconds
        Serial.print("ADC " + String(potValue));
        Serial.print(sec2time(potSetting) + " / " + String(potSetting) + " s duration, same "+ String(potSameSetting));
        break;
      case DISPLAY_PotSetupAccel:
        if (potValue < 3000) {
          potSetting = map(potValue, 100, 2000, 0, 10); // half a minute
          potSetting *= 30; // convert to seconds
        }
        else {
          potSetting = map(potValue, 2000, 4095, 5, 15); // minute
          potSetting *= 60; // convert to seconds
        }
        Serial.print("ADC " + String(potValue));
        Serial.print(sec2time(potSetting) + " / " + String(potSetting) + " s accelleration, same "+ String(potSameSetting));
        break;
      case DISPLAY_PotWaitStart:
        potSetting = (potValue > 3000) + 1; // 1 = wait, 2 = start
        Serial.print(String((potValue == 2) ? "Start" : "Wait") + ", same "+ String(potSameSetting));
        break;
      default:
        displayPrevState = displayState;
        if (wifiConnected) {
          displayState = DISPLAY_WiFiShowIP;
        }
        else {
          displayState = DISPLAY_WiFiLogin;
        }
        return;
    }

    if (potSetting == potPrevSetting) {
      potSameSetting += (200 / portTICK_PERIOD_MS);
      if (potSameSetting > 5000 / portTICK_PERIOD_MS) { // 5 seconds the same value
        switch (displayState) {
          case DISPLAY_PotSetupDegrees:
            settings.tripodDegrees = potSetting;
            displayState = DISPLAY_PotSetupDuration;
            break;
          case DISPLAY_PotSetupDuration:
            settings.rotationDuration = potSetting;
            displayState = DISPLAY_PotSetupAccel;
            break; 
          case DISPLAY_PotSetupAccel:
            settings.motorAccelSeconds = potSetting;
            displayState = DISPLAY_PotWaitStart;
            break;           
          case DISPLAY_PotWaitStart:
            if (potSetting == 2) {
              displayState = DISPLAY_ShowStatus1;
              settings.preDelay = 0;
              startTick = millis();
              driveStepperAuto(); 
            }
            break;
          default:
            break;
        }
      }
    }
    else {
      potSameSetting = 0;
      potPrevSetting = potSetting;
    }
  }
  Serial.println(" "+ String(displayState));
  potValue = 0; // Reset for next averaging run
}

void driveStepperStop()
{
  if (!stepper.motionComplete()) {
    stepper.setupRelativeMoveInSteps(0); // non-blocking move
  }
}

motor_runtime_t calculateMotorSetup(settings_t eval)
{
  motor_runtime_t run;
  
  run.tripodSteps = ((float)eval.tripodDegrees * eval.tripodPulley * (float)STEPS_PER_REVOLUTION) / 360.0; // Total steps for the stepper to make the desired tripod rotation in degrees
  if ((eval.motorAccelSeconds < 1) || (eval.motorAccelSeconds * 2.2 > eval.rotationDuration)) {
    run.motorSpeed = (run.tripodSteps / (float)eval.rotationDuration);
    run.motorAccelleration = (STEPS_PER_REVOLUTION / 4);
  }
  else {
    // See also https://www.linearmotiontips.com/how-to-calculate-velocity/: trapezoid move profile
    run.motorSpeed = run.tripodSteps / ((float)eval.rotationDuration - eval.motorAccelSeconds);
    run.motorAccelleration = run.motorSpeed / eval.motorAccelSeconds;
  }
  
  return run;
}

void driveStepperManual(settings_t eval)
{
  long stepperTotalRotation = (long)((((float)eval.tripodDegrees * eval.tripodPulley * (float)STEPS_PER_REVOLUTION) / 360.0) + 0.5); // equal rounding
  Serial.println("(New) Rotate " + String(stepperTotalRotation) + " at Speed "+ String(eval.stepperSpeed)); 
  if (eval.stepperSpeed == 0) {
	  driveStepperStop();
    return;
  }
  stepper.setSpeedInStepsPerSecond(abs(eval.stepperSpeed));
  stepper.setAccelerationInStepsPerSecondPerSecond(STEPS_PER_REVOLUTION / 4);
  if (eval.stepperSpeed > 0) {
    stepper.setupRelativeMoveInSteps(stepperTotalRotation);
  }
  else {
    stepper.setupRelativeMoveInSteps(-stepperTotalRotation);
  }
}

void driveStepperAuto()
{
  motor_runtime_t run = calculateMotorSetup(settings);
  long stepperTotalRotation = (long)(run.tripodSteps + 0.5); // equal rounding
  
  Serial.println("Auto: Tripod steps " + String(stepperTotalRotation) + "\t Speed " + String(run.motorSpeed) + "\tAccel " + String(run.motorAccelleration) + " @ " + String(settings.motorAccelSeconds) + " s preDelay " +String(settings.preDelay) + " s");
  stepper.setCurrentPositionInSteps(0);
  stepper.setAccelerationInStepsPerSecondPerSecond(run.motorAccelleration);
  stepper.setSpeedInStepsPerSecond(run.motorSpeed);
  stepper.setupRelativeMoveInSteps(stepperTotalRotation); // non-blocking move

  displayRefreshTick = 0;
  displaySameStatus = 0;
  displayState = DISPLAY_ShowStatus1;
}

void sendStatusWebPage(WiFiClient client)
{
  float degsMotor = (float)stepper.getCurrentPositionInSteps() * 360.0 / (float)STEPS_PER_REVOLUTION;
  int motorSpeed = stepper.getCurrentVelocityInStepsPerSecond();
  float degsPerMinute = (float)motorSpeed * (60.0 * 360.0) / (settings.tripodPulley * STEPS_PER_REVOLUTION); 
  float degsTripod = degsMotor / settings.tripodPulley;
  int spent = (int)((degsTripod * (float)settings.rotationDuration / (float)settings.tripodDegrees) + 0.5);
  int progress = (int)((degsTripod * 100.0 / settings.tripodDegrees) + 0.5);
  int refresh = 30;
  int preDelay = 0;
  int togo = 0;
  
  sendWebHeader(client, 20);
  client.print("<meta http-equiv=\"refresh\" content=\"");
  if (appMode == 'd') {
    preDelay = (500 + millis() - startTick) / 1000;
    togo = settings.preDelay - preDelay;
    refresh = min(10, togo + 1);
  }
  else {
    togo = settings.rotationDuration - spent;
    if (!stepper.motionComplete()) {
      refresh = min(10, togo + 1);
    }
    preDelay = settings.preDelay;
  }
  if (refresh < 4) {
    refresh = 4; // Do not flood the connection with refresh requests
  }
  client.print(String(refresh) + "\">"); // Set refresh every 10 seconds or just after motor stop, after pre-delay, but no less than 4 seconds

  // Web page
  sendBodyTag(client);
  client.println("<font size=\"+2\"><center>WiFi Timelapse Status</center></font><br/>");

  client.println("<table id=\"tblProgress\" border=\"1\" width=\"100%\"><colgroup><col class=\"colL\"/><col class=\"colLt\"/><col class=\"colPrePost\"/><col/></colgroup>");  
  // Progress
  client.println("<tr><td>Progress</td><td>" + String(progress) + "</td><td>%</td><td><center><canvas id=\"progressCnvs\" width=\"150\" height=\"40\" style=\"border:no;\"/></center></td></tr>");
  client.println("<script>var c = document.getElementById(\"progressCnvs\"); var ctx = c.getContext(\"2d\"); ctx.clearRect(0, 0, c.width, c.height);");
  client.println("ctx.beginPath(); ctx.rect(0, (c.height/2) - 5, c.width, 10); ctx.lineWidth = 1; ctx.strokeStyle = '#ffffff'; ctx.stroke();");
  if (progress > 0) {
    client.println("ctx.beginPath(); ctx.fillStyle = '#ffffff'; ctx.fillRect(1, (c.height/2) - 4, (c.width * " + String(progress) + ")/100, 8); ctx.stroke();");
  }
  client.println("</script>");
  // Tripod degrees
  client.println("<tr><td>Degrees done</td><td>" + String(degsTripod) + "</td><td>&deg;</td><td rowspan=\"2\"/><center><canvas id=\"tripodCnvs\" width=\"80\" height=\"80\" style=\"border:no;\"/></center></tr>"); 
  client.println("<tr><td>Degrees to go</td><td>" + String((float)settings.tripodDegrees - degsTripod) + "</td><td>&deg;</td></font></tr>");
  // Draw tripod graph
  client.println("<script>var d = parseFloat("+ String(settings.tripodDegrees) + ") / 2.0; var d1 = -(d + 90.0) * Math.PI / 180.0; var d2 = -(90.0 - d) * Math.PI / 180.0; var cen = 40; var rad = 39;");
  client.println("var c = document.getElementById(\"tripodCnvs\"); var ctx = c.getContext(\"2d\"); ctx.clearRect(0, 0, c.width, c.height);");
  client.println("ctx.beginPath(); ctx.moveTo(cen, cen); ctx.arc(cen, cen, rad, d1, d2); ctx.lineTo(cen,cen); ctx.lineWidth = 1; ctx.strokeStyle = '#ffffff'; ctx.stroke();");
  client.println("ctx.beginPath(); ctx.arc(cen, cen, rad - 11, 0, 2 * Math.PI); ctx.fillStyle = '#000066'; ctx.fill();");
  client.println("ctx.beginPath(); ctx.arc(cen, cen, rad - 10, d1, d2); ctx.stroke();");
  if (degsTripod > 0) {
    client.println("var d3 = d1 + (" + String(degsTripod) + "*Math.PI / 180.0); ctx.beginPath(); ctx.arc(cen, cen, rad - 5, d1, d3); ctx.lineWidth = 10; ctx.stroke();");
  }
  client.println("</script>");
  
  // Set table for the remainder of the status page
  client.println("<table border=\"1\" width=\"100%\"><colgroup><col class=\"colL\"/><col/><col class=\"colPrePost\"/><col class=\"colR\"/></colgroup>");  
  if (appMode == 'd') {
    client.println("<tr><td>Delay done</td><td>" + sec2time(preDelay) + "</td><td/><td>mm:ss</td></tr>");
    client.print("<tr><td>Delay to go</td><td>" + sec2time(togo));
  }
  else {
    client.println("<tr><td>Time done</td><td>" + sec2time(spent) + "</td><td/><td>mm:ss</td></tr>");
    if (stepper.motionComplete()) {
      client.print("<tr><td>Finished</td><td>" + sec2time(getFinishedSeconds()));
      spent += getFinishedSeconds();
    }
    else {
      client.print("<tr><td>Time left</td><td>" + sec2time(togo));
    }
  }
  client.println("</td><td/><td>mm:ss</td></tr>");
  client.println("<tr><td>Tripod rotation</td><td>" + String(degsPerMinute) + "</td><td/><td>&deg;/<sub>min</sub></td></tr>");
  client.println("<tr><td>Image</td><td>" + String((spent + preDelay) / settings.cameraInterval) + " of " + String((int)(((settings.rotationDuration + settings.preDelay) / settings.cameraInterval) + 0.5)) + "</td><td colspan=\"2\"/></tr>");
  client.println("<tr><td>Final lapse</td><td colspan=\"2\">" + sec2time((spent + preDelay) / (settings.cameraInterval * settings.playbackFPS)) + " mm:ss @ " + String(settings.playbackFPS) + " fps</td><td/></tr>");

  // Debug
  client.println("<tr><td colspan=\"4\" bgcolor=\"#000088\"><center>Motor information</center></td></tr>");
  client.println("<tr><td>Motor rotation</td><td>" + String(degsMotor) + "</td><td/><td>&deg;</td></tr>");
  client.println("<tr><td>Motor speed</td><td>" + String(motorSpeed) + "</td><td/><td>steps/s</td></tr>");

  // Break
  client.println("</table><br/><form method=\"get\"><input type=\"hidden\" name=\"key\" value=\"" + String(webGetKey) + "\"/><table border=\"0\" width=\"100%\">");
  
  // Buttons
  client.println("<tr><td colspan=\"3\"><center><button name=\"act\" type=\"submit\" width=\"100\" value=\"quit\">");
  if (stepper.motionComplete()) {
    client.print("Control");
  }
  else {
    client.print("Stop");
  }
  client.println("</button></center></td></tr>");

  client.println("</table></form>");
  sendWebFooter(client);
}

void sendControlWebPage(WiFiClient client)
{  
  sendWebHeader(client, 20);          
           
  // Web Page scripting
  client.println("<script>");
  client.println("function sec2time(secs) { var pad = function(num, size) { return ('000' + num).slice(size * -1); }, hours = Math.floor(secs / 60 / 60), minutes = Math.floor(secs / 60) % 60;");
  client.println("if (hours>0) return hours + ':' + pad(minutes, 2) + ':' + pad(secs % 60, 2) + \" <font size=\\\"-1\\\">hh:mm:ss</font>\";");
  client.println("return pad(minutes, 2) + ':' + pad(secs % 60, 2) + \" <font size=\\\"-1\\\">mm:ss</font>\"; }");

  client.println("function imagenum() { var dur=document.getElementById(\"txtDuration\").value, intv=document.getElementById(\"txtInterval\").value, dly=document.getElementById(\"txtDelay\").value, num=Math.round((parseInt(dur)+parseInt(dly))/intv);");
  client.println("document.getElementById(\"imgnum\").innerHTML=num;"); 
  client.println("document.getElementById(\"final-dur\").innerHTML=sec2time(Math.round(num/document.getElementById(\"txtFPS\").value)); }"); 

  client.println("function updateDur(dur) { var accel=document.getElementById(\"slAccel\"); document.getElementById(\"dur-hm\").innerHTML = sec2time(parseInt(dur)); imagenum();");
  client.println("if (dur<2.2 * accel.max) { if (dur<2.2 * accel.value) { accel.value=dur/2.2; } accel.max = dur/2.2; }");
  client.println("else if (dur<1980) { accel.max=dur/2.2; }");
  client.println("else { accel.max=900; }");
  client.println("document.getElementById(\"txtAccel\").value=accel.value; }");

  client.println("function slToTxt(sl, txt) { var x = document.getElementById(txt); var y = document.getElementById(sl); x.value = y.value; }");
  client.println("function txtToSl(txt, sl) { var x = document.getElementById(txt); var y = document.getElementById(sl); y.value = x.value; }");

  client.println("function slToTxtDur(sl, txt) { var x = document.getElementById(txt); var y = document.getElementById(sl); x.value = y.value; updateDur(x.value); }");
  client.println("function txtToSlDur(txt, sl) { var x = document.getElementById(txt); var y = document.getElementById(sl); y.value = x.value; updateDur(x.value); }");
  
  client.println("function slToTxtIv(sl, txt) { var x = document.getElementById(txt); var y = document.getElementById(sl); x.value = y.value; imagenum(); }");
  client.println("function txtToSlIv(txt, sl) { var x = document.getElementById(txt); var y = document.getElementById(sl); y.value = x.value; imagenum(); }");

  client.println("window.onload = function() { slToTxtDeg('slDegrees','txtDegrees'); slToTxtDur('slDuration','txtDuration'); slToTxt('slAccel','txtAccel'); slToTxt('slDelay','txtDelay'); slToTxt('slSpeed','txtSpeed'); slToTxt('slFPS','txtFPS'); slToTxtIv('slInterval','txtInterval'); }");
  client.println("</script>");

  if (appMode == 'm') {
    if (stepper.motionComplete()) {
      appMode = 'a'; // Set back to Idle for simplicity here
    }
    else {
      client.print("<meta http-equiv=\"refresh\" content=\"5\">"); // Set refresh every 5 seconds to unlock the page when the motor stoppped
    }
  }

  // Web page
  sendBodyTag(client);
  client.println("<font size=\"+2\"><center>WiFi Timelapse Control</center></font><br/>");
  // Columns: text, slider, textbox, extra postfix (CW), units
  client.println("<form method=\"get\"><table border=\"1\" width=\"100%\"><colgroup><col class=\"colLt\"/><col class=\"colPrePost\"/><col/><col class=\"colTXT\"/><col class=\"colPrePost\"/><col class=\"colCanvas\"/></colgroup>");

  // Degrees + setup column widths in 1st row
  client.println("<tr><td colspan=\"2\">Tripod degrees</td><td>");
  sendSliderTextCoupledControl(client, "Degrees", "graden", String(settings.tripodDegrees), 10, 360, 5, "Deg", "degreeTicks", (appMode == 's') || (appMode == 'm'));
  client.print("<datalist id=\"degreeTicks\"><option value=\"10\"><option value=\"45\"><option value=\"90\"><option value=\"180\"><option value=\"270\"><option value=\"360\"></datalist>");
  client.println("<td/><td>&deg;</td><td><center><canvas id=\"tripodCnvs\" width=\"60\" height=\"60\" style=\"border:no;\"/></center></td></tr></table>");
  // Draw tripod graph
  client.println("<script>function drawArc(degs) { var d = parseFloat(degs) / 2.0; var d1 = -(d + 90.0) * Math.PI / 180.0; var d2 = -(90.0 - d) * Math.PI / 180.0; var cen = 30; var rad = 28;");
  client.println("var c = document.getElementById(\"tripodCnvs\"); var ctx = c.getContext(\"2d\"); ctx.clearRect(0, 0, c.width, c.height);");
  client.println("ctx.beginPath(); ctx.moveTo(cen, cen); ctx.arc(cen, cen, rad, d1, d2); ctx.lineTo(cen,cen); ctx.lineWidth = 1; ctx.strokeStyle = '#ffffff'; ctx.stroke();");
  client.println("ctx.beginPath(); ctx.arc(cen, cen, rad - 11, 0, 2 * Math.PI); ctx.fillStyle = '#000066'; ctx.fill();");
  client.println("ctx.beginPath(); ctx.arc(cen, cen, rad - 10, d1, d2); ctx.stroke(); }");
  client.println("function slToTxtDeg(sl, txt) { var x = document.getElementById(txt); var y = document.getElementById(sl); x.value = y.value; drawArc(x.value); }");
  client.println("function txtToSlDeg(txt, sl) { var x = document.getElementById(txt); var y = document.getElementById(sl); y.value = x.value; drawArc(x.value); }");
  client.println("</script></table>");
  // Set table for the remainder of the controls
  client.println("<table border=\"1\" width=\"100%\"><colgroup><col class=\"colLt\"/><col class=\"colPrePost\"/><col/><col class=\"colTXT\"/><col class=\"colPrePost\"/><col class=\"colR\"/></colgroup>");

  // Duration
  client.print("<tr><td colspan=\"2\">Duration<br/><div id=\"dur-hm\">" + sec2time(settings.rotationDuration) + " <font size=\"-1\">");
  if (settings.rotationDuration > 3600) {
    client.print("hh:");  
  }
  client.print("mm:ss</font></div></td><td>");
  sendSliderTextCoupledControl(client, "Duration", "duur", String(settings.rotationDuration), 30, 7200, 5, "Dur", "duurTicks", (appMode == 's') || (appMode == 'm'));
  client.println("<datalist id=\"duurTicks\"><option value=\"30\"><option value=\"900\"><option value=\"1800\"><option value=\"3600\"><option value=\"5400\"><option value=\"7200\"></datalist>");
  client.println("<td/><td>s</td></tr>");

  // Accelleration
  client.println("<tr><td colspan=\"2\">Acceleration</td><td>");
  sendSliderTextCoupledControl(client, "Accel", "rem", String(settings.motorAccelSeconds), 0, 1200, -1, "", "accelTicks", (appMode == 's') || (appMode == 'm'));
  client.println("<datalist id=\"accelTicks\"><option value=\"30\"><option value=\"60\"><option value=\"120\"><option value=\"300\"><option value=\"600\"><option value=\"900\"><option value=\"1200\"></datalist>");
  client.println("<td/><td>s</td></tr>");

  // Pre-timelapse delay
  client.println("<tr><td colspan=\"2\">Pre delay</td><td>");
  sendSliderTextCoupledControl(client, "Delay", "wait", String(settings.preDelay), 0, 900, -1, "Iv", "delayTicks", (appMode == 's'));
  client.println("<datalist id=\"delayTicks\"><option value=\"0\"><option value=\"60\"><option value=\"120\"><option value=\"180\"><option value=\"240\"><option value=\"300\"><option value=\"600\"><option value=\"900\"></datalist>");
  client.println("<td/><td>s</td></tr>");
  
  // Overspeed warning
  if (appMode == 'o') {
    client.println("<tr><td colspan=\"6\" bgcolor=\"#880000\"><center><font size\"-1\">Too fast: reduce degrees, increase duration, reduce accelleration</font></center></td></tr>");
  }

  // Camera interval
  client.println("<tr><td colspan=\"6\" bgcolor=\"#000088\"><center>Timelapse indication</center></td></tr>");
  client.println("<tr><td colspan=\"2\">Camera Interval</td><td>");
  sendSliderTextCoupledControl(client, "Interval", "int", String(settings.cameraInterval), 1, 60, -1, "Iv", "", (appMode == 's') || (appMode == 'm'));
  client.println("</td><td/><td>s</td></tr>");
 
  // Final framerate
  client.println("<tr><td colspan=\"2\">Playback</td><td>");
  sendSliderTextCoupledControl(client, "FPS", "fps", String(settings.playbackFPS), 25, 60, -1, "Iv", "", (appMode == 's') || (appMode == 'm'));
  client.println("</td><td/><td>fps</td></tr>");

  client.print("<tr><td colspan=\"2\">Images &amp;<br/>Duration</td><td># <div id=\"imgnum\">0</div> &amp;<br/><div id=\"final-dur\">0</div></td><td colspan=\"3\"/></tr>");

  // Manual / Test
  client.println("<tr><td colspan=\"6\" bgcolor=\"#000088\"><center>Test control</center></td></tr>");
  client.println("<tr><td>Speed</td><td>&#8630;</td><td>");
  sendSliderTextCoupledControl(client, "Speed", "snel", String(settings.stepperSpeed), -STEPS_PER_REVOLUTION / 8, STEPS_PER_REVOLUTION / 8, 8, "", "testTicks", (appMode == 's'));
  client.println("<datalist id=\"testTicks\"><option value=\"-"+ String(STEPS_PER_REVOLUTION / 4) + "\"><option value=\"0\"><option value=\""+ String(STEPS_PER_REVOLUTION / 4) + "\"></datalist>");
  client.println("</td><td>&#8631;</td><td><sup>steps</sup>/<sub>s</sub></td></tr>");

  // Break
  client.println("</table><br/><input type=\"hidden\" name=\"key\" value=\"" + String(webGetKey) + "\"/><table border=\"0\" width=\"100%\">");

  // Buttons
  client.println("<tr><td><center>");
  if (appMode != 's') {
    client.print("<button name=\"act\" value=\"tripod\" type=\"submit\" width=\"100\"");
    if (appMode == 'm') {
      client.print(" disabled");
    }
    client.print(">Setup</button>&nbsp;&nbsp;<button name=\"act\" value=\"man\" type=\"submit\" width=\"100\"");
    if (appMode == 'm') {
      client.print(" disabled");
    }
    client.println(">Test</button>&nbsp;&nbsp;");
  }
  client.print("<button name=\"act\" type=\"submit\" width=\"100\" value=\"");
  if ((appMode == 's') || (appMode == 'm')) {
    client.println("quit\">Stop</button>");
  }
  else {
    client.println("start\">Start</button>");
  }
  client.println("</center></td></tr>");

  client.println("</table></form>");
  sendWebFooter(client);
}

void sendTripodSetupWebPage(WiFiClient client)
{  
  sendWebHeader(client, 20);          
           
  // Web Page scripting
  client.println("<script>");
  client.println("function updVerh() { var t = document.getElementById(\"txtTripod\").value; var p = document.getElementById(\"txtPulley\").value; document.getElementById(\"txtVerh\").value = (parseFloat(t) / parseFloat(p)).toFixed(2); }");
  client.println("function updPulley() { var t = document.getElementById(\"txtTripod\").value; var v = document.getElementById(\"txtVerh\").value; document.getElementById(\"txtPulley\").value = (parseFloat(t) / parseFloat(v)).toFixed(2); }");
  client.println("</script>");

  // Web page
  sendBodyTag(client);
  client.println("<font size=\"+2\"><center>Tripod setup</center></font><br/>");
  // Columns: text, textbox
  client.println("<form method=\"get\"><table border=\"1\" width=\"100%\"><colgroup><col class=\"colL\"/><col/></colgroup>");

  // Tripod diameter
  client.println("<tr><td>Tripod &#8960;</td><td><input id=\"txtTripod\" name=\"tripod\" type=\"number\" step=\"0.01\" value=\"" + String(settings.tripodDiameter) + "\" onchange=\"updVerh()\"></td></tr>");

  // Pulley diameter
  client.println("<tr><td>Pulley &#8960;</td><td><input id=\"txtPulley\" name=\"pulley\" type=\"number\" step=\"0.01\" value=\"" + String(settings.pulleyDiameter) + "\" onchange=\"updVerh()\"></td></tr>");

  // Relation
  client.println("<tr><td>Relation &#8960;<br/>Tripod/Pulley</td><td><input id=\"txtVerh\" name=\"verh\" type=\"number\" step=\"0.01\" value=\"" + String(settings.tripodPulley) + "\" onchange=\"updPulley()\"></td></tr>");

  // Break
  client.println("</table><br/><input type=\"hidden\" name=\"key\" value=\"" + String(webGetKey) + "\"/><table border=\"0\" width=\"100%\">");

  // Buttons
  client.println("<tr><td><center><button name=\"act\" value=\"apply\" type=\"submit\" width=\"100\">Ok</button></center></td></tr>");
  client.println("</table></form>");
  sendWebFooter(client);
}

bool parseParams(String parameters)
{ 
  settings_t eval = settings;
  char aMode = appMode;
  long key = ~webGetKey;
  bool storePersistent = false;
  int pos1 = 0, pos2 = 0, pos3 = 0;
  String pValue;
  char param[255];
  
  //GET /?verh=&graden=180&duur=295&rem=470&snel=350&int=3&fps=25&act=apply HTTP/1.1
  if (parameters.length() <= 15) { // More than "GET /? HTTP/1.1" received
    return false;
  }
  pos1 = parameters.indexOf("GET /");
  if (pos1 < 0) {
    return false;
  }
  pos1 = parameters.indexOf("?", pos1);
  if (pos1 < 0) {
    return false;
  }
  parameters.toCharArray(param, parameters.length());
  pos1++; // start position past questionmark
  while (pos1 >= 0) {
    pos2 = parameters.indexOf("=", pos1);
    if (pos2 <= 0) {
      break;
    }
    param[pos2++] = 0;
    pos3 = parameters.indexOf("&", pos2);
    if (pos3 <= 0) {
      pos3 = parameters.indexOf(" ", pos2);
      if (pos3 <= 0) {
        break;
      }
    }
    param[pos3++] = 0;
    if (pos3 - pos2 > 1) {
      pValue = String(param + pos2);
      switch (param[pos1]) {
        case 'v': // verhouding - relation tripod / pulley
          pValue.replace(",", ".");
          if (pValue.toFloat() != 0) {
             eval.tripodPulley = pValue.toFloat();
          }
          break;
        case 't': // tripod diameter
          pValue.replace(",", ".");
          if (pValue.toFloat() != 0) {
             eval.tripodDiameter = pValue.toFloat();
          }
          break;
        case 'p': // pulley diameter
          pValue.replace(",", ".");
          if (pValue.toFloat() != 0) {
             eval.pulleyDiameter = pValue.toFloat();
          }
          break;
        case 'g': // graden - degrees
          if (pValue.toInt() != 0) {
             eval.tripodDegrees = pValue.toInt();
          }
          break;
        case 'd': // duur - duration
          if (pValue.toInt() != 0) {
             eval.rotationDuration = pValue.toInt();
          }
          break;
        case 'r': // rem - brake / decel / accel
          eval.motorAccelSeconds = pValue.toInt();
          break;
        case 'w': // pre-delay
          eval.preDelay = pValue.toInt();
          break;
        case 'i': // camera image interval (status indication)
          eval.cameraInterval = pValue.toInt();
          break;
        case 'f': // final timelapse fps (status indication)
          eval.playbackFPS = pValue.toInt();
          break;          
        case 's': // (Manual) stepper speed)
          pValue.replace(",", ".");
          if (pValue.toFloat() != 0) {
             eval.stepperSpeed = pValue.toFloat();
          }          
          break;
        case 'a': // action
          aMode = param[pos2];
          break;
		    case 'k': // unique key
          key = pValue.toInt();
          break;
        default:
          break;
      }
    }
    // else ignore parameter without value
    pos1 = pos3;
  }

  if (webGetKey != key) { // Illegal key, maybe this is a browser cached GET ?
    Serial.println("Wrong GET ? key");
    return true;
  }
  switch(aMode) { // Webpage action handling
    case 'm': // Manual control
      if ((appMode == 's') || appMode == 'd') { // start(ed): ignore
        return true;
      }
      break;
    case 'd': // pre-start delay
    case 's': // start
      if ((appMode == 's') || appMode == 'd') { // Started already, ignore new settings
        return true;
      }
      if (appMode == 'm') { // Stop motor when in manual mode
        // Turn off motor
        driveStepperStop();
      }
      if (calculateMotorSetup(eval).motorSpeed > (STEPS_PER_REVOLUTION / 2)) { // need 2 ms between steps minimum or the motor will not move
        appMode = 'o'; // overspeed
        return true;
      }
      webGetKey = random(INT_MAX); // refresh key because we're going to the status page now
      storePersistent = true;
      break;
    case 'q': // quit / stop
      if ((appMode == 'a') || (appMode == 'o')) {
        return true;
      }
      break;
    case 'a': // apply settings
      if ((appMode == 's') || appMode == 'd') { // don't change stuff now
        return true;
      }
      storePersistent = true;
      break;
    case 't': // Enter setup page tripod/pulley relation
      if ((appMode == 's') || (appMode == 'm')) { // don't change stuff now
        return true;
      }
      webGetKey = random(INT_MAX); // refresh key because we're going to the status page now
      break;
    default:
      return true;
  }  
  // Apply changes
  settings = eval;
  
  if (storePersistent) {
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  // Post handling for mode change
  if (aMode != appMode) {
    if (aMode == 'm') {
      driveStepperManual(eval);
    }
    else if (aMode == 's') {
      if (settings.preDelay > 0) {
        stepper.setCurrentPositionInSteps(0);
        startTick = millis();
        aMode = 'd';
        displayState = DISPLAY_ShowWaitStatus;
      }
      else {
        // Make calculations & start motor
        startTick = millis();
        driveStepperAuto();
      }
    }
    else if (aMode == 'q') {
      // Turn off motor
      driveStepperStop();
      while (!stepper.motionComplete()) { }
      stepper.disableMotor();
      webGetKey = random(INT_MAX); // refresh key because we're going to the control page now
      aMode = 'a'; // apply / idle
    }
    appMode = aMode;
  }
  
  return true;
}

// Support functions

void sendWebHeader(WiFiClient client, int colPrePostWidth)
{            
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();

  client.println("<!DOCTYPE html><html>");
  client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.println("<link rel=\"icon\" href=\"data:,\">");
  client.println("<style>body { text-align:left; font-family: \"Trebuchet MS\", Arial; margin:0; padding:2; }");
  client.println("div { display: inline; }");
  client.println("table { width:100%; table-layout:fixed; border-bottom: 1px grey; border-top: 1px grey; border-collapse: collapse; }");
  client.println(".colL { width:125px; }"); // text column
  client.println(".colLt { width:" + String(125-colPrePostWidth) + "px; }"); // text - prefix column (rotation direction)
  client.println(".colTXT { width:54px; }"); // textbox column
  client.println(".colPrePost { width:" + String(colPrePostWidth) + "px; }"); // extra column (rotation direction)
  client.println(".colR { width:65px; }"); // units
  client.println(".colCanvas { width:10px; }"); // for tripod degrees graphic
  client.println("td { border-left: none; border-right: none; }");
  client.println("th, td { padding:3px; height:40px; }");
  client.println(".slider { width: 98%; }");
  client.println("#footer { position : absolute; bottom : 0; height : 40px; margin-top : 40px; }</style></head>");
}

void sendBodyTag(WiFiClient client)
{
  client.println("<body style=\"background-color:#000066;color:#FFFFFF;\">");
}

void sendWebFooter(WiFiClient client)
{ 
  client.println("<div id=\"footer\" align=\"left\"><font size=\"1\"><i>Version " + String(__DATE__) + " " + String(__TIME__) + "</i><br/>by Bart Rijvers 2020</font></div>");
  client.println("</body></html>"); 
}

void sendSliderTextCoupledControl(WiFiClient client, String OnPageName, String HTMLName, String CurrentValue, int Min, int Max, int Step, String ScriptPostfix, String TickList, bool Readonly)
{
  client.print("<input id=\"sl"+ OnPageName + "\" type=\"range\" class=\"slider\" min=\""+ String(Min) + "\" max=\""+ String(Max) + "\" ");
  if (Step > 0) {
    client.print("step=\""+ String(Step) + "\" ");
  }
  client.print("value=\"" + CurrentValue + "\" ");
  if (TickList.length() > 0) {
    client.print("list=\"" + TickList + "\" ");
  }
  if (Readonly) {
    client.println("disabled=\"true\">");
  }
  else {
    client.println("onchange=\"slToTxt" + ScriptPostfix + "('sl"+ OnPageName + "','txt"+ OnPageName + "')\">");
  }
  client.print("</td><td><input id=\"txt"+ OnPageName + "\" name=\""+ HTMLName + "\" type=\"text\" size=\"2\" ");
  if (Readonly) {
    client.println("readonly=\"true\">");
  }
  else {
    client.println("onchange=\"txtToSl"+ ScriptPostfix + "('txt"+ OnPageName + "','sl"+ OnPageName + "')\">");
  }
}

String pad2(int num)
{
  if (num > 9) return String(num);
  return "0" + String(num);
}

String sec2time(int secs)
{
  int hours = (secs / 60 / 60), minutes = (secs / 60) % 60;
  if (hours>0) return String(hours) + ':' + pad2(minutes) + ':' + pad2(secs % 60);
  return pad2(minutes) + ':' + pad2(secs % 60); 
}

long getFinishedSeconds()
{
  if (stepper.motionComplete()) {
    return (millis()- finishedTick) / 1000;
  }
  return 0;
}

bool isMotorEnabled()
{
  return ((digitalRead(MOTOR_IN1_PIN) == HIGH) ||
      (digitalRead(MOTOR_IN2_PIN) == HIGH) ||
      (digitalRead(MOTOR_IN3_PIN) == HIGH) ||
      (digitalRead(MOTOR_IN4_PIN) == HIGH));
}

// Event fired once at start to sync with further WiFi initialization
// https://github.com/espressif/arduino-esp32/issues/985
void onWiFiAPStarted(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.print("WiFi AP started ");
  wifiStarted = true;
}

// Events from https://techtutorialsx.com/2019/09/22/esp32-soft-ap-station-disconnected-event/
void onWiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  wifiConnected = true;
  if (displayState < DISPLAY_WiFiShowIP) {
    displayState = DISPLAY_WiFiShowIP;
  }
  Serial.print("Station connected ");
  for(int i = 0; i< 6; i++) {
    Serial.printf("%02X", info.sta_connected.mac[i]);  
    if(i<5)Serial.print(":");
  }
  Serial.println();
}

void onWiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Station disconnected ");
  for(int i = 0; i< 6; i++) {
    Serial.printf("%02X", info.sta_disconnected.mac[i]);  
    if(i<5)Serial.print(":");
  }
  Serial.println();
  wifiConnected = false;
  if (displayState <= DISPLAY_WiFiSetup) {
    displayState = DISPLAY_WiFiLogin;
  }
}

void drawWifi(uint16_t x, uint16_t y)
{
  uint16_t color = BLACK;
  if (wifiConnected) {
    color = WHITE;
  }
  display.drawCircleHelper(x, y, 2, 2, color);
  display.drawCircleHelper(x, y, 5, 2, color);
  display.drawCircleHelper(x, y, 8, 2, color);
}

void drawTripodSector(int x, int y, int r, int currentDegs, int totalDegs)
{
  if (totalDegs < 5) {
    return;
  }
  if ((currentDegs > 0) && (currentDegs % 360 == 0)) {
    display.fillCircle(x, y, r, WHITE);
    return;
  }
  totalDegs %= 360; // can only draw a full circle or less
  currentDegs %= 360;
  
  // Fill outline
  float radT = radians((float)totalDegs  / 2.0);
  int dxT = ((sin(radT) * (double)r) + 0.5);
  int dyT = -((cos(radT) * (double)r) + 0.5);
  int dxC, dyC;
 
  display.drawCircle(x, y, r, WHITE);
  if ((totalDegs != 0) && (totalDegs < 355) && ((2 * r) > dyT - 1)) {
    display.fillRect(x - r, y + dyT + 1, 2 * r + 1, (2 * r) - dyT - 1, BLACK);
  }
  if (r > 10) {
    int sr = (r / 2.5) + 1;
    int sdyT = -((cos(radT) * (double)sr) + 0.5);
    display.drawCircle(x, y, sr, WHITE);
    if ((totalDegs != 0) && (totalDegs < 355) && ((2 * sr) > sdyT - 1)) {
      display.fillRect(x - sr, y + sdyT + 1, 2 * sr + 1, (2 * sr) - sdyT - 1, BLACK);
    }
  }
  
  // Fill sectors that have been finished
  if (currentDegs > 0) {
    float radC = radT;
    float radCP = radT;
    float radPT = radians(20); // degrees per triangle to draw will draw nicely for circles up to r 32 => A = acos((r-2)/r)
    int dxC2, dyC2;
    dxC = dxT;
    dyC = dyT;
    for (int i = 0; i <= currentDegs - 20; i += 20) {
      radC = radCP - radPT;
      dxC2 = ((sin(radC) * (double)(r + 1)) + 0.5);
      dyC2 = -((cos(radC) * (double)(r + 1)) + 0.5);
      display.fillTriangle(x, y, x - dxC, y + dyC, x - dxC2, y + dyC2, WHITE); // sector of 20°
      dxC = dxC2;
      dyC = dyC2;
      radCP = radC;
    }
    currentDegs %= 20;
    if (currentDegs) {
      radC = radC - radians(currentDegs);
      dxC2 = ((sin(radC) * (double)(r + 1)) + 0.5);
      dyC2 = -((cos(radC) * (double)(r + 1)) + 0.5);  
      display.fillTriangle(x, y, x - dxC, y + dyC, x - dxC2, y + dyC2, WHITE); // (last) sector of < 20°
    }
  }
  display.drawCircle(x, y, r + 1, BLACK); // clip all excess triangle edges that stick outside the circle

  if (totalDegs != 0) {
    // Draw lines center circle to the extremes
    display.drawLine(x, y, x + dxT, y + dyT, WHITE);
    display.drawLine(x, y, x - dxT, y + dyT, WHITE);
  }
  if (r > 10) { // Blank out the middel section when the graph is large enough
    display.fillCircle(x,y,(r / 2.5), BLACK);
    display.drawPixel(x,y, WHITE);
  }
}

bool handleDisplay()
{
  if (millis() - displayRefreshTick <= DISPLAY_STATUS_REFRESH){
    return false;
  }
  displayRefreshTick = millis();
  
  switch(displayState) {
    // Dynamic update screens first
    case DISPLAY_PotSetupDegrees:
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      if (potSetting > 0) {
        drawTripodSector(90, 21, 21, 0, potSetting);
      }
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.println("Tripod");
      if (potSetting == 0) {
        display.println("Stop in " + String(10 - ((potSameSetting + 500) / 1000)) + " s");
      }
      else {        
        display.println(String(potSetting));
        display.drawCircle(50, 25, 3, WHITE);
      }
      display.println("Stop - 10 - 360");
      display.drawCircle(124, 46, 3, WHITE);
      display.display();
      return true;
    case DISPLAY_PotSetupDuration:
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.println("Duration");
      if (potSetting == 0) {
        display.println("Back in " + String(10 - ((potSameSetting + 500) / 1000)) + " s");
      }
      else {
        display.print(sec2time(potSetting) + " ");
        if (potSetting >= 3600) {
          display.println("h:m:s");
        }
        else {
          display.println("mm:ss");
        }
      }
      display.println("Back - 1m - 1h");
      display.display();
      return true;
    case DISPLAY_PotSetupAccel:
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);        
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.println("Accelleration");
      if (potSetting == 0) {
        display.print("Back in " + String(10 - ((potSameSetting + 500) / 1000)) + " s");
      }
      else {
        display.print(sec2time(potSetting) + " ");
        display.println("mm:ss");
      }
      display.println("Back - 0s - 15m");
      display.display();
      return true;
    case DISPLAY_PotWaitStart:
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.println();
      switch (potSetting) {
        case 0: display.println("Back in " + String(10 - ((potSameSetting + 500) / 1000)) + " s"); break;
        default:
        case 1: display.println("Wait for start"); break;
        case 2: display.println("Start in " + String(5 - ((potSameSetting + 500) / 1000)) + " s"); break;
      }
      display.println("Back-Wait-Start");
      display.display();
      return true;
    case DISPLAY_ShowWaitStatus:
    {
      int preDelay = (millis() - startTick);
      int progressPix = (int)((preDelay * 126.0 / (settings.preDelay * 1000.0)) + 0.5);
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.drawRoundRect(0, 56, 128, 8, 3, WHITE);
      display.fillRoundRect(1, 57, progressPix, 6, 1, WHITE);
      display.print("Delay");
      display.setCursor(50, display.getCursorY());
      display.println(sec2time((preDelay + 500) / 1000));
      display.print("Total");
      display.setCursor(50, display.getCursorY());
      display.println(sec2time(settings.preDelay));
      display.display();      
      return true;
    }
    case DISPLAY_ShowStatus1:
    {
      float degsMotor = (float)stepper.getCurrentPositionInSteps() * 360.0 / (float)STEPS_PER_REVOLUTION;
      float degsTripod = degsMotor / settings.tripodPulley;
      int progressPix = (int)((degsTripod * 126.0 / settings.tripodDegrees) + 0.5);
      display.clearDisplay();
      // Draw tripod sector
      drawTripodSector(85, 26, 26, degsTripod, settings.tripodDegrees);
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      display.drawRoundRect(0, 56, 128, 8, 3, WHITE);
      display.fillRoundRect(1, 57, progressPix, 6, 1, WHITE);
      display.print(String((int)(degsTripod + 0.5)));
      display.drawCircle(36, 3, 3, WHITE);
      display.setCursor(40, display.getCursorY());
      display.println("/");
      display.println(String(settings.tripodDegrees));
      display.drawCircle(36, 24, 3, WHITE);
      display.display();
      if (++displaySameStatus > DISPLAY_STATUS_TIME_PER_SCREEN) {
        displaySameStatus = 0;
        displayState = DISPLAY_ShowStatus2;
      }        
      return true;
    }
    case DISPLAY_ShowStatus2:
    {
      float degsMotor = (float)stepper.getCurrentPositionInSteps() * 360.0 / (float)STEPS_PER_REVOLUTION;
      float degsTripod = degsMotor / settings.tripodPulley;
      int spent = (int)((degsTripod * (float)settings.rotationDuration / (float)settings.tripodDegrees) + 0.5);
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      if (stepper.motionComplete()) {
        spent += getFinishedSeconds();
        display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
      }
      else {
        display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y + 15);
      }
      display.print("Time");
      display.setCursor(46, display.getCursorY());      
      display.println(sec2time(spent));
      display.print("Total");
      display.setCursor(46, display.getCursorY());      
      display.println(sec2time(settings.rotationDuration));
      if (stepper.motionComplete()) {
        display.print("Done");
        display.setCursor(46, display.getCursorY());      
        display.println(sec2time(getFinishedSeconds()) +" ago");
      }
      else {        
        int progressPix = (int)((spent * 126.0 / (settings.rotationDuration)) + 0.5);
        display.drawRoundRect(0, 59, 128, 5, 3, WHITE);
        display.fillRoundRect(1, 60, progressPix, 3, 1, WHITE);

      }
      display.display();
      if (++displaySameStatus > DISPLAY_STATUS_TIME_PER_SCREEN) {
        displaySameStatus = 0;
        displayState = DISPLAY_ShowStatus3;
      }
      return true;
    }
    case DISPLAY_ShowStatus3:
    {
      float degsMotor = (float)stepper.getCurrentPositionInSteps() * 360.0 / (float)STEPS_PER_REVOLUTION;
      float degsTripod = degsMotor / settings.tripodPulley;
      int spent = settings.preDelay + (int)((degsTripod * (float)settings.rotationDuration / (float)settings.tripodDegrees) + 0.5);
      if (stepper.motionComplete()) {
        spent += getFinishedSeconds();
      }
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y + 15);
      display.print("Image");
      display.setCursor(60, display.getCursorY());
      display.println(String(spent / settings.cameraInterval));
      display.print("Total");
      display.setCursor(60, display.getCursorY());      
      display.println(String((int)(((settings.rotationDuration + settings.preDelay) / settings.cameraInterval) + 0.5)));
      if (!stepper.motionComplete()) {
        int progressPix = (int)((spent * 126.0 / (settings.rotationDuration)) + 0.5);
        display.drawRoundRect(0, 59, 128, 5, 3, WHITE);
        display.fillRoundRect(1, 60, progressPix, 3, 1, WHITE);
      }
      display.display();
      if (++displaySameStatus > DISPLAY_STATUS_TIME_PER_SCREEN) {
        displaySameStatus = 0;
        displayState = DISPLAY_ShowStatus4;
      }           
      return true;
    }
    case DISPLAY_ShowStatus4:
    {
      float degsMotor = (float)stepper.getCurrentPositionInSteps() * 360.0 / (float)STEPS_PER_REVOLUTION;
      float degsTripod = degsMotor / settings.tripodPulley;
      int spent = settings.preDelay + (int)((degsTripod * (float)settings.rotationDuration / (float)settings.tripodDegrees) + 0.5);
      if (stepper.motionComplete()) {
        spent += getFinishedSeconds();
      }
      display.clearDisplay();
      drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
      display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y + 15);      
      display.print("Final");
      display.setCursor(60, display.getCursorY());      
      display.println(sec2time(spent / (settings.cameraInterval * settings.playbackFPS)));
      display.print("F.rate");
      display.setCursor(60, display.getCursorY());      
      display.println(String(settings.playbackFPS) + " fps");
      if (!stepper.motionComplete()) {
        int progressPix = (int)((spent * 126.0 / (settings.rotationDuration)) + 0.5);
        display.drawRoundRect(0, 59, 128, 5, 3, WHITE);
        display.fillRoundRect(1, 60, progressPix, 3, 1, WHITE);
      }
      display.display();
      if (++displaySameStatus > DISPLAY_STATUS_TIME_PER_SCREEN) {
        displaySameStatus = 0;
        if (stepper.motionComplete()) {
          displayState = DISPLAY_ShowStatus2;
        }
        else {
          displayState = DISPLAY_ShowStatus1;
        }
      }           
      return true;
    }
    default: // More static screens only when switching display states
      if (displayState != displayPrevState) {
        display.clearDisplay();
        switch(displayState) {
          case DISPLAY_Splash:
            display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
            display.setTextSize(1);
            display.println("WiFi Timelapse");
            display.println("by");
            display.println("Bart Rijvers");
            break;
          case DISPLAY_WiFiLogin:
            display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
            display.println("ssid " + String(ssid));
            display.println("Password:");
            display.println(String(password));
            break;
          case DISPLAY_WiFiShowIP:
            drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
            display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
            display.println();
            display.println("IP address:");
            display.println(myIP.toString());
            break;
          case DISPLAY_WiFiSetup:
            drawWifi(DISPLAY_WIFI_X, DISPLAY_WIFI_Y);
            display.setCursor(DISPLAY_HOME_X, DISPLAY_HOME_Y);
            display.println();
            display.println("Setup via");
            display.println("web interface");
            break;
          default:
            break;
        }
        display.display();
        displayPrevState = displayState;
        return true;
      }
      break;
  }
  return false;
}
