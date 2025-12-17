#line 1 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
/**
 * **********************************************************************
 * GhostRover 3 - NMEA sentence relay from ZED-F9P to Bluetooth LE.
 * **********************************************************************
 *
 * @author   D. Foster <doug@dougfoster.me>.
 * @since    3.0.9 [2025-12-17-10:00am] New.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover_BT_relay.
 * @see      https://github.com/doug-foster/DougFoster_Ghost_Rover_EVK_RTCM_relay.
 * @link     http://dougfoster.me.
 *
 * ----------------------------------------------------------------------------
 *                          Comments.
 * ----------------------------------------------------------------------------
 * 
 * --- Description & operation. ---
 *
 * --- Major components. ---
 *     -- MCU https://www.sparkfun.com/sparkfun-thing-plus-esp32-s3.html.
 *
 * --- Other components. ---
 * --- Misc references. ---
 *
 * --- Dev environment. ---
 *     -- IDE        VS Code & Arduino Maker Workshop 1.0.8 extension (uses Arduino CLI 1.3).
 *     -- GitHub     https://github.com/doug-foster/DougFoster_Ghost_Rover_BT_relay.
 *
 * --- Caveats. ---
 *     -- https://github.com/avinabmalla/ESP32_BleSerial/issues/24.
 *        ESP32_BleSerial version 3.0.0 requires Espressif core version 3.2.1.
 *        Espressif core version 3.3.x produces error: 'ESP_GATT_MAX_ATTR_LEN'
 *          was not declared in this scope.
 *        https://github.com/avinabmalla/ESP32_BleSerial/issues/25.
 *        BleBufferedSerial .end causes a crash.
 * 
 * --- TODO: ---
 * --- Code flow. ---
 *     -- Include libraries.
 *     -- Global vars.
 *     -- Setup functions.
 *     -- Setup.
 *     -- Task functions.
 *     -- Loop functions.
 *     -- Loop.
*/

/**
 * ============================================================================
 *                          Include libraries.
 * ============================================================================
 *
 * @since                   3.0.9 [2025-12-17-10:00am] New.
 * @link  Arduino           https://docs.arduino.cc/libraries/.
 * @link  ESP32             https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html.
 * @link  I2C               https://docs.arduino.cc/language-reference/en/functions/communication/wire/.
 */

// --- Core. ---
#include <Arduino.h>            // https://github.com/espressif/arduino-esp32.
#include <Wire.h>               // https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h.
#include <esp_chip_info.h>      // https://github.com/pycom/pycom-esp-idf.

// --- Additional. ---
#include <BleSerial.h>          // https://github.com/avinabmalla/ESP32_BleSerial (3.0.0).

/**
 * ============================================================================
 *                          Global vars.
 * ============================================================================
 *
 * @since 3.0.9 [2025-12-17-10:00am] New.
 */

// --- Pin (pth) definitions. ---

// --- LED. ---
enum ws2812_LED_COLOR {                                         // WS2812 RGB STAT LED, pin=RGB_BUILTIN=IO23.
    OFF,                                                        // 0.
    RED,                                                        // 1.
    YELLOW,                                                     // 2.
    GREEN,                                                      // 3.
    BLUE,                                                       // 4.
    WHITE                                                       // 5.
} ws2812LedColor;
bool ws2812LedBlink;

// --- Serial. ---
const char*    COMMANDS[2] = {                                  // Valid commands. Point to array of C-strings.
                               "debugBT",
                               "reset"
};
const uint8_t  NUM_COMMANDS     = sizeof(COMMANDS) / sizeof(COMMANDS[0]);   // Number of array elements in commands[].
const uint32_t SERIAL_MON_SPEED = 115200;                       // Serial USB monitor speed.
      char     monitorBuffer[50];                               // Serial monitor buffer (C-string).
      uint8_t  posn = 0;                                        // Input position for USB serial monitor command.

// --- I2C. ---
const uint8_t   I2C_SLAVE_ADDRESS   = 8;                        // I2C slave address.
const uint16_t  I2C_SPEED_FAST_MODE = 400000;                   // I2C speed - Fast Mode.
      char      buffer[100];                                    // Incoming buffer.

// --- BLE (Bluetooth Low Energy). ---
const char       BLE_NAME[]        = "GhostRover";              // BLE name.
const uint8_t    LED_TRIGGER_COUNT = 5;                         // Flash BLE LED once for every LED_TRIGGER_COUNT sentences sent.
const uint64_t   NMEA_TIMEOUT      = 5000000;                   // Time (us) not to exceed for DevUBLOXGNSS::processNMEA().
const TickType_t BLE_TEST_CYCLE    = 1000/portTICK_PERIOD_MS;   // Time (ms).
      char       stateBLE;                                      // BLE state. Used in OLED dsiplay.
      BleSerial  ble;                                           // BLE object.

// --- Timing. ---
const TickType_t LED_TIME_FLASH_ON  = 100/portTICK_PERIOD_MS;   // Timer (ms) =  0.1 seconds.
const TickType_t LED_TIME_FLASH_OFF = 1000/portTICK_PERIOD_MS;  // Timer (ms) =  1.0 seconds.

// --- Task handles. ---
TaskHandle_t loopStatusLEDtaskHandle;                           // Task: Loop status LED.

// --- Operation. ---
       bool debugBT;                                            // Debug Bluetooth.
       bool reset;                                              // Reset MCU.

// --- General. ---
esp_chip_info_t chip_info;                                      // Chip info.
bool            inLoop = false;                                 // In loop indicator.

// --- Version. ---
const char BUILD_DATE[]  = "[2025-12-17-02:00pm]";
const char MAJOR_VERSION = '3';
const char MINOR_VERSION = '0';
const char PATCH_VERSION = '9';
const char NAME[]        = "Ghost Rover 3 - BT Relay";

// --- Declaration. ---
// --- Test. ---

/**
 * ============================================================================
 *                          Setup functions.
 * ============================================================================
 */

 /**
 * ------------------------------------------------
 *      Start serial USB monitor.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 */
#line 151 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startSerialUsbMonitor();
#line 170 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void chipInfo();
#line 185 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void initVars();
#line 203 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void configPins();
#line 218 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startSerialInterfaces();
#line 234 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startI2C();
#line 261 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void receiveEvent(int bytesIn);
#line 287 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void requestEvent();
#line 309 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startTasks();
#line 326 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startBLE();
#line 343 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startLoop();
#line 357 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void setup();
#line 385 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void loopStatusLedTask(void * pvParameters);
#line 428 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void checkSerialMonitor();
#line 494 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void loop();
#line 151 "/Users/dougfoster/Library/CloudStorage/Dropbox/Data/doug/Topics/_dev-arduino/DougFoster_Ghost_Rover_BT_relay/DougFoster_Ghost_Rover_BT_relay.ino"
void startSerialUsbMonitor() {
    Serial.begin(SERIAL_MON_SPEED);
    delay(1000);
    Serial.printf("\n%s, Version: %c.%c.%c, Build date: %s.\n", NAME, MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION, BUILD_DATE);
    chipInfo();     // Display processor info.
    Serial.println("\nSetup() started.");
    Serial.printf("Serial USB monitor started @ %i bps.\n", SERIAL_MON_SPEED);
}

/**
 * ------------------------------------------------
 *      Display processor info.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    startSerialUsbMonitor().
 * @see    setup().
 */
void chipInfo() {
    esp_chip_info(&chip_info);
    Serial.printf("Using %s, Rev %d,  %d core(s), ID (MAC) %012llX.\n",
    ESP.getChipModel(), chip_info.revision, chip_info.cores, ESP.getEfuseMac());
}

/**
 * ------------------------------------------------
 *      Initialize global vars.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 */
void initVars() {
    Serial.print("Init global vars");
    ws2812LedColor = RED;       // Task - loop status indicator LED.
    ws2812LedBlink = false;     // Task - loop status indicator LED.
    memset(buffer,'\0',sizeof(buffer));
    debugBT = false;
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Initialize pins & pin values.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 */
void configPins() {

    // --- Initialize pin modes. ---
    // --- Initialize pin values. ---
}

/**
 * ------------------------------------------------
 *      Start serial interfaces.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 */
void startSerialInterfaces() {

        // --- Serial0 interface. ---
        // --- Serial1 interface. ---
        // --- Serial2 interface. ---
}

/**
 * ------------------------------------------------
 *      Start I2C interface.
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 */
void startI2C() {

    // --- Start interface. ---
    if (Wire.begin(I2C_SLAVE_ADDRESS) == false) {
        Serial.println("Start Wire failed. Freezing.");
        while (true);
    }
    Wire.setClock(I2C_SPEED_FAST_MODE);
    Serial.println("Wire started @ 4kHz.");

    // --- Register event functions. ---
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.println("Wire receive/request event functions registered.");
}

/**
 * ------------------------------------------------
 *      I2C receive event function.
 * ------------------------------------------------
 *
 * Executes when data is received from I2C master.
 * 
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-04-06:30pm] New.
 * @see    setup().
 */
void receiveEvent(int bytesIn) {

    for (int i = 0; i <= bytesIn; i++) {
      char incomingByte = Wire.read();
      buffer[i] = incomingByte;
    }
    if(debugBT) {
        Serial.printf("%i bytes  %s", strlen(buffer), buffer);
    }
    ble.print(buffer);
    memset(buffer, '\0', sizeof(buffer));
    ws2812LedColor = BLUE;
    ws2812LedBlink = true;
}

/**
 * ------------------------------------------------
 *      I2C request event function.
 * ------------------------------------------------
 *
 * Executes when data is requested by I2C master.
 * 
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-04:00pm] New.
 * @see    setup().
 */
void requestEvent() {

    // Wire.write(inByte); // respond with message of 6 bytes as expected by master.
    // Serial.print("<- ");
    // Serial.println(inByte);
    // ToDo: BT send - ack to main processor for UI.
    // ToDo: BT send - flash LED blue.
}

/**
 * ------------------------------------------------
 *      Start tasks.
 * ------------------------------------------------
 * 
 * Tasks: Create, suspend, & print status.
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    Global vars: Task handles.
 * @see    setup().
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate.
 */
void startTasks() {

    // -- LOOP status LED. --
    xTaskCreate(loopStatusLedTask, "LOOP status LED", 2048, NULL, 2, &loopStatusLEDtaskHandle);
    Serial.println("Task started: \"loop() LED\".");
}

/**
 * ------------------------------------------------
 *      Begin & start BLE interface.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 * @link   https://github.com/avinabmalla/ESP32_BleSerial.
 */
void startBLE() {

    // --- Begin BLE interface and start. ---
    Serial.print("Start Bluetooth LE");
    ble.begin(BLE_NAME);
    Serial.printf(" \"%s\".\n", BLE_NAME);
}

/**
 * ------------------------------------------------
 *      Start loop().
 * ------------------------------------------------
 *
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-03:30pm] New.
 * @see    setup().
 */
void startLoop() {
    ws2812LedColor = RED;
    ws2812LedBlink = true;
    Serial.println("Loop() started.\n");
    inLoop = true;
}

/**
 * ============================================================================
 *                          Setup.
 * ============================================================================
 *
 * @since  3.0.9 [2025-12-03-04:45pm] New.
 */
void setup() {
    startSerialUsbMonitor();            // Start serial USB monitor.
    initVars();                         // Initialize global vars.
    configPins();                       // Initialize pins & pin values.
    startSerialInterfaces();            // Start serial interfaces.
    startI2C();                         // Start I2C interface.
    startBLE();                         // Start BLE interface.  
    startTasks();                       // Start tasks.
    startLoop();                        // On to loop().
}

/**
 * ============================================================================
 *                          Task functions.
 * ============================================================================
 */

 /**
 * ------------------------------------------------
 *      Task - loop status LED.
 * ------------------------------------------------
 *
 * @param  void  * pvParameters Pointer to task parameters.
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    startTasks().
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 */
void loopStatusLedTask(void * pvParameters) {
    u8_t bright = 18;                                   // 0-255.
    while(true) {
        switch (ws2812LedColor) {
            case RED:
                rgbLedWrite(LED_BUILTIN, bright, 0, 0);  // red, green, blue.
                break;
            case YELLOW:
                rgbLedWrite(LED_BUILTIN, bright, bright, 0);
                break;
            case GREEN:
                rgbLedWrite(LED_BUILTIN, 0, bright, 0);
                break;
            case BLUE:
                rgbLedWrite(LED_BUILTIN, 0, 0, bright);
                break;
            case WHITE:
                rgbLedWrite(LED_BUILTIN, 0, 0, 0);
                break;
        }
        vTaskDelay(LED_TIME_FLASH_ON);                  // LED remains on.
        if (ws2812LedBlink == true) {
            rgbLedWrite(LED_BUILTIN, 0, 0, 0);          // LED off.
            vTaskDelay(LED_TIME_FLASH_OFF);             // LED remains off.
        }
    }
}

/**
 * ============================================================================
 *                          Loop functions.
 * ============================================================================
 */

/**
 * ------------------------------------------------
 *      Check serial monitor (USB) for input.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-17-06:00pm] New.
 * @see    loop().
 */
void checkSerialMonitor() {

    // --- Local vars. ---
    static char monitorCommand[11];
           char incomingByte;
           uint8_t whichMonitorCommand = 0;

    // --- Read bytes. ---
    if (Serial.available() > 0) {
        incomingByte = Serial.read();

        // -- Process input buffer. --
        if ((incomingByte == '\n') || (incomingByte == '\r'))  {
            memset(monitorCommand, '\0', sizeof(monitorCommand));
            strncpy(monitorCommand, monitorBuffer, posn);
            posn = 0;
            memset(monitorBuffer, '\0', sizeof(monitorBuffer));
            if (strstr(monitorCommand,"?") != NULL) {                                       // List commands.
                Serial.print("\nValid commands: ");
                for (size_t i = 0; i < NUM_COMMANDS-1; i++) {                               // Loop command array.
                    if ((i != 0) && (i % 7 == 0)) {                                         // List a max of (7) commands per line.
                        Serial.println();
                    }
                    Serial.printf("%s, ", COMMANDS[i]);
                }
                Serial.printf("%s.\n! to quit.\n", COMMANDS[NUM_COMMANDS-1]);
            } else if (strstr(monitorCommand,"!") != NULL) {                                // Disable all debugs.
                debugBT = false;
                Serial.println("\nAll debugging disabled.");
            } else {
                whichMonitorCommand = 99;                                                   // Check which command.
                for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++) {       // Loop commands.
                    if (strcmp(monitorCommand, COMMANDS[i]) == 0) {                         // Match a valid command.
                        whichMonitorCommand = i;
                        switch (whichMonitorCommand) {
                            case 0:                                                         // Display data sent to Bluetooth.
                                debugBT = (debugBT == true) ? false : true;                 // Flip the debug flag.
                                Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (debugBT ? "enabled." : "disabled."));
                                break;
                            case 1:                                                         // Reset MCU.
                                reset = (reset == true) ? false : true;                     // Flip the debug flag.
                                Serial.printf("%s %s\n", COMMANDS[whichMonitorCommand], (reset ? "enabled." : "disabled."));
                                Serial.println("Restarting ...");
                                esp_restart();                                              // Reset MCU.
                        }
                    }
                }
                if ((whichMonitorCommand > 1) && (strlen(monitorCommand) > 0))  {
                    Serial.printf("\n\"%s\" is not a valid command. \n", monitorCommand);   // Invalid command.
                }
            }
            memset(monitorCommand, '\0', sizeof(monitorCommand));                           // Reset buffer.  
        } else {
            monitorBuffer[posn] = incomingByte;                                             // Add to input buffer.
            posn++;
        }
    }
}

/**
 * ============================================================================
 *                          Loop.
 * ============================================================================
 *
 * @since 3.0.9 [2025-12-17-10:00am] New.
 */
void loop() {
    checkSerialMonitor();               // Check serial monitor (USB) for input.
}

