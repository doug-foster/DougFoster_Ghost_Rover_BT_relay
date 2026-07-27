/**
 * *************************************************************************
 *  GhostRover 3 - NMEA sentence relay from ZED-F9P to Bluetooth LE.
 * *************************************************************************
 *
 * @author D. Foster <doug@dougfoster.me>.
 * @since  3.0.12 [2026-02-10-03:45pm].
 * @since  3.1.2  [2026-07-27-02:00pm] Regroup. Cleanup.
 * @since  3.2.1 [2026-06-25-01:00pm] Updated GR-MCU2 LED status.
 * @see    https://github.com/doug-foster/DougFoster_Ghost_Rover.
 * @see    https://github.com/doug-foster/DougFoster_Ghost_Rover_BT_relay.
 * @see    https://github.com/doug-foster/DougFoster_Ghost_Rover_EVK_RTCM_relay.
 * @link   http://dougfoster.me.
*/

/**
 * =========================================================================
 *  Docs.
 * =========================================================================
 * 
 * @since 3.1.1  [2026-06-26-10:00am] New.
 * 
 * --- Comments. ---
 * --- Code structure. ---
 * --- Code operation. ---
 * --- Board LED status. ---
 * --- ESP32 (Arduino framework) data types. ---
 * --- Other. ---
 */

 /*
 * -------------------------------------------------------------------------
 *  Comments.
 * -------------------------------------------------------------------------
 * 
 * @since 3.1.1  [2026-06-26-10:00am] New.
 * @link https://github.com/doug-foster/DougFoster_Ghost_Rover.
 * 
 * --- Description & operation. ---
 * -- Primary use for GR-MCU2 is to send NMEA sentences over Bluetooth:
 *    GNSS receiver --I2C--I2C--> GR-MCU1 --I2C1--I2C--> GR-MCU2 --> Bluetooth. 
 *
 * --- Major components. ---
 *     -- MCU https://www.sparkfun.com/sparkfun-thing-plus-esp32-s3.html.
 *
 * --- Other components. ---
 * --- Misc references. ---
 *
* --- Dev environment. ---
 *     -- IDE         VS Code & Arduino Maker Workshop 1.1.8 extension (uses Arduino CLI 1.2.0).
 *     -- Platform    https://github.com/espressif/arduino-esp32/releases/latest (Arduino Release v3.3.10 based on ESP-IDF v5.5.4).
 *
 * --- Caveats. --- // ToDo: ok to delete?
 *     -- https://github.com/avinabmalla/ESP32_BleSerial/issues/24.
 *        ESP32_BleSerial version 3.0.0 requires Espressif core version 3.2.1.
 *        Espressif core version 3.3.x produces error: 'ESP_GATT_MAX_ATTR_LEN'
 *          was not declared in this scope.
 *        https://github.com/avinabmalla/ESP32_BleSerial/issues/25.
 *        BleBufferedSerial .end causes a crash.
 * 
 * --- TODO: ---
 *     1. Review & complete ToDo comments.
*/

/**
 * -------------------------------------------------------------------------
 *  Code structure.
 * -------------------------------------------------------------------------
 * 
 * @since 3.1.1 [2026-06-25-01:00pm] New.
 *
 *  --- Docs. ---
 *  --- Include libraries. ---
 *      -- Core.
 *      -- Additional.
 *  --- Global vars.---
 *      -- Pin assignments.
 *      -- LED.
 *      -- BLE (Bluetooth Low Energy).
 *      -- Task handles.
 *      -- Operation.
 *      -- Declaration.
 *      -- Test.
 *  --- General functions. ---
 *      -- statusLedOn()          - Turn on status LED.
 *  --- Setup functions. ---
 *      -- showBuild()            - Display build & processor info.
 *      -- startSerial()          - Start serial interfaces.
 *      -- initPins()             - Initialize pins & pin values.
 *      -- startI2C()             - Start I2C wire interfaces.
 *      -- startTasks()           - Start tasks.
 *      -- startBLE()             - Begin & start BLE interface.
 *      -- preLoop()              - Prepare for loop().
 *  --- Task functions. ---
 *      -- taskLoopStatusLed()    - Loop() status LED.
 *  --- Event handlers. ---
 *      -- receiveEvent()         - I2C receive event function.
 *  --- Loop functions. ---
 *      -- checkSerialUSB();      - Check serial USB for input.
 *      -- checkNMEAin()          - Check if last NMEA sent to MCU #2 was within NMEA_TIMEOUT.
 *      -- debug();               - Display debug.
 *  --- Setup. ---
 *  --- Loop. ---
 */

/**
 * -------------------------------------------------------------------------
 *  Code Operation.
 * -------------------------------------------------------------------------
 *
 * @since 3.1.1 [2026-06-26-11:00am] New.
 *
 * --- Boot. ---
 *     Include libraries.
 *     Define global vars.
 *     Define general functions.
 *     Define setup() functions.
 *     Define task functions.
 *     Define event handlers.
 *     Define loop() functions.
 * --- Run setup(). ---
 *     showBuild()                  // Display build & processor info.
 *     initPins()                   // Initialize pins & pin values.
 *     startSerial()                // Start serial interfaces.
 *     startI2C()                   // Start I2C wire interfaces.
 *     startBLE()                   // Start BLE interface.  
 *     startTasks()                 // Start tasks.
 *     preLoop()                    // Prepare for loop().
 * --- Run loop(). ---    
 *     checkSerialUSB()             // Check serial USB for input.
 *     checkNMEAin()                // Check if last NMEA received from MCU #2 was within NMEA_TIMEOUT.
 *     debug()                      // Display debug.
 * --- Tasks. ---
 *     taskLoopStatusLed()          // Loop() status LED.
 * --- Event handlers. ---
 *     receiveEvent()               // I2C receive event function.
 */

/**
 * -------------------------------------------------------------------------
 *  Board LED status.
 * -------------------------------------------------------------------------
 *
 * @since 3.1.1 [2026-06-25-01:00pm] New.
 * @since 3.2.1 [2026-06-25-01:00pm] Updated GR-MCU2 LED status.
 *
 *   ws2812LedColor = RED, YELLOW, GREEN, BLUE, WHITE.
 *   ws2812LedBlink = true, false.
 * 
 * --- GR-MCU1 ----
 * @see DougFoster_Ghost_Rover.ino.
 *
 * --- GR-MCU2 ----
 *     -- setup(). --
 *        - solid YELLOW: not used.
 *        - solid  WHITE: setup() started & running ok.
 *        - solid    RED: not used.
 *     -- loop(). --  
 *        - solid   BLUE: exiting setup(), entering loop().
 *        - solid  GREEN: loop running ok.
 *        - blink  GREEN: receiveEvent(): RTCM in I2C, out BT.
 *        - solid    RED: checkNMEAin(): NMEA timeout error.
 * 
 * --- GNSS ----
 * @see DougFoster_Ghost_Rover.ino.
 */

/**
 * -------------------------------------------------------------------------
 *  ESP32 (Arduino framework) data types.
 * -------------------------------------------------------------------------
 *
 * @since 3.0.12 [2026-02-20-09:00am] New.
 * 
 * --- Unsigned integer. ---
 *     uint8_t                      %u         8 bits = 1 byte,  0 to 255.
 *     uint16_t/unsigned short      %u        16 bits = 2 bytes, 0 to 65,535.
 *     uint32_t/unsigned long       %u,%lu    32 bits = 4 bytes, 0 to 4,294,967,295.
 *     size_t (size,length,count)   %zu       32 bits = 4 bytes, 0 to 4,294,967,295.
 *     uint64_t/unsigned long long  %llu      64 bits = 8 bytes, 0 to 18,446,744,073,709,551,615.
 *
 * --- Signed integer. ---
 *     int8_t                       %d         8 bits = 1 byte,            -128 to 127.
 *     int16_t/short                %d        16 bits = 2 bytes,        -32,768 to 32,767.
 *     int32_t/int/long             %d,%ld    32 bits = 4 bytes, -2,147,483,648 to 2,147,483,647.
 *     int64_t/long long            %lld      64 bits = 8 bytes,      -9.22e+18 to 9.22e+18.
 *
 * --- Signed decimal/floating point. ---
 *     float                        %f        32 bits = 4 bytes,   6-7 sig. digits (hardware),  -3.40e+38 to 3.40e+38).
 *     double/long double           %f,%lf    64 bits = 8 bytes, 15-17 sig. digits (software), -1.79e+308 to 1.79e+308).
 *
 * --- Character/text. ---
 *     char (signed)                %c         8 bit = 1 byte,  -128 to 127.
 *     unsigned char                %c         8 bit = 1 byte,     0 to 255.
 *
 * --- Other. ---
 *     bool                        %d (0/1)   8 bit = 1 byte,  true or false.
 *     bool                        %s (text)  8 bit = 1 byte,  true or false.
 *     void                        n/a.
 *     array                       n/a.
 *     string                      %s
 */

/**
 * =========================================================================
 *  Include libraries.
 * =========================================================================
 *
 * @since         3.0.9 [2025-12-17-10:00am] New.
 * @since         3.2.1 [2026-07-27-02:30pm] Updated library <BleSerial.h> from 3.0.0 to 3.0.1.
 * @link  Arduino https://docs.arduino.cc/libraries/.
 * @link  ESP32   https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html.
 * @link  I2C     https://docs.arduino.cc/language-reference/en/functions/communication/wire/.
 */

// --- Core. ---
#include <Arduino.h>            // https://github.com/espressif/arduino-esp32.
#include <Wire.h>               // https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h.
#include <esp_chip_info.h>      // https://github.com/pycom/pycom-esp-idf.

// --- Additional. ---
#include <BleSerial.h>          // https://github.com/avinabmalla/ESP32_BleSerial (3.0.1).

/**
 * =========================================================================
 *  Global vars.
 * =========================================================================
 *
 * @since 3.0.9  [2025-12-17-10:00am] New.
 * @since 3.0.11 [2026-01-18-04:00pm] Match rover basic functionality.
 * @since 3.0.12 [2026-02-10-03:45pm] Status LED changes.
 */

// --- Pin assignments. ---

// --- LED. ---
bool  ws2812LedBlink     = false;
const uint8_t LED_BRIGHT = 50;                      // 0-255. taskLoopStatusLed()
enum ws2812_LED_COLOR {                             // WS2812 RGB STAT LED, pin=RGB_BUILTIN=IO23.
    OFF,
    RED,
    YELLOW,
    GREEN,
    BLUE,
    WHITE
} ws2812LedColor;

// --- BLE (Bluetooth Low Energy). ---
BleSerial ble;                                      // BLE object.

// --- Task handles. ---
TaskHandle_t taskLoopStatusLedHandle;               // Task: Loop status LED.

// --- Operation. ---
enum CommandIndex {                                 // Readable index for command array.
    DEBUG_NMEA,                                     // 0.
    SHOW_UPTIME,                                    // 2.
    RESET,                                          // 3.
    DEBUG_NMEA_HEX,                                 // 4.
    NUM_COMMANDS                                    // 5 = automatic array length.
};      
const char* COMMAND[NUM_COMMANDS] = {               // Command strings; match CommandIndex.
    "debugNMEA",                                    // DEBUG_NMEA.
    "showUpTime",                                   // SHOW_UPTIME.
    "reset",                                        // RESET.
    "debugNMEAhex",                                 // DEBUG_NMEA_HEX.
};
    bool    commandFlag[NUM_COMMANDS] = {false};    // Command flags.
    bool    i2cUp  = false;                         // Status: true/false if both Wire & Wire1 up.
    bool    NMEAin = false;                         // Status: true/false if NMEA received from MCU #1 within NMEA_TIMEOUT.
    bool    inLoop = false;                         // In loop indicator.
    char    serialState[4];                         // Serial state: [USB] [S0] [S1] [S2]; value = u, d, or -.
    int64_t startTime;                              // Boot time.
    int64_t lastNMEAtime = 0;                       // Time last NMEA received from MCU #1.

// --- Declaration. ---
// --- Test. ---

/**
 * =========================================================================
 *  General functions.
 * =========================================================================
 *
 * @since  3.0.12 [2026-02-06-04:00pm] New.
 * @see   checkPrefs() - Check preferences.
 */

  /**
 * ------------------------------------------------
 *      Turn on status LED
 * ------------------------------------------------
 * 
 * @return void  No output is returned.
 * @since  3.0.12 [2026-02-10-10:45pm] New.
 * @see    
 */
void statusLedOn() {
    switch (ws2812LedColor) {
        case RED:
            rgbLedWrite(LED_BUILTIN, LED_BRIGHT, 0, 0);         // red, green, blue.
            break;
        case YELLOW:
            rgbLedWrite(LED_BUILTIN, LED_BRIGHT, LED_BRIGHT, 0);
            break;
        case GREEN:
            rgbLedWrite(LED_BUILTIN, 0, LED_BRIGHT, 0);
            break;
        case BLUE:
            rgbLedWrite(LED_BUILTIN, 0, 0, LED_BRIGHT);
            break;
        case WHITE:
            rgbLedWrite(LED_BUILTIN, LED_BRIGHT, LED_BRIGHT, LED_BRIGHT);
            break;
    }
}

/**
 * =========================================================================
 *  Setup functions.
 * =========================================================================
 *
 * @since 3.0.11 [2026-01-08-10:30am] Browser initiated updates.
 * @see   showBuild()   - Display build & processor info.
 * @see   startSerial() - Start serial interfaces.
 * @see   initPins()    - Initialize pins & pin values.
 * @see   startI2C()    - Start I2C wire interfaces.
 * @see   startTasks()  - Start tasks.
 * @see   startBLE()    - Begin & start BLE interface.
 * @see   preLoop()     - Prepare for loop().
 */

 /**
 * -------------------------------------------------------------------------
 *  Display build & processor info.
 * -------------------------------------------------------------------------
 * 
 * Default pins for ESP32-S3 Thing Plus using Arduino core:
 *   GPIO 19 - Serial USB UART0 used as Communication Device Class interface D- (negative data line).
 *   GPIO 20 - Serial USB UART0 used as Communication Device Class interface D+ (positive data line).
 * 
 * @return void  No output is returned.
 * @since  3.0.11 [2026-01-18-05:00pm] New.
 * @since  3.0.12 [2026-02-10-10:45pm] Status LED changes.
 * @since  3.1.1  [2026-06-26-01:30pm] Replaced with showBuild() from DougFoster_Ghost_Rover.ino.
 * @see    setup().
 * @link   https://github.com/pycom/pycom-esp-idf.
 */
void showBuild() {

    // --- Local vars. ---
    const uint8_t MAJOR_VERSION     = 3;
    const uint8_t MINOR_VERSION     = 2;
    const uint8_t PATCH_VERSION     = 1;
    const char NAME[]               = "Ghost Rover 3 - BT Relay";
    const uint32_t SERIAL_USB_SPEED = 115200;   // Serial USB speed.
    const uint64_t  START_DELAY     = 4000000;  // 4 second startup delay.
          char    buildString[40]   = {'\0'};   // Build string (build version on date at time). e.g. 3.0.12 - Feb 19 2026 @ 12:23:13
          esp_chip_info_t chip_info;

    // --- Run. ---
    startTime = esp_timer_get_time();
    ws2812LedColor = YELLOW;
    ws2812LedBlink = false;
    statusLedOn();
    Serial.begin(SERIAL_USB_SPEED);
    serialState[0] = 'u';   // Serial USB is up [u] [S0] [S1] [S2].
    esp_chip_info(&chip_info);
    sprintf(buildString, "%u.%u.%u - %s @ %s", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION, __DATE__, __TIME__);
    while ((esp_timer_get_time() - startTime) < START_DELAY) {
        vTaskDelay(1);  // busy-wait; yield to RTOS if needed
    }
    ws2812LedColor = WHITE;
    Serial.print("\033[2J");   // Clear screen.
    Serial.printf("%s\n%s\n", NAME, buildString);
    Serial.printf("Using %s, Rev %d, %d core(s), ID (MAC) %012llX.\n", ESP.getChipModel(), chip_info.revision, chip_info.cores, ESP.getEfuseMac());
    Serial.println("setup() started.");
    Serial.printf("Serial (USB) started @ %u bps.\n", SERIAL_USB_SPEED);
}

/**
 * -------------------------------------------------------------------------
 *  Start serial interfaces.
 * -------------------------------------------------------------------------
 * 
 * Serial interfaces are not used. NMEA in is on I2C1 bus connected to I2C2 bus on GRMCU1.
 *
 * @return void  No output is returned.
 * @since  3.0.11 [2026-01-18-05:00pm] New.
 * @see    showBuild().
 * @see    setup().
 * @link   https://github.com/G6EJD/ESP32-Using-Hardware-Serial-Ports.
 * @link   https://randomnerdtutorials.com/esp32-uart-communication-serial-arduino/#esp32-custom-uart-pins.
 */
void startSerial() {

    // --- Start serial interfaces. ---
    serialState[1] = '-';   // Serial0 is not used: [USB][-][S1][S2].
    Serial.println("Serial0 is not used.");
    serialState[2] = '-';   // Serial1 is not used: [USB][S0][-][S2].
    Serial.println("Serial1 is not used.");
    serialState[3] = '-';   // Serial2 is not used: [USB][S0][S1][-].
    Serial.println("Serial2 is not used.");
}

/**
 * -------------------------------------------------------------------------
 *  Initialize pins & pin values.
 * -------------------------------------------------------------------------
 * 
 * Serial interfaces are not used. NMEA in is on I2C1 bus connected to I2C2 bus on GRMCU1.
 *
 * @return void No output is returned.
 * @since  3.0.11  [2026-01-18-05:00pm] New.
 * @see    setup().
 */
void initPins() {
    Serial.println("Init pins - no pins.");
}

/**
 * -------------------------------------------------------------------------
 *  Start I2C wire interfaces.
 * -------------------------------------------------------------------------
 * 
 * Default pins for ESP32-S3 Thing Plus using Arduino core:
 *   GPIO  8 - SDA for I2C0 {Qwiic}.
 *   GPIO  9 - SCL for I2C0 {Qwiic}.
 *   GPIO 17 - SDA for I2C1 {PTH} (also default for Serial2 TX).
 *   GPIO 18 - SCL for I2C1 {PTH} (also default for Serial2 RX).
 *
 * @return void  No output is returned.
 * @since  3.0.11  [2026-01-18-09:45pm] New.
 * @see    setup().
 * @link   https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h.
 * @link   https://docs.arduino.cc/language-reference/en/functions/communication/wire/. 
 */
void startI2C() {

    // --- Local vars. ---
    const uint8_t  I2C_SLAVE_ADDRESS = 8;       // I2C slave address.
    const uint16_t RETRY             = 500;     // Try restarting I2C interfaces.
    const uint32_t WIRE_SPEED        = 400000;  // I2C Fast mode (4kHz).

    // --- Start interfaces. ---
    i2cUp = false;
    if (Wire.begin(I2C_SLAVE_ADDRESS)) {
        Wire.setClock(WIRE_SPEED);
        Serial.printf("Wire started @ 4kHz.\n");
        i2cUp = true;
    } else {
        Serial.println("Wire failed to start. Retrying.");
        delay(RETRY);
        startI2C();
    };

    // --- Register event functions. ---
    Wire.onReceive(receiveEvent);
    // Wire.onRequest(requestEvent);
    Serial.println("Wire receive/request event functions registered.");
}

/**
 * -------------------------------------------------------------------------
 *  Start tasks.
 * -------------------------------------------------------------------------
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

    // --- LOOP status LED. ---
    xTaskCreate(taskLoopStatusLed, "LOOP status LED", 2048, NULL, 2, &taskLoopStatusLedHandle);
    Serial.println("Task \"Loop status LED\" started.");
}

/**
 * -------------------------------------------------------------------------
 *  Begin & start BLE interface.
 * -------------------------------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @see    setup().
 * @link   https://github.com/avinabmalla/ESP32_BleSerial.
 */
void startBLE() {

    // --- Local vars. ---
    const char BLE_NAME[] = "GhostRover";              // BLE name.

    // --- Begin BLE interface and start. ---
    Serial.print("Start Bluetooth LE");
    ble.begin(BLE_NAME);
    Serial.printf(" \"%s\".\n", BLE_NAME);
}

/**
 * -------------------------------------------------------------------------
 *  Prepare for loop().
 * -------------------------------------------------------------------------
 *
 * @return void   No output is returned.
 * @since  3.0.11 [2026-01-18-05:30pm] New.
 * @since  3.0.12 [2026-02-10-03:45pm] Status LED changes.
 * @see    setup().
 */
void preLoop() {
    ws2812LedColor = BLUE;
    ws2812LedBlink = false;
    inLoop = true;
    Serial.println("Loop() starting.");
}

/**
 * =========================================================================
 *  Task functions.
 * =========================================================================
 * 
 * @since 3.0.11 [2026-01-08-10:30am] New.
 * @see   startTasks()        - Start tasks in setup().
 * @see   taskLoopStatusLed() - Status LED for loop().
 */

 /**
 * -------------------------------------------------------------------------
 *  Task: Loop() status LED.
 * -------------------------------------------------------------------------
 *
 * @param  void  * pvParameters Pointer to task parameters.
 * @return void  No output is returned.
 * @since  3.0.9 [2025-12-03-02:45pm] New.
 * @since  3.0.12 [2026-02-10-10:45pm] Status LED changes.
 * @see    startTasks().
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 */
void taskLoopStatusLed(void * pvParameters) {

    // --- Local vars. ---
    const TickType_t DELAY      = 40/portTICK_PERIOD_MS;            // Timer (ms) =  0.04 seconds.

    // --- Loop. ---
    while(true) {
        statusLedOn();
        vTaskDelay(DELAY);                                          // LED remains on.
        if (ws2812LedBlink == true) {
            rgbLedWrite(LED_BUILTIN, 0, 0, 0);                      // LED off.
            ws2812LedBlink = false;
        }
    }
}

/**
 * =========================================================================
 *  Event handlers.
 * =========================================================================
 *
 * @since 3.0.11 [2026-01-12-06:00pm] Browser initiated updates.
 * @see   receiveEvent() - I2C receive event function.
 * @see   requestEvent() - I2C request event function.
 */

/**
 * -------------------------------------------------------------------------
 *  I2C receive event function.
 * -------------------------------------------------------------------------
 *
 * Executes when data is received from I2C master.
 * 
 * @return void  No output is returned.
 * @since  3.0.11 [2026-01-18-11:00pm] New.
 * @see    setup().
 */
void receiveEvent(int bytesIn) {

    // --- Local vars. ---
    static char    nmeaBuffer[120]  = {'\0'};               // Buffer for NMEA sentence.
    static size_t  nmeaCount        = 0;

    // --- Read NMEA in. ---
    for (size_t i = 0; i <= bytesIn; i++) {
      char incoming = Wire.read();
      if (incoming != 255) {                                // Not sure where these are coming from.
        nmeaBuffer[i] = incoming;
      }
    }

    // --- Write NMEA out over BT. ---
    ble.print(nmeaBuffer);
    ws2812LedColor = GREEN;
    ws2812LedBlink = true;                                  // Flash the status LED.
    if (commandFlag[DEBUG_NMEA]) {                          // Debug.
        if (strncmp("$GNGGA", nmeaBuffer, 6) == 0) {
            Serial.print('\n');
        }
        Serial.printf("%u %s", nmeaCount, nmeaBuffer);      // Display NMEA sentence (nmeaBuffer already ends with [CR][LF]).
    }
    if (commandFlag[DEBUG_NMEA_HEX]) {
        if (strncmp("$GNGGA", nmeaBuffer, 6) == 0) {
            Serial.print('\n');
        }
        Serial.printf("%u %s", nmeaCount, nmeaBuffer);      // Display NMEA sentence (nmeaBuffer already ends with [CR][LF]).
        for (int i = 0; i < strlen(nmeaBuffer); i++) {      // Display NMEA sentence characters in hex.
            Serial.printf("[\"%c\" 0x%02X] ",nmeaBuffer[i], nmeaBuffer[i]);
        }
        Serial.println('\n');
    }

    // --- Prepare for next receive. ---
    nmeaCount++;
    NMEAin = true;
    lastNMEAtime = esp_timer_get_time();
    memset(nmeaBuffer, '\0', sizeof(nmeaBuffer));
}

/**
 * =========================================================================
 *  Loop functions.
 * =========================================================================
 * 
 * @see checkSerialUSB();         - Check serial USB for input.
 * @see checkNMEAin()             - Check if last NMEA sent to MCU #2 was within NMEA_TIMEOUT.
 * @see debug();                  - Display debug.
 */

/**
 * -------------------------------------------------------------------------
 *  Check serial USB for input.
 * -------------------------------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.11 [2026-01-18-10:15pm] New.
 * @see    loop().
 */
void checkSerialUSB() {

    // --- Local vars. ---
    static size_t posn        = 0;                              // Input position for command buffer.
    static char   command[20] = {'\0'};                         // Serial USB command buffer.
    static char   inputChar   = '\0';

    if (Serial.available() == 0) {                              // Nothing to see, move on.
        return;
    }

    // --- Fill command buffer. ---
    while ((Serial.available() > 0) )  {
        inputChar = Serial.read();                              // Read char from USB Serial.
        if ((inputChar != '\n') && (inputChar != '\r')) {
            command[posn] = inputChar;                          // Add input to buffer.
            posn++;
        }
    }

    // --- Process command. ---
    if (inputChar == '\n')  {
        if ((command[0]) == '?') {                              // List commands.
            Serial.print("\nGR-MCU2 commands:");
            for (size_t i = 0; i <= NUM_COMMANDS-1; i++) {
                Serial.printf(" %s", COMMAND[i]);
            }
            Serial.println('.');
        } else if ((command[0]) == '!') {                       // Disable all debugs.
            for (size_t i = 0; i <= NUM_COMMANDS; i++) {
                commandFlag[i] = false;
            }
            Serial.println("All debugging disabled.");
        } else {                                                // Possible command.
            size_t i;
            for (i = 0; i < NUM_COMMANDS; i++) {
                if (strcmp(COMMAND[i], command) == 0) {
                    break;
                }
            }
            if (i == NUM_COMMANDS) {                            // Invalid command.
                Serial.printf("%s is not a command. \n", command);
            } else {
                commandFlag[i] = !commandFlag[i];               // Toggle the debug flag.
                Serial.printf("%s %s\n", COMMAND[i], (commandFlag[i]  ? "enabled." : "disabled."));
            }
        }
        posn = 0;                                               // Prepare for next command.
        memset(command, '\0', sizeof(command));
        inputChar = 0;
    }
}

/**
 * -------------------------------------------------------------------------
 *  Check if last NMEA received from MCU #2 was within NMEA_TIMEOUT.
 * -------------------------------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.11 [2026-01-19-03:15pm] New.
 * @since  3.0.12 [2026-02-10-03:45pm] Status LED changes.
 * @see    receiveEvent().
 */
void checkNMEAin() {

    // --- Local vars. ---
    const int64_t  NMEA_TIMEOUT = 3000000;                          // Time (us) not to exceed for NMEA received from MCU #1 (3 sec).

    // --- Check for timeout. ---
    if ((esp_timer_get_time()-lastNMEAtime) > NMEA_TIMEOUT) {       // RTCM received from MCU #1 within NMEA_TIMEOUT?
        NMEAin = false;
        ws2812LedColor = RED;
        ws2812LedBlink = false;
    }
}

/**
 * -------------------------------------------------------------------------
 *  Display debug.
 * -------------------------------------------------------------------------
 *
 * @return void No output is returned.
 * @since  3.0.11 [2026-01-18-10:15pm] New.
 * @see    checkSerialUSB().
 */

void debug() {

    // --- Local vars. ---
    const  int64_t THROTTLE_DEBUG = 1000000;                            // Time (us) between debug() = (every 1 sec).
    static int64_t lastThrottleTime = esp_timer_get_time();             // Throttle. Initialize only once, then persist.
           int64_t lastTime;

    // --- Throttle loop() calls. ---
    if ((esp_timer_get_time() - lastThrottleTime) < THROTTLE_DEBUG) {   // Not time to run.
        return; 
    }
    lastThrottleTime = esp_timer_get_time();                            // Time to run. Reset timer.

    // --- NMEA. ---
    // @see "if (commandFlag[DEBUG_NMEA])" in receiveEvent() event handler.

    // --- Uptime. ---
    if (commandFlag[SHOW_UPTIME]) {
        int32_t seconds = (esp_timer_get_time() - startTime)/1000000;
        int32_t minutes = seconds / 60;
        int32_t hours = minutes / 60;
        Serial.printf("Uptime: %u hrs %u min %u sec\n", hours % 24, minutes % 60, seconds % 60);
    }

    // --- Reset. ---
    if (commandFlag[RESET]) {
        Serial.println("Restarting ...");
        esp_restart();
    }
}

/**
 * =========================================================================
 *  Setup.
 * =========================================================================
 *
 * @since  3.0.9 [2025-12-03-04:45pm] New.
 * @see    Global vars.
 */
void setup() {
    showBuild();             // Display build & processor info.
    initPins();              // Initialize pins & pin values.
    startSerial();           // Start serial interfaces.
    startI2C();              // Start I2C wire interfaces.
    startBLE();              // Start BLE interface.  
    startTasks();            // Start tasks.
    preLoop();               // Prepare for loop().
}

/**
 * =========================================================================
 *  Loop.
 * =========================================================================
 *
 * @since 3.0.9 [2025-12-17-10:00am] New.
 * @see   startTasks().
 * @see   Task functions.
 * @see   Event handlers.
 */
void loop() {
    checkSerialUSB();         // Check serial USB for input.
    checkNMEAin();            // Check if last NMEA received from MCU #2 was within NMEA_TIMEOUT.
    debug();                  // Display debug.
}
