/**
 * ***********************************
 *      Ghost Rover - NMEA to Bluetooth relay.
 * ***********************************
 * 
 * @author   D. Foster <doug@dougfoster.me>.
 * @since    0.1.2 [2025-07-11-06:15pm] New.
 * @link     http://dougfoster.me.
 *
 * ===================================
 *      Comments.
 * ===================================
 * 
 * --- Description & operation. ---
 *     -- NMEA sentences in:
 *        RTK-SMA TX2 (UART2) -> ESP32-S3 PTH 16 (UART0) -> ESP32-S3 PTH 14 (UART1) -> ESP32C6 D7 (BLE).
 *     -- NEMA ACK out:
 *        (1) ACK for every LED_TRIGGER_COUNT number of NMEA sentences sent out the BLE interface.
 *        Used by ESP32-S3 to trigger BT LED flash.
 *        ESP32C6 D6 -> ESP32-S3 PTH 15 (UART1).
 *     -- The BleSerial library uses a large amount of program storage space. In Ghost Rover 0.5.2, BleSerial
 *        was run on the ESP32-S3. However, this did not leave enough space to add additional libraries (wifi,
 *         web, L53L1X, ...). As a result, BleSerial was moved to a dedicated ESP32C6 and NMEA sentences passed
 *        in over a serial connection from the ESP32-S3 in to NMEA_RX.
 *
 * --- Major components. ---
 *     -- XIAO_ESP32C6   https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/.
 *
 * --- Other components. ---
 *     -- Sparkfun ESP32-S3 Thing+  https://www.sparkfun.com/sparkfun-thing-plus-esp32-s3.html.
 *
 * --- Misc. references. ---
 *     -- TBD
 * 
 * --- Dev environment. ---
 *     -- IDE: Arduino 2.3.6.
 *     -- Board: "XIAO_ESP32C6" (~/Library/Arduino15/packages/esp32/hardware/esp32/3.2.1/boards.txt).
 *     -- VS Code 1.100.2 (Extensions: Better Comments, Bookmarks, C/C++, C/C++ Extension Pack, C/C++ Themes,
 *        CMake Tools, Dash, Diff Folders, Git Graph, GitHub Theme, GitLens, Markdown All in One, Serial Monitor,
 *        SFTP).
 *     -- GitHub repo: TBD.
 * 
 * --- Caveats. ---
 *     -- Only 1 HardwareSerial UART (UART0) is available on the XIAO_ESP32C6.
 *     -- For some unknown reason, ble.begin() in startBT() _MUST_ be run before Serial.begin() in
 *        beginSerialMonitor()when using an ESP32-C6 variant. If it is not, the flash uploader hangs at 100% during
 *        new code upload and host port used for the Serial monitor (USB) dissappears. This is not the case when run
 *        on the ESP32-S3 variant: start order does not matter.  // TODO: #1.
 *
 * --- TODO: ---
 *     1. Determine why running ble.begin() before Serial.begin() is necessary.
 *
 * --- Code flow. ---
 *     -- Include libraries.
 *     -- Global vars: define vars, set constants, declarations.
 *     -- Functions: init, config, begin, start, check, display, callback, operation, tasks, test.
 *     -- Setup.
 *     -- Loop.
 */

// ===================================
//      Include libraries.
// ===================================

// --- Libraries. ---
#include <HardwareSerial.h>     // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.cpp.
#include <BleSerial.h>          // https://github.com/avinabmalla/ESP32_BleSerial (2.0.1).

// ===================================
//      Global vars.
// ===================================

// -- Version. --
const char BUILD_DATE[]   = "2025-07-11-18:00";     // 24hr format, need to fit max (16) characters.
const char MAJOR_VERSION  = '0';
const char MINOR_VERSION  = '1';
const char PATCH_VERSION  = '2';

// -- Communication port usage. --
// Serial monitor (USB).
//
// Serial 0 (UART0) NMEA_ACK_TX: SEEED XIAO ESP32C6 D6 -> ESP32-S3 Thing+ PTH 14 {green wire} - NMEA ACK out.
// Serial 0 (UART0) NMEA_RX:     SEEED XIAO ESP32C6 D7 <- ESP32-S3 Thing+ PTH 15 {yellow wire} - NMEA in.

// -- Pin (pth) definitions. --
const uint8_t NMEA_ACK_TX  = D6;
const uint8_t NMEA_RX      = D7;

// -- Serial monitor. --
const  uint32_t SERIAL_MON_SPEED = 115200;      // Serial USB monitor speed.

// -- Serial 0. --
const uint32_t       SERIAL0_SPEED = 115200;    // ESP32-S3 <-> ESP32C6 speed.
      HardwareSerial serial0(0);                // UART0 object.

// -- I2C. --
// Not used.

// -- Timing. --
const TickType_t LED_TIME_FLASH_ON  = 100/portTICK_PERIOD_MS;   // Time (ms).

// -- Task handles. --
TaskHandle_t nmeaBtLEDtaskHandle;               // NMEA BT LED task handle.

// -- Bluetooth (BLE). --
const char       BLE_NAME[] = "GhostRover";     // BT name.
      BleSerial  ble;                           // BT object.

// -- GNSS. --
      int64_t  latitude;                        // Latitude.
      int64_t  longitude;                       // Longitude.
      int64_t  altitude;                        // Altitude in meters above Mean Sea Level (-1.0 until set).

// -- I/O. --
      char monitorChar;                         // Monitor i/o character.
      char serial0Char;                         // Serial 0 i/o character.

// -- Operation. --
const  u_int8_t NMEA_ACK_COUNT = 5;             // Flash BT LED once for every NMEA_ACK_COUNT sentences sent.

// -- Commands. --
const  uint8_t  NUM_COMMANDS           = 3;     // How many possible commands.
const  char     EXIT_TEST              = '!';   // Exit test mode.
const  char*    commands[NUM_COMMANDS] = {      // Valid commands. Point to array of C-strings.
                "testLEDbt",
                "debugBT",
                "reset"
};
       char     monitorCommand[11];             // Serial monitor command (C-string).
       bool     testLEDbt;                      // Test Bluetooth LED.
       bool     debugBT;                        // Debug Bluetooth.
       bool     reset;                          // Reset MCU.

// -- Declarations. --                          // Eliminate compiler scope error due to definition order.
void updateLEDs(char);

// -- Test. --

// ===================================
//             Functions
// ===================================

// --- Init. ---

/**
 * ------------------------------------------------
 *      Initialize global vars.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 */
void initVars() {

    Serial.println("Running setup().");
    Serial.print("Initialize global vars");

    // -- Bluetooth. --

    // -- NMEA. --

    // -- I/O. --
    monitorChar = '\0';
    serial0Char = '\0';

    // -- Operation. --

    // -- Commands. --
    memset(monitorCommand, '\0', sizeof(monitorCommand));
    testLEDbt = false;
    debugBT   = false;
    reset     = false;

    Serial.println(".");
}

// --- Config. ---

/**
 * ------------------------------------------------
 *      Initialize pin modes & pin values.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 */
void configPins() {

    // -- Initialize pin modes. --
    Serial.print("Config pins");
    pinMode(LED_BUILTIN, OUTPUT);

    // -- Initialize pin values. --
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(".");
}

// --- Begin. ---

/**
 * ------------------------------------------------
 *      Begin serial monitor (USB).
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 */
void beginSerialMonitor() {

    Serial.begin(SERIAL_MON_SPEED);
    Serial.println("\nBluetooth LE started early.");  // TODO: #1.
    Serial.printf("Begin serial monitor (USB) @ %i bps.\n", SERIAL_MON_SPEED);
}

/**
 * ------------------------------------------------
 *      Begin serial interfaces.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 * @link   https://github.com/G6EJD/ESP32-Using-Hardware-Serial-Ports.
 * @link   https://randomnerdtutorials.com/esp32-uart-communication-serial-arduino/#esp32-custom-uart-pins.
 */
void beginSerialInterfaces() {

    // -- Serial 0 interface. --
    Serial.printf("Begin serial 0 (UART0) @ %i bps", SERIAL0_SPEED);
    serial0.begin(SERIAL0_SPEED, SERIAL_8N1, NMEA_RX, NMEA_ACK_TX);     // UART0 object. RX, TX.
    Serial.println(".");
}

/**
 * ------------------------------------------------
 *      Begin I2C interface.
 * ------------------------------------------------
 *
 * Not used.
 *
 */

// --- Start. ---

/**
 * ------------------------------------------------
 *      Begin & start Bluetooth interface.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 * @link   https://github.com/avinabmalla/ESP32_BleSerial.
 */
void startBT() {

    // -- Begin BLE interface and start. --
    // Serial.print("Start Bluetooth LE"); // TODO: #1.
    ble.begin(BLE_NAME);

    // -- Print status. --
    // Serial.printf(" \"%s\".\n", BLE_NAME); // TODO: #1.
}

/**
 * ------------------------------------------------
 *      Start tasks.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:15pm] New.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate.
 */
void startTasks() {

    // -- Create Tasks. --
    xTaskCreate(nmeaBtLEDtask,    "NMEA_BT_LED_task",       2048, NULL, 2, &nmeaBtLEDtaskHandle);

    // -- Suspend tasks. --
    vTaskSuspend(nmeaBtLEDtaskHandle);

    // -- Print status. --
    Serial.println("LED task: NMEA BT.");
}

/**
 * ------------------------------------------------
 *      Start UI.
 * ------------------------------------------------
 *
 * Last step in setup before loop().
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 */
void startUI() {

    // -- Verify LEDs.
    Serial.print("Verify LEDs");
    updateLEDs('1');        // All LEDs on.
    delay(2000);            // Pause & show startup UI for 2 seconds.
    updateLEDs('0');        // All LEDs off.
    Serial.println(".");

    // -- Loop. --
    Serial.println("Starting loop().");

    // -- Display serial (USB) startup message. --
    Serial.printf("\nGhost Rover Bluetooth Relay - Version %c.%c.%c @ %s.\n\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION, BUILD_DATE);
}

// --- Check. ---

/**
 * ------------------------------------------------
 *      Check serial monitor (USB) for input.
 * ------------------------------------------------
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 */
void checkSerialMonitor(char print = ' ') {

    // -- Local vars. --
    static uint8_t posnMon = 0;         // Persistant input position for USB serial monitor command.
    uint8_t whichMonitorCommand;        // Which command was entered from the USB serial monitor.

    // -- Print valid commands. --
    if (print == 'p') {
        Serial.print("\nValid commands: ");
        for (size_t i = 0; i < NUM_COMMANDS-1; i++) {
            if ((i != 0) && (i % 7 == 0)) {             // List a max of (7) commands per line.
                Serial.println();
            }
            Serial.printf("%s, ", commands[i]);
        }
        Serial.printf("%s.\n! to quit.\n\n", commands[NUM_COMMANDS-1]);
        return;                         // Done for now.
    }

    // -- Read Serial monitor (USB) input. --
    while (Serial.available() > 0) {                        // Bytes available to be read.
        monitorChar = Serial.read();                        // Read a byte in.
        if (monitorChar != '\n' && (posnMon < (sizeof(monitorCommand) - 1))) {    // Are we done?
            monitorCommand[posnMon] = monitorChar;          // Not done yet, add char to command.
            posnMon++;                                      // Increment command buffer posn.
        } else {
            monitorCommand[posnMon] = '\0';                 // We're done reading, treat command[] as C-string.
            posnMon = 0;                                    // Reset command buffer position.

        // - Which command? -
        if (*monitorCommand == EXIT_TEST) {                 // Reset debug flags & return.
            debugBT = false;
            Serial.println("All debugging disabled.");
            return;
        }
        whichMonitorCommand = 99;                           // Which command was entered. Assume invalid until validated.
            for (size_t i = 0; i < NUM_COMMANDS; i++) {
            if (strcmp(monitorCommand, commands[i]) == 0) {  // Compare C-strings.
                whichMonitorCommand = i;
                break;
            }
        }

        // - Valid command? -
        if(whichMonitorCommand < 99) {   

            // Toggle command flag & print new state.
            switch (whichMonitorCommand) {
                    case 0:
                        testLEDbt = (testLEDbt == true) ? false : true;     // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (testLEDbt  ? "enabled." : "disabled."));
                        break;
                    case 1:
                        debugBT = (debugBT == true) ? false : true;         // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (debugBT ? "enabled." : "disabled."));
                        break;
                    case 2:
                        reset = (reset == true) ? false : true;             // Flip the state.
                        Serial.printf("%s %s\n", commands[whichMonitorCommand], (reset       ? "enabled." : "disabled."));
                        break;
                };
                monitorCommand[0] = '\0';                                   // Ready for the next time.
            } else {

                Serial.printf("\n%s is not a valid command. \n", monitorCommand);   // Invalid command.
                checkSerialMonitor('p');                                    // Display valid serial Monitor commands.
            }

            // Reset MCU.
            if (reset) {
                Serial.println("Restarting ...");
                esp_restart();
            }

            // Test BT LED.
            if (testLEDbt) {

                // Display instructions.
                Serial.printf("Valid options: 0(off), 1(on), 2(active). %c to quit.\n", EXIT_TEST);

                // Loop.
                while (true) {                                  // Infinite loop.
                    if (Serial.available() > 0) {
                        monitorChar = Serial.read();            // Read input from serial monitor.
                        Serial.read();                          // Discard newline.
                        switch (monitorChar) {
                            case EXIT_TEST:                     // All done.
                                Serial.println("testLEDbt disabled.");
                                testLEDbt = false;              // Clear test flag.
                                return;                         // Exit test mode.
                            case '0':                           // Radio LED - off.
                                Serial.printf("%c - BT LED off.\n", monitorChar);
                                updateLEDs('0');
                                break;
                            case '1':                           // BLE LED - on.
                                Serial.printf("%c - BT LED on.\n", monitorChar);
                                updateLEDs('1');
                                break;
                            case '2':                           // BLE LED - active.
                                Serial.printf("%c - BT LED active - 5 cycles.\n", monitorChar);
                                for (size_t i = 0; i < 5; i++) {
                                    updateLEDs('2');
                                    Serial.printf("Blink %i\n", i+1);
                                    delay(1000);
                                }
                                Serial.println();
                                break;
                            default:
                                Serial.printf("%c to quit. Valid options: 0(off), 1(on), 2(active).\n", EXIT_TEST);
                        }
                    }
                }
            }
        }
    }
}

/**
 * ------------------------------------------------
 *      Check serial0 (NMEA in) from ESP32-S3. Send out to Bluetooth (BLE).
 * ------------------------------------------------
 * 
 * -- Process each NMEA character in from serial0.
 * -- NMEA RTK-SMA config:
 *    - startAndConfigGNSS() in DougFoster_Ghost_Rover.ino setup() sets the following for ZED-F9P UART2:
 *        Output GNSS solutions periodically (as opposed to being polled).
 *        Calculate two GNSS solutions every second.
 *        NMEA protocol out.
 *        NMEA sentences suppressed (GLL, VTG).
 *        NMEA sentences out at two/second:
 *          GNGGA = PVT, fix quality, SIV, HDOP, ...
 *          GPGSV = # Sats visible, sat info, ...
 *          GNGSA = PRN # for active sat, PDOP/HDOP/VDOP, ...
 *          GNRMC = PVT, ...
 *          GNGST = Position error statistics, ...
 * -- NMEA in:
 *    - serial 0, pin NMEA_RX.
 * -- BT (BLE) out:
 *    - library: ble = BleSerial library object in startBT() in setup().
 * -- NMEA ACK out:
 *    - serial 0, pin NMEA_ACK TX.
 *
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 * @see    startBT().
 * @link   https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html.
 * @link   https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_RMC.html.
 * @link   https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSA.html.
 * @link   https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSV.html.
 * @link   https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GST.html.
 * @link   https://www.tutorialspoint.com/c_standard_library/c_function_strtok.htm.
 * @link  https://www.w3schools.com/c/ref_string_strtok.php.
 */

void checkNMEAin() {

    // -- Local vars. --
    static bool      gnssSentenceStarted    = false;        // Flag, forming a sentence.
    static char      gnssSentence[128]      = {'\0'};       // Working sentence.
    static char      thisSentence[128]      = {'\0'};       // Copy sentence.  
    static u_int8_t  LEDsentenceCount       = 1;            // Flash BT LED if count == LEDTRIGGER_LED_COUNT.
    static u_int8_t  gnssSentencePosn       = 0;            // Index position for working sentence.
    static u_int8_t  gnssSentencePosnEnd    = 0;            // Working sentence end.
    static u_int32_t sentenceCount          = 0;            // Total # of sentences sent out BT since boot.
    const  char *    delimiter              = ",";          // Delimiter for GNGGA strtok().
           char *    token;                                 // Token for GNGGA strtok().
           u_int8_t  count;                                 // Counter for GNGGA strtok() field tokens.

    // -- Read serial 0. --
    while(serial0.available() > 0) {            // Serial 0 - bytes to read?

        serial0Char = serial0.read();           // Read a character @ ZED-F9P default speed.

        // - Look for start of sentence. -
        if (serial0Char == '$') {                               // First char of a NMEA sentence is always '$'.
            gnssSentenceStarted = true;                         // Starting a new sentence.
            memset(gnssSentence, '\0', sizeof(gnssSentence));   // Clear the sentence buffer.
            gnssSentencePosn = 0;                               // First character.
            gnssSentencePosnEnd = sizeof(gnssSentence);         // Initial length. Shorten once '*' is read.
        }

        // - If forming a sentence, add the serial0Char character. -
        if (gnssSentenceStarted == true) {
            gnssSentence[gnssSentencePosn++] = serial0Char;     // Started a sentence, add the serial0Char char.

            // Is sentence too long, almost done, or done?
            if (gnssSentencePosn == sizeof(gnssSentence)) {     // Sentence too long.
                gnssSentenceStarted = false;                    // Start over.
            } else if (serial0Char == '*') {                    // Almost done. CRC indicator. Length of sentence is now known.
                gnssSentencePosnEnd = gnssSentencePosn + 2;     // Length of sentence = current position + two bytes for CRC.
            } else if (gnssSentencePosn == gnssSentencePosnEnd) {   // Done!

                // Done. Full NMEA sentence.
                gnssSentenceStarted = false;                    // Ready to start over.
                gnssSentence[gnssSentencePosn+1] = '\0';        // Terminate the sentence (C-string).
                sentenceCount++;                                // Counter for total # of sentences sent out BT since boot.

                ble.printf("%s\r\n", gnssSentence);             // Send current sentence (any Gxxxx) out BT.

                // Update rover BT LED & counts.
                if (LEDsentenceCount == NMEA_ACK_COUNT) {
                    updateLEDs('2');                            // Flash BT LED once for every NMEA_ACK_COUNT sentences sent.
                    serial0.print("ACK-BT");                    // Tell Ghost Rover ESP32-S3 that BT sends are active.
                    if (debugBT) {                              // Debug.
                        Serial.println("\n** ACK-BT sent to ESP32-S3 **\n");
                    }
                    LEDsentenceCount = 1;
                } else {
                    LEDsentenceCount++;                         // Counter for LED flash.
                }
                
                // Check debug.
                if (debugBT) {                                  // Debug.
                
                    strcpy(thisSentence, gnssSentence);         // Copy gnssSentence since strtok will change it.

                    // Tokenize $GNGGA gnssSentence to retrive & display lat, long, & alt.
                    token = strtok(gnssSentence, delimiter);    // Tokenize.

                    if (strcmp(token, "$GNGGA") == 0) {
                        Serial.println("\n----------------------------------------------------------------------------------");
                        Serial.printf("BT NMEA sentence #%i.\n%s\n", sentenceCount, thisSentence);
                        count = 0;                              // Field counter.
                        while (token != NULL) {                 // Loop gnssSentence fields.
                            switch (count) {
                                case 2:                         // Field #2 is ddmm.mmmmmmm latitude.                         
                                    latitude = convert_nmea_ddmm_to_int64(token);
                                    break;
                                case 3:                         // Field #3 is (N/S) latitude direction.
                                    if (*token == 'S') {
                                        latitude = latitude * -1;
                                    }
                                    Serial.printf("Lat    %.7f\n", double( latitude ) / 10000000.0);
                                    break;
                                case 4:                         // Field #4 is ddmm.mmmmmmm longitude.
                                    longitude = convert_nmea_ddmm_to_int64(token);
                                    break;
                                case 5:                         // Field #5 is (E/W) longitude direction.
                                    if (*token == 'W') {
                                        longitude = longitude * -1;
                                    }
                                    Serial.printf("Long  %.7f\n", double( longitude ) / 10000000.0);
                                    break;
                                case 9:                         // Field #9 is M altitude (orthometric height - MSL).
                                    if (count == 9) {
                                        altitude = (int64_t)(((double)atof(token)) * 1000.0);
                                    }
                                    Serial.printf("Alt    %.2f *\n", double( altitude ) / 1000.0);
                                    Serial.printf("* height (m) above WGS84 ellipsoid (HAE), not orthometric height above sea level (MSL).\n");
                                    break;
                                }
                            count++;                            // Next field.
                            token = strtok(NULL, delimiter);    // Next token.
                        }
                        Serial.println("----------------------------------------------------------------------------------\n");
                    } else {
                        Serial.printf("BT NMEA sentence #%i.\n%s\n", sentenceCount, thisSentence);
                    }
                }

                memset(gnssSentence, '\0', sizeof(gnssSentence));   // Null out the working sentence.
            }
        }
    }
}

// --- Display. ---

/**
 * ------------------------------------------------
 *      Toggle LED.
 * ------------------------------------------------
 *
 * @param  char ledR Radio LED.
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/07-vTaskResume.
 * @link   https://forum.seeedstudio.com/t/arduino-blink-led-builtin-inverted-output/275405.
 */
void updateLEDs(char ledBt) {

    // --- Radio LED. ---
    switch (ledBt) {
        case '0':
            digitalWrite(LED_BUILTIN, HIGH);        // LED off (reversed for XIAO ESP32C6).
            break;
        case '1':
            digitalWrite(LED_BUILTIN, LOW);         // LED on (reversed for XIAO ESP32C6).
            break;
        case '2':
            vTaskResume(nmeaBtLEDtaskHandle);       // Resume task.
            break;
    }
}

// --- Callback. ---

// --- Operation. ---

/**
 * Utility to convert ddmm.mmmmmmm to int_64.
 *
 * @param  const char *nmea_token Position token from GNGGA sentence. 
 * @return int64_t number.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 * @see    checkNMEAin().
 */
int64_t convert_nmea_ddmm_to_int64(const char *nmea_token) {

    // -- Convert token to float. --
    double raw = atof(nmea_token);

    // -- Separate degrees and minutes. --
    int degrees = (int)(raw / 100);  // Get dd.
    double minutes = raw - (degrees * 100);  // Get mm.mmmmmmm.

    // -- Convert to decimal degrees. --
    double decimal_degrees = degrees + (minutes / 60.0);

    // -- Scale to preserve 7 decimal places and convert to int64_t. --
    return (int64_t)round(decimal_degrees * 10000000.0);
}

// --- Tasks. ---

/**
 * ------------------------------------------------
 *      BT LED - active task.
 * ------------------------------------------------
 *
 * @param  void * pvParameters Pointer to task parameters.
 * @return void No output is returned.
 * @since  0.1.0 [2025-07-09-06:45pm] New.
 * @see    startTasks().
 * @link   https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html.
 * @link   https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/06-vTaskSuspend.
 * @link   https://forum.seeedstudio.com/t/arduino-blink-led-builtin-inverted-output/275405.
 * 
 */
void nmeaBtLEDtask(void * pvParameters) {
    while(true) {
        digitalWrite(LED_BUILTIN, LOW);                     // LED on (reversed for SEEED XIAO ESP32C6).
        vTaskDelay(LED_TIME_FLASH_ON);                      // LED remains on (ms).
        digitalWrite(LED_BUILTIN, HIGH);                    // LED off (reversed for SEEED XIAO ESP32C6).
        vTaskSuspend(NULL);                                 // Suspend task.
    }
}

// --- Test. ---

// ===================================
//               Setup.
// ===================================

void setup() {
    startBT();                      // Begin & start Bluetooth interface.  // TODO: #1.
    beginSerialMonitor();           // Begin serial monitor (USB).
    initVars();                     // Initialize global vars.
    configPins();                   // Initialize pin modes & pin values.
    beginSerialInterfaces();        // Begin serial interfaces. 
    startTasks();                   // Start tasks.
    startUI();                      // Start UI.
}                                               

// =================================== 
//               Loop.
// ===================================

void loop() {
    checkSerialMonitor();           // Check serial monitor (USB) for input.
    checkNMEAin();                  // Check serial0 (NMEA in) from ESP32-S3. Send out to Bluetooth (BLE).
}
