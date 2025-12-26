#include <DFRobot_MAX30102.h>
#include "LSM6DS3.h"

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

DFRobot_MAX30102 particleSensor;
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery



typedef struct {
    uint8_t start_frame[2];  // 开头帧 0x5A, 0xA5
    int32_t red_light_data;  // 红光数据
    int32_t ir_light_data;   // 红外光数据
    int16_t sensor_data[6];  // 6轴传感器数据，每个16位
    int32_t counter;         // 计数位
    uint8_t end_frame[2];    // 结束帧 0xEE, 0xFF
} DataFrame;

void setup()
{ 
  // low charge rate 50 mA
  pinMode (13, OUTPUT);
  digitalWrite(13, HIGH);
  //Init serial 
  Serial.begin(115200);
  while (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }

  /*!
   *@brief Use macro definition to configure sensor
   *@param ledBrightness LED brightness, default value: 0x1F（6.4mA), Range: 0~255（0=Off, 255=50mA）
   *@param sampleAverage Average multiple samples then draw once, reduce data throughput, default 4 samples average
   *@param ledMode LED mode, default to use red light and IR at the same time 
   *@param sampleRate Sampling rate, default 400 samples every second 
   *@param pulseWidth Pulse width: the longer the pulse width, the wider the detection range. Default to be Max range
   *@param adcRange Measurement Range, default 4096 (nA), 15.63(pA) per LSB
   */
  particleSensor.sensorConfiguration(/*ledBrightness=*/0x1F, /*sampleAverage=*/SAMPLEAVG_4, \
                                  /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_400, \
                                  /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_4096);
  // IMU init
  if (myIMU.begin() != 0) {
	Serial.println("Device error");
	} else {
		Serial.println("Device OK!");
	}
	
  // BLE init
  
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin() 
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);
  // Set up and start advertising
  startAdv();
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
}

int32_t red_light;
int32_t ir_light;
int16_t IMU_data[6] = {0, 0, 0, 0, 0, 0};
int32_t counter = 0;
DataFrame frame;
unsigned long startTime; // 记录起始时间
unsigned long endTime;   // 记录结束时间
unsigned long elapsedTime; // 记录时间差
void loop()
{ 
  // startTime = millis(); // 记录当前时间
  counter++;
  red_light = particleSensor.getRed();
  ir_light = particleSensor.getIR();
  IMU_data[0] = myIMU.readRawAccelX();
  IMU_data[1] = myIMU.readRawAccelY();
  IMU_data[2] = myIMU.readRawAccelZ();
  IMU_data[3] = myIMU.readRawGyroX();
  IMU_data[4] = myIMU.readRawGyroY();
  IMU_data[5] = myIMU.readRawGyroZ();
  initializeDataFrame(&frame, red_light, ir_light, IMU_data, counter);
  sendFrame(&frame);
  // endTime = millis(); // 记录当前时间
  // elapsedTime = endTime - startTime; // 计算时间差
  // Serial.print("Elapsed time: ");
  // Serial.print(elapsedTime);
  // Serial.println(" ms");
  Serial.print("RED: ");
  Serial.print(red_light);
  Serial.print(" ");
  Serial.println("IR: ");
  Serial.print(ir_light);
  Serial.println(" ");
}

void sendFrame(DataFrame *frame) {
    uint8_t *byte_ptr = (uint8_t *)frame;
    size_t frame_size = sizeof(DataFrame);
    for (size_t i = 0; i < frame_size; i++) {
        bleuart.write(byte_ptr[i]);
    }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}


void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

// 初始化和填充数据帧
void initializeDataFrame(DataFrame *frame, int32_t red_light, int32_t ir_light, int16_t sensor[], int32_t count) {
    frame->start_frame[0] = 0x5A;
    frame->start_frame[1] = 0xA5;
    frame->red_light_data = red_light;
    frame->ir_light_data = ir_light;
    memcpy(frame->sensor_data, sensor, sizeof(frame->sensor_data));
    frame->counter = count;
    frame->end_frame[0] = 0xEE;
    frame->end_frame[1] = 0xFF;
}
