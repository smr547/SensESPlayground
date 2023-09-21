// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <SparkFunBME280.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/typecast.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

reactesp::ReactESP app;

BME280 myBME280;
unsigned int readTempIntervalMs = 2000;
unsigned int readRainIntervalMs = 60 * 5 * 1000;  // report rain every 5 minutes
const uint8_t rain_pin = 35;
const uint8_t wind_speed_pin = 27;

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

Wire.begin();

//Initialize BME280
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  byte id = myBME280.begin(); //Returns ID of 0x60 if successful

  Serial.print("BME280.begin() returned ");
  Serial.println(id, HEX);

  if (id != 0x60){
    Serial.println("Problem with BME280");
  } else {
    Serial.println("BME280 online");
  }



  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("my-sensesp-project")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // GPIO number to use for the analog input
  const uint8_t kAnalogInputPin = 36;
  // Define how often (in milliseconds) new samples are acquired
  const unsigned int kAnalogInputReadInterval = 500;
  // Define the produced value at the maximum input voltage (3.3V).
  // A value of 3.3 gives output equal to the input voltage.
  const float kAnalogInputScale = 3.3;

  // Create a new Analog Input Sensor that reads an analog input pin
  // periodically.
  auto* analog_input = new AnalogInput(
      kAnalogInputPin, kAnalogInputReadInterval, "", kAnalogInputScale);

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  analog_input->attach([analog_input]() {
    debugD("Analog input value: %f", analog_input->get());
  });

  // Set GPIO pin 15 to output and toggle it every 650 ms

  const uint8_t kDigitalOutputPin = 15;
  const unsigned int kDigitalOutputInterval = 650;
  pinMode(kDigitalOutputPin, OUTPUT);
  app.onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  // Read GPIO 14 every time it changes

  const uint8_t kDigitalInput1Pin = 14;
  auto* digital_input1 =
      new DigitalInputChange(kDigitalInput1Pin, INPUT_PULLUP, CHANGE);

  // Connect the digital input to a lambda consumer that prints out the
  // value every time it changes.

  // Test this yourself by connecting pin 15 to pin 14 with a jumper wire and
  // see if the value changes!

  digital_input1->connect_to(new LambdaConsumer<bool>(
      [](bool input) { debugD("Digital input value changed: %d", input); }));

  // Create another digital input, this time with RepeatSensor. This approach
  // can be used to connect external sensor library to SensESP!

  const uint8_t kDigitalInput2Pin = 13;
  const unsigned int kDigitalInput2Interval = 1000;

  // Configure the pin. Replace this with your custom library initialization
  // code!
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  // Define a new RepeatSensor that reads the pin every 100 ms.
  // Replace the lambda function internals with the input routine of your custom
  // library.

  // Again, test this yourself by connecting pin 15 to pin 13 with a jumper
  // wire and see if the value changes!

  auto* digital_input2 = new RepeatSensor<bool>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin]() { return digitalRead(kDigitalInput2Pin); });

  // C onnect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  analog_input->connect_to(new SKOutputFloat(
      "sensors.analog_input.voltage",         // Signal K path
      "/sensors/analog_input/voltage",        // configuration path, used in the
                                              // web UI and for storing the
                                              // configuration
      new SKMetadata("V",                     // Define output units
                     "Analog input voltage")  // Value description
      ));

  // Connect digital input 2 to Signal K output.
  digital_input2->connect_to(new SKOutputBool(
      "sensors.digital_input2.value",          // Signal K path
      "/sensors/digital_input2/value",         // configuration path
      new SKMetadata("",                       // No units for boolean values
                     "Digital input 2 value")  // Value description
      ));

  /*****************************************************
   * TEMPERATURE                                       *
   *****************************************************/
  auto* study_room_temp = new RepeatSensor<float>(readTempIntervalMs, []() {
    return (myBME280.readTempC() + 273.15);
  });
  
  const char * sk_temp_path = "study.temperature";
  SKMetadata* metadata = new SKMetadata();
  metadata->description_ = "Study Temperature";
  metadata->display_name_ = "Study Temperature";
  metadata->short_name_ = "Study Temp";
  metadata->units_ = "K";
  study_room_temp->connect_to(new Linear(1.0,0.0, "/study/temperature/calibrate"))
  ->connect_to(new SKOutputFloat(sk_temp_path, metadata));


  /*****************************************************
   * HUMIDITY                                          *
   *****************************************************/
  auto* study_humidity = new RepeatSensor<float>(readTempIntervalMs, []() {
    return (myBME280.readFloatHumidity()/100.0);
  });
  
  const char * sk_humidity_path = "study.humidity";
  SKMetadata* metadata_humidity = new SKMetadata();
  metadata_humidity->description_ = "Study Humidity";
  metadata_humidity->display_name_ = "Study Humidity";
  metadata_humidity->short_name_ = "Study Humid";
  metadata_humidity->units_ = "ratio";
  study_humidity->connect_to(new Linear(1.0,0.0, "/study/humdity/calibrate"))
  ->connect_to(new SKOutputFloat(sk_humidity_path, metadata_humidity));



  /*****************************************************
   * PRESSURE                                          *
   *****************************************************/
  auto* study_pressure = new RepeatSensor<float>(readTempIntervalMs, []() {
    return (myBME280.readFloatPressure());
  });
  
  const char * sk_pressure_path = "study.pressure";
  SKMetadata* metadata_pressure = new SKMetadata();
  metadata_pressure->description_ = "Study Pressure";
  metadata_pressure->display_name_ = "Study Pressure";
  metadata_pressure->short_name_ = "Study Pres";
  metadata_pressure->units_ = "Pa";
  study_pressure->connect_to(new Linear(1.0,0.0, "/study/pressure/calibrate"))
  ->connect_to(new SKOutputFloat(sk_pressure_path, metadata_pressure));


  /*****************************************************
   * RAIN                                              *
   *****************************************************/

  
    const unsigned int ignore_interval_ms = 200;  // switch is kinda noisy
    const float multiplier = 0.18;                // mm per count

    // There's no path in the Signal K spec for rain, so let's make one.

  const char * sk_rain_path = "study.rain.last5mins";
  SKMetadata* metadata_rain = new SKMetadata();
  metadata_rain->description_ = "Study Rain last 5 mins";
  metadata_rain->display_name_ = "Study Rain 5 mins";
  metadata_rain->short_name_ = "Study Rain";
  metadata_rain->units_ = "mm";
  
  auto *study_rain = new DigitalInputDebounceCounter(
        rain_pin, INPUT_PULLUP, FALLING, readRainIntervalMs, ignore_interval_ms);

  study_rain->connect_to(new Typecast<int, float>())
        ->connect_to(new Linear(multiplier, 0.0, "/study/rain/calibrate"))
        ->connect_to(
            new SKOutputFloat(sk_rain_path, metadata_rain));


  /*****************************************************
   * Wind speed                                        *
   *****************************************************/
    
    uint wind_read_interval_ms = 3 * 1000 /* read every 3s */;
    uint wind_ignore_interval_ms = 5 /* 200 counts/s, or 205m/s of wind */;

  SKMetadata* metadata_windspeed = new SKMetadata();
  metadata_windspeed->description_ = "Study windspeed";
  metadata_windspeed->display_name_ = "Study windspeed";
  metadata_windspeed->short_name_ = "Study windspeed";
  metadata_windspeed->units_ = "m/s";
    auto *wind_sensor = new DigitalInputDebounceCounter(
        wind_speed_pin, INPUT_PULLUP, FALLING, wind_read_interval_ms, wind_ignore_interval_ms);
    wind_sensor->connect_to(new Frequency(1.026, "/study/windspeed/calibrate"))
        ->connect_to(new SKOutputFloat("study.wind.speedApparent", metadata_windspeed));
  


  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
