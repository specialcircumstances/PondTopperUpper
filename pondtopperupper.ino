//#include "SparkIntervalTimer.h"
#include "photon-wdgs.h"
#include "onewire.h"
#include "ds18x20.h"
#include "HttpClient.h"
#include "math.h"


// Ultrasonics based on https://github.com/Hypnopompia/photon-desk/blob/master/firmware/photon-desk.ino


//
SYSTEM_THREAD(ENABLED);


// DEBUG STUFF
bool debug_serial = false;

// These defaults are reset according to SystemID in setup()
int flask_port = 5001;    //5000 for live, 5001 for beta/testing
String flask_path = "/pond/test"; //


// The following all in ms

int interval_flask = 20000;          // How long after we wake up that we get measurements
int interval_cloud = 360000;         // How often we try to log to Particle Cloud.
int interval_connection_check = 5000;// How often we check the WiFi is OK.
int interval_sense_slow = interval_flask - 5000;           // How often we take measurements, period after last successful
int interval_sense_fast = 8000;           // How often we take measurements, period after last successful
int interval_sense = interval_sense_slow;
int interval_sense_read = 1000;      // How long after (temp) measurement request we wait before asking for result
int interval_sleep = 300 - (interval_flask / 1000); // How long we sleep for, periodicity
int interval_watchdog = interval_sleep * 2;

// Setup various constants and variables
const uint8_t how_many_sensors = 3;   // Number of temp sensors that we expect
//const uint8_t how_many_sensors = 1;   // Number of temp sensors that we expect
char tablename[32] = "unknown_device";       // Name of the table we are logging to, note that the flask code needs to be configured with the same name
                                             // is reset in setup according to HW ID
char myversion[32] = "unknown_version";      // String of the firmware version we are using


// Setup various variables
uint8_t sensors[80];            // Number of sensors detected
uint8_t max_temp_tick = 60;     // How often to reset max temp
uint8_t startup_tick = 0;       // To track startup, to help build realistic average from start

char postdata[1024];             // For posting to DB

// THis section from flow return, replaced
float water_temp_prev = 0;            // Previous temp (primary)
float air_temp_prev = 0;            // Previous temp (primary)
float system_temp_prev = 0;            // Previous temp (primary)

float water_temp = 0;              // Room temperature
float air_temp = 0;              // Room temperature
float system_temp = 0;              // Room temperature

float water_temp_ave = 0;          // Running average
float air_temp_ave = 0;          // Running average
float system_temp_ave = 0;          // Running average

float water_temp_max = 0;           // Max temperature, not really used
float air_temp_max = 0;             // Max temperature, not really used
float system_temp_max = 0;          // Max temperature, not really used

float water_temp_min = 0;           // min temperature, not really used
float air_temp_min = 0;             // min temperature, not really used
float system_temp_min = 0;          // min temperature, not really used

// Water Level variables and settings
int water_level = 0;                // Water level below sensor, in mm
int water_level_low_trig = 470;     // Water Level LOW trigger (start filling), distance from sensor in mm
int water_level_high_trig = 460;    // Water Level HIGH trigger (stop filling), distance from sensor in mm
int min_fill_battery_mv = 11740;    // Don't attempt to operate the valve if Battery level is below this.
int valve_bad_read_count = 0;       // Used to track bad readings.
int valve_bad_read_max = 6;        // x bad readings in a row  will force valve shut.

int water_level_stdev = 0;              // Water level below sensor, in mm
int water_level_stdev_required = 100;   // Max std deviation allowed in water level readings.
int valve_status = 0;                 //0 for closed, 1 for open
int initial_mem_usage = 0;
float minimum_operating_temp = 5;    // below this temp do not try to operate the valve
int minimum_operating_battery = 25;  // below this battery percentage we don't try to operate the valve

int battery_mV = 0;
int battery_percent = 0;
const int battery_empty_mV = 11500;
const int battery_full_mV = 12900;
const float batt_R1 = 1032;   // Voltage divider, upper R in ohms
const float batt_R2 = 227;   // Voltage diveder, lower (measured) R in ohms
const float batt_adc_steps = 0.00080898;  // V per steps in ADC with 3.3V at 3.31V - tuned
const float batt_multiplier = 1000.0 * batt_adc_steps / ( batt_R2 / ( batt_R1 + batt_R2 ) ); // Should result in mV of battery

int solar_mV = 0;
const float solar_R1 = 1156;   // Voltage divider, upper R in ohms
const float solar_R2 = 113;   // Voltage divider, lower (measured) R in ohms
const float solar_adc_steps = 0.000809;  // V per steps in ADC with 3.3V at 3.31V - tweaked a little
const float solar_multiplier = 1000.0 * solar_adc_steps / ( solar_R2 / ( solar_R1 + solar_R2 ) ); // Should result in mV of battery

// All size 12
char batt_status_charging_fast[]    = "CHARGE_FAST";  // over 14.4V
char batt_status_charging_normal[]  = "CHARGE_NORM"; // over 13.8V
char batt_status_charging_trickle[] = "CHARGE_TRIC"; // over 13.5V
char batt_status_charging_min[]     = "CHARGE__MIN";  // over 12.9V
char batt_status_just_charged[]     = "JUSTCHARGED";
char batt_status_full[]             = "FULL___100%";
char batt_status_high[]             = "HIGH___>70%";
char batt_status_medium[]           = "MEDIUM_>40%";
char batt_status_low[]              = "LOW____>10%";
char batt_status_discharged[]       = "DISCHARGED!";  // Um... Dead...
char current_battery_status[12];


bool measuring_flag = false;    // Measurement in progress
bool good_measurement = false;  // Last measurement valid
bool enable_valve = true;      // Stops actual operation of the valve (mainly for testing)

//uint8_t g_ave_values = 30;    // If we sample every 10 seconds then this is a 5 minute average.
uint8_t g_ave_values = 90;      // If we sample every 10 seconds then this is a 15 minute average.

int usonic_echoPin_01 = D1;
int usonic_trigPin_01 = D2;
int usonic_echoPin_02 = D3;
int usonic_trigPin_02 = D4;
int sleep_wakeupPin = D5;
int valve_controlPin = D6;
int usonic_enable = D7;
int batt_mon_pin = A0;
int solar_mon_pin = A1;

bool sleeping = false;

unsigned long last_measurement = 0;     // Timing variable - Last time we completed processing a valid measurement
unsigned long last_cloud = 0;           // Timing variable - Last time we logged to the cloud
unsigned long last_flask = 0;           // Timing variable - Last time we logged to the DB via flask server
unsigned long measurement_request = 0;  // Timing variable - Last time we requested a new measurement
unsigned long reconnect_count = 0;
unsigned long last_connection_check = 0;
unsigned long last_cloud_connection_check = 0;
unsigned long last_idle_blip = 0;


//RestClient client = RestClient("flask.home",5000);  Moved inside main loop, so it's created and destroyed - hope might help stability.


void log(char* msg)
{
  Particle.publish("log", msg);
  delay(500);
}


void setup()
{
  String myID = System.deviceID();
  // 28003c000a47343232363230 pond_topper_upper_dev
  // 41002f000447343233323032 pond_topper_upper_prod
  if (myID == "41002f000447343233323032") // pond_topper_upper_prod
  {
    String tempstr = "pond";
    tempstr.toCharArray(tablename, 32);
    debug_serial = false;
    flask_port = 5000;    //5000 for live, 5001 for beta/testing
    flask_path = "/pond/PondLog";
    if (debug_serial) {  Serial.println("I am Pond Topper Upper"); }
  }

  if (debug_serial) {
    Serial.begin(9600);
    delay(250);
    Serial.println("Starting.");
  }
  ow_setPin(D0);
  usonic_setup(1);
  usonic_setup(2);
  pinMode(sleep_wakeupPin, INPUT_PULLDOWN);  // wake up trigger pin  - NC
  pinMode(valve_controlPin, OUTPUT);
  pinMode(usonic_enable, OUTPUT);
  air_temp_min = 100;
  air_temp_max = 0;
  water_temp_min = 100;
  water_temp_max = 0;
  system_temp_min = 100;
  system_temp_max = 0;


  startup_tick = 0;

  air_temp_ave = 0;
  water_temp_ave = 0;
  system_temp_ave = 0;

  air_temp_prev = 0;
  water_temp_prev = 0;
  system_temp_prev = 0;

  last_measurement = millis();
  last_flask = millis();
  last_cloud = millis();
  last_connection_check = millis();
  last_cloud_connection_check = millis();
  last_idle_blip = millis();
  System.version().toCharArray(myversion,32);
  // This relies on us running the System Thread.
  if (waitFor(Particle.connected, 10000)) {
    if (debug_serial) {  Serial.println("I am Connected to Particle Cloud (boot time)."); }
  }
  else
  {
    if (debug_serial) {  Serial.println("Timed out connecting to Particle Cloud at boot...."); }
  }
  if (debug_serial)
  {
    Serial.print("My IP Address is: ");
    Serial.println(WiFi.localIP());
    Serial.print("My deviceID is: ");
    Serial.println(myID);
  }
  //RGB.control(true);
  initial_mem_usage = System.freeMemory();
  if (debug_serial)
  {
    Serial.print("Finishing setup. Initial Memory is:");
    Serial.println(initial_mem_usage);
  }
  // Enable ultrasonic power (will sleep later)
  //delay(100);
  //digitalWrite(usonic_enable, HIGH);
  //delay(100);
  //PhotonWdgs::begin(true,true,interval_watchdog,TIMER7);  //https://github.com/raphitheking/photon-wdgs/blob/master/firmware/examples/photon-wdgs-demo.cpp
}


void usonic_setup(int sensorid) {
  if (sensorid == 1)
  {
	   pinMode(usonic_echoPin_01, INPUT); // Is 5V so must NOT be PULLUP
	   pinMode(usonic_trigPin_01, OUTPUT);
   }
   else if  (sensorid == 2)
   {
     pinMode(usonic_echoPin_02, INPUT); // Is 5V so must NOT be PULLUP
	   pinMode(usonic_trigPin_02, OUTPUT);
   }
}


unsigned long usonic_ping(int sensorid) {
	unsigned long duration;
  if (sensorid == 1)
  {
     // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
     ATOMIC_BLOCK() {
       digitalWriteFast(usonic_trigPin_01, HIGH);
       delayMicroseconds(25);
       digitalWriteFast(usonic_trigPin_01, LOW);
       delayMicroseconds(25);
     }
     duration = pulseIn(usonic_echoPin_01, HIGH); // Time in microseconds to recieve a ping back on the echo pin
     //if (debug_serial) {  Serial.print(duration); }
     return duration;
   }
   else if  (sensorid == 2) // Not currently connected
   {
     // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
     ATOMIC_BLOCK() {
       digitalWriteFast(usonic_trigPin_02, HIGH);
       delayMicroseconds(10);
       digitalWriteFast(usonic_trigPin_02, LOW);
     }
     duration = pulseIn(usonic_echoPin_02, HIGH); // Time in microseconds to recieve a ping back on the echo pin

     return duration;
   }
   else if  (sensorid == 3) // A test ID
   {
     return 3333;
   }
   else // Some error
   {
      return 0;
   }
}

// Struct for mping responses
//struct mping_response { unsigned long min, max, average, variance, std_dev; };
// Alternatively, let's just try an array of unsigned long instead

//struct mping_response usonic_mping(int sensorid) {
void usonic_mping(int sensorid, unsigned long retarray[5])
{
  // Runs multiple pings (ultrasonic) and trys to remove errors
  int number_of_pings = 20;
  int interping_delay = 50; // delay between pings in milliseconds
                                // 20 pings @ 50ms each - 1 second sample
  unsigned long results[number_of_pings];
  unsigned long total = 0;
  unsigned long total_sq = 0;
  if (debug_serial) {  Serial.print("I"); }
  unsigned long initial = usonic_ping(sensorid);
  unsigned long max = initial;
  unsigned long min = initial;
  if (debug_serial) {  Serial.print("."); }
  delay(interping_delay);

  for (int i=0; i<number_of_pings; i++) {
    if (debug_serial) {  Serial.print("*"); }
    results[i]=usonic_ping(sensorid);
    total += results[i];
    total_sq += results[i] * results[i];
    if (results[i] > max) { max = results[i]; }
    if (results[i] < min) { min = results[i]; }
    //PhotonWdgs::tickle();
    delay(interping_delay);
  }

  if (debug_serial) {  Serial.println("."); }
  unsigned long average = total / number_of_pings;
  unsigned long variance = ( total_sq / number_of_pings ) - ( average * average );
  unsigned long std_dev = sqrt(variance);
  if (std_dev > water_level_stdev_required) {
    // Possibly a couple of bad values so let's try and remove them
    total = 0;
    total_sq = 0;
    initial = usonic_ping(sensorid);
    min = initial;
    max = initial;
    delay(interping_delay * 2);
    for (int i=0; i<number_of_pings; i++) {
      if (debug_serial) {  Serial.print("C"); }
      if (abs((water_level * 58) - results[i]) > 290) {  // Over 5 cm from previous reading
        results[i]=usonic_ping(sensorid);     // Get new reading
        if ((results[i] < 1450) || (results[i] > 17400)) {
          //PhotonWdgs::tickle();
          delay(interping_delay * 2);
          results[i]=usonic_ping(sensorid);   // Last chance...
        }
        //PhotonWdgs::tickle();
        delay(interping_delay * 2);
      }
      total += results[i];
      total_sq += results[i] * results[i];
      if (results[i] > max) { max = results[i]; }
      if (results[i] < min) { min = results[i]; }
    }
    if (debug_serial) {  Serial.println("."); }
    average = total / number_of_pings;
    variance = ( total_sq / number_of_pings ) - ( average * average );
    std_dev = sqrt(variance);
  }
  if (debug_serial)
  {
    Serial.print("usonic_mping average is: ");
    Serial.print(average);
    Serial.print(", min: ");
    Serial.print(min);
    Serial.print(", max: ");
    Serial.print(max);
    Serial.print(", variance: ");
    Serial.print(variance);
    Serial.print(", std_dev: ");
    Serial.println(std_dev);
  }
  //struct mping_response result_stuct = { min, max, average, variance, std_dev };
  //unsigned long min, max, average, variance, std_dev;
  retarray[0] = min;
  retarray[1] = max;
  retarray[2] = average;
  retarray[3] = variance;
  retarray[4] = std_dev;
  // return nothing
}


unsigned long microsecondsToCentimeters(unsigned long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


unsigned long microsecondsToMillimeters(unsigned long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  // So to work out mm, we use ms * 10 / 58
  return (microseconds * 10) / 58;
}


float float_average(float new_val, float cur_ave, uint8_t values)
{
  if ((startup_tick < values) && (startup_tick > 0))
  {
    return ( (cur_ave * (startup_tick - 1)) + (new_val * 1) ) / startup_tick;
  }
  else
  {
    return ( (cur_ave * (values - 1)) + (new_val * 1) ) / values;
  }
}

void start_measurement()
{
  if (debug_serial) {  Serial.println("\r\n----------------New Cycle------------------\r\nRequesting Measurement..."); }
  DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); //Asks all DS18x20 devices to start temperature measurement, takes up to 750ms at max resolution
}


int get_numsensors()
// Note this can loop forever if there aren't the right number of sensors!
// TODO - do something about that!
{
  int numsensors = 0;
  while (numsensors != how_many_sensors)
  {
    numsensors = ow_search_sensors(10, sensors);
    if (numsensors != how_many_sensors)
    {
      if (debug_serial)
      {
        Serial.printlnf("Found %i sensors but I want %i - sulking now :(", numsensors, how_many_sensors);
        //log(msg);
      }
      delay(100);
    }
  }
  if (debug_serial) {
    Serial.print("Got numsensors, it is: ");
    Serial.println(numsensors);
  }
  return numsensors;
}


bool do_measurement(int numsensors, float my_results_f[])
{
  if (debug_serial) {  Serial.print("\r\nGetting measurement results:-\r\n"); }
  uint8_t subzero, cel, cel_frac_bits;
  int my_results_dc [ numsensors ];
  bool had_error = false;
  for (uint8_t i=0; i<numsensors; i++)
  {
    delay(5);
    if (sensors[i*OW_ROMCODE_SIZE+0] == 0x10 || sensors[i*OW_ROMCODE_SIZE+0] == 0x28) //0x10=DS18S20, 0x28=DS18B20
    {
      //log("Found a DS18B20");
      if ( DS18X20_read_meas( &sensors[i*OW_ROMCODE_SIZE], &subzero, &cel, &cel_frac_bits) == DS18X20_OK ) {
        char sign = (subzero) ? '-' : '+';
        int frac = cel_frac_bits*DS18X20_FRACCONV;
        my_results_dc[i] = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
        my_results_f[i] = float(my_results_dc[i]) / 10;
        if (debug_serial) {
          char mymsg[64] = "";
          sprintf(mymsg, "Got reading %.1f", my_results_f[i]);
          Serial.println(mymsg);
        } else {
          delay(5);
        }
      }
      else
      {
        //Particle.publish("log", "CRC Error (lost connection?)");
        if (debug_serial) {  Serial.println("Error getting measurement."); }
        had_error = true;
      }
    }
  }
  //Particle.publish("log", "Measurement");
  return had_error;
}


float track_max(float this_temp, float this_max_temp)
{
  if (debug_serial) {  Serial.print("Called track_max..."); }

  // used a decaying average for CH but for pond may just invoke resets
  if (this_temp >= this_max_temp)
  {
    this_max_temp = this_temp;
  }
  if (debug_serial) {  Serial.println("..finished track_max"); }
  return this_max_temp;
}

float track_min(float this_temp, float this_min_temp)
{
  if (debug_serial) {  Serial.print("Called track_min..."); }

  // used a decaying average for CH but for pond may just invoke resets
  if (this_temp <= this_min_temp)
  {
    this_min_temp = this_temp;
  }
  if (debug_serial) {  Serial.println("..finished track_min"); }
  return this_min_temp;
}

int get_battery_percentage(int mymV)
{
  if (mymV > battery_full_mV) {
    return 100;
  } else if (mymV < battery_empty_mV) {
    return 0;
  } else {
    return (100 * (mymV - battery_empty_mV)) / (battery_full_mV - battery_empty_mV);
  }
}

int check_battery()
{
  // Returns battery voltage in mV
  int val = analogRead(batt_mon_pin);  // read the analogPin
  float mymV = val * batt_multiplier;
  if (debug_serial) {
    Serial.print("Battery Monitor: raw ADC reading is ");
    Serial.println(val);
    Serial.print("Battery Monitor: batt_multiplier is  ");
    Serial.println(batt_multiplier);
    Serial.print("Battery Monitor: Voltage in mv is  ");
    Serial.println(mymV);
    Serial.print("Battery Monitor: SOC Percentage is  ");
    Serial.println(get_battery_percentage(mymV));
  }
  return int(mymV);
}



int check_solar()
{
  // This does not work at the moment for hardware reasons.
  // Returns battery voltage in mV
  int val = analogRead(solar_mon_pin);  // read the analogPin
  float mymV = val * solar_multiplier;
  if (debug_serial) {
    Serial.print("Solar Monitor: raw ADC reading is ");
    Serial.println(val);
    Serial.print("Solar Monitor: solar_multiplier is  ");
    Serial.println(solar_multiplier);
    Serial.print("Solar Monitor: Voltage in mv is  ");
    Serial.println(mymV);
  }
  return int(mymV);
}


void battery_status_text(int batt_mV, char status_text[12])
// Note, changes contents of passed char array
{
  if (batt_mV > 14400) { strncpy(status_text, batt_status_charging_fast, 12);}
  else if (batt_mV > 13800) { strncpy(status_text, batt_status_charging_normal, 12);}
  else if (batt_mV > 13500) { strncpy(status_text, batt_status_charging_trickle, 12);}
  else if (batt_mV > 13000) { strncpy(status_text, batt_status_charging_min, 12);}
  else if (batt_mV > 12900) { strncpy(status_text, batt_status_just_charged, 12);} // above 100%
  else if (batt_mV > 12760) { strncpy(status_text, batt_status_full, 12);}  // above 90%
  else if (batt_mV > 12480) { strncpy(status_text, batt_status_high, 12);}  // above 70%
  else if (batt_mV > 12060) { strncpy(status_text, batt_status_medium, 12);}  // above 40%
  else if (batt_mV > 11640) { strncpy(status_text, batt_status_low, 12);} // above 10%
  else { status_text = strncpy(status_text, batt_status_discharged, 12);}
}

void operate_valve()
// Remember that we are dealing with distance from sensor, not absolute depth of water.
{
  if ( water_level_stdev < water_level_stdev_required )
  // We are confident in our measurement
  {
    // Reset bad reading count
    valve_bad_read_count = 0;
    if ( (water_level > water_level_low_trig) && (battery_mV > min_fill_battery_mv) && (air_temp > minimum_operating_temp) && enable_valve)
    {
      // Need to fill the pond
      digitalWrite(valve_controlPin, HIGH);
      valve_status = 1;
      interval_sense = interval_sense_fast;
    }
    else if (water_level < water_level_high_trig) {
      // Time to stop filling the pond.
      digitalWrite(valve_controlPin, LOW);
      valve_status = 0;
      interval_sense = interval_sense_slow;
    }
  } else {
  // Could also end up here when battery_mV drops too low I think TODO
  // Bad water level readings
    if (valve_bad_read_count > valve_bad_read_max) {
      // Could be a problem so best to ensure the valve is closed
      digitalWrite(valve_controlPin, LOW);
      valve_status = 0;
      // Leave the interval sense fast until we are sure of result.
    } else {
      valve_bad_read_count++;
    }
  }
}

void loop()
{
  //dogcount = false;   // Reset Watchdog
  //PhotonWdgs::tickle();
  if ( ( (millis() - last_measurement) > interval_sense) && not(measuring_flag) ) // If it's time, ask for a measurement
  {
    //digitalWrite(valve_controlPin, HIGH);
    start_measurement();
    measuring_flag = true;
    measurement_request = millis();
    if (debug_serial) {  Serial.println("Finished asking for measurement"); }
  }
  if ( ( (millis() - measurement_request) > interval_sense_read) && measuring_flag ) // 1 second after the measurement check for the result
  {
    if (debug_serial) {  Serial.println("Checking for result from measurement."); }
    //digitalWrite(valve_controlPin, LOW);
    int numsensors = get_numsensors();
    float my_results_f[numsensors];
    bool had_error = do_measurement(numsensors, my_results_f);
    delay(10);
    if (not(had_error))
    {
      if (debug_serial) {  Serial.println("Appears we have a measurement ok. Processing it."); }
      water_temp = my_results_f[0];
      air_temp = my_results_f[1];
      system_temp = my_results_f[2];
      //float primary_temp = water_temp;   // Use this to select which of the temperatures we are primarily interested in
      water_temp_max = track_max(water_temp, water_temp_max);
      air_temp_max = track_max(air_temp, air_temp_max);
      system_temp_max = track_max(system_temp, system_temp_max);

      water_temp_min = track_min(water_temp, water_temp_min);
      air_temp_min = track_min(air_temp, air_temp_min);
      system_temp_min = track_min(system_temp, system_temp_min);

      // Do averages
      if (startup_tick < g_ave_values)
      {
        startup_tick = startup_tick + 1;
      }
      //rm_temp_ave = float_average(rm_temp, rm_temp_ave, g_ave_values);
      water_temp_ave = float_average(water_temp, water_temp_ave, g_ave_values);
      air_temp_ave = float_average(air_temp, air_temp_ave, g_ave_values);
      system_temp_ave = float_average(system_temp, system_temp_ave, g_ave_values);

      //temp_prev = primary_temp;
      air_temp_prev = air_temp;
      water_temp_prev = water_temp;
      system_temp_prev = system_temp;

      good_measurement = true;
      measuring_flag = false;
      if (debug_serial) {  Serial.println("Temp Measurement is Good."); }


      // With that let's on to the water level test.
      if (debug_serial) {  Serial.print("Starting water level measurement..."); }
      long unsigned usonic_results[5];
      //unsigned long min, max, average, variance, std_dev;
      if (debug_serial) {  Serial.print("calling unsonic_ping"); }
      usonic_mping(1, usonic_results);
      water_level_stdev = usonic_results[4];
      // How reliable is the measurement?
      if (water_level_stdev > 100) {
        // That's not reliable enough
        // We can try again I guess
        if (debug_serial) {  Serial.print("calling unsonic_ping - again"); }
        usonic_mping(1, usonic_results);
        water_level_stdev = usonic_results[4];
      }
        //good_measurement = false; // should be a pointless thing, but can't be too careful
        //measuring_flag = false; // without resetting the last_measurement time this should result in an immediate re-measure
      //} else {
      water_level = microsecondsToMillimeters(usonic_results[2]);
      //}
      if (debug_serial) {  Serial.println("Finished water level measurement."); }

      if (debug_serial) {  Serial.println("Starting battery voltage measurement."); }
      battery_mV = check_battery();
      battery_percent = get_battery_percentage(battery_mV);
      battery_status_text(battery_mV, current_battery_status);
      if (debug_serial) {
        Serial.print("Battery (mV): ");
        Serial.println(battery_mV);
      }
      if (debug_serial) {  Serial.println("Finished battery voltage measurement."); }

      // TODO - Check solar status
      //if (debug_serial) {  Serial.println("Starting solar voltage measurement."); }
      //solar_mV = check_solar();
      //battery_status_text(battery_mV, current_battery_status);
      //if (debug_serial) {
      //  Serial.print("Solar (mV): ");
      //  Serial.println(solar_mV);
      //}
      //if (debug_serial) {  Serial.println("Finished solar voltage measurement."); }

      if (good_measurement) {
        last_measurement = millis();
      }

      // Basic valve control for testing
      // Should be AFTER checking battery level.
      operate_valve();

    }
    else
    {
      // Some sort of measurement problem - oh what to do, what to do...
      // Just start another measurement asap please
      if (debug_serial) {  Serial.println("Error getting measurement. Resetting variables..."); }
      good_measurement = false; // should be a pointless thing, but can't be too careful
      measuring_flag = false; // without resetting the last_measurement time this should result in an immediate re-measure
    }
  }
  //
  //
  //
  //
  // The following sections require a network connection, so best check we're properly connected
  // It might be good if I seperated out the Cloud dependant items, and those that just need WiFi
  // Right now, although mostly not necessary, if Particle Cloud is not available we won't log locally
  //if ( Particle.connected() )
  // if ( WiFi.ready() && (WiFi.RSSI() < 0))
  if ( (millis() - last_flask) > (interval_flask - 10000))
  {
    // remove the sleeping indicator
    sleeping = false;
    if ( !WiFi.connecting() ) { WiFi.connect(); }
    last_connection_check = millis();
  }
  if ( WiFi.ready() && not(sleeping) )
  {
    //RGB.control(true);
    // Section for logging to local database via REST JSON FLASK
    if ( (millis() - last_flask > interval_flask) && good_measurement)
    {
      if (debug_serial) {  Serial.println("\r\nStarting to log to flask..."); }
      HttpClient http;
      http_header_t headers[] = {
        { "Content-Type", "application/json" },
        // { "Accept" , "application/json" },
        { "Accept" , "*/*"},
        { NULL, NULL } // NOTE: Always terminate headers will NULL
      };
      http_request_t request;
      http_response_t response;
      request.hostname = "flask.home";
      request.port = flask_port;
      request.path = flask_path;
      //if (debug_serial) {  Serial.print("Getting RSSI...."); }
      //int myrssi  = WiFi.RSSI();
      //if (debug_serial) {  Serial.printf("Done (%d)\r\n",myrssi); }
      uint32_t freemem = System.freeMemory();
      sprintf(postdata, "{\"tablename\":\"%s\",\
      \"air_temp\":\"%.1f\",\"air_temp_ave\":\"%.1f\",\"air_temp_max\":\"%.1f\",\"air_temp_min\":\"%.1f\",\
      \"water_temp\":\"%.1f\",\"water_temp_ave\":\"%.1f\",\"water_temp_max\":\"%.1f\",\"water_temp_min\":\"%.1f\",\
      \"system_temp\":\"%.1f\",\"system_temp_ave\":\"%.1f\",\"system_temp_max\":\"%.1f\",\"system_temp_min\":\"%.1f\",\
      \"water_level\":\"%d\",\"water_level_stdev\":\"%d\",\"valve_status\":\"%d\",\
      \"battery_mV\":\"%d\",\"battery_percent\":\"%d\",\"battery_status\":\"%s\",\
      \"freemem\":\"%d\",\"initmem\":\"%d\",\"reconnects\":\"%d\",\"version\":\"%s\"}",
      tablename,
      air_temp, air_temp_ave, air_temp_max, air_temp_min,
      water_temp, water_temp_ave, water_temp_max, water_temp_min,
      system_temp, system_temp_ave, system_temp_max, system_temp_min,
      water_level, water_level_stdev, valve_status,
      battery_mV, battery_percent, current_battery_status,
      freemem, initial_mem_usage, reconnect_count, myversion
      );
      if (debug_serial)
      {
        Serial.println("Posting to flask now.");
        Serial.println(postdata);
      }
      request.body = postdata;
      //PhotonWdgs::tickle();
      http.post(request, response, headers);
      //
      if (debug_serial)
      {
        Serial.print("Application>\tResponse status: ");
        Serial.println(response.status);
        Serial.print("Application>\tHTTP Response Body: ");
        Serial.println(response.body);
      }
      if (response.status == 299)
      {
        // This signals that we should enter Safe mode
        // It's a way to ensure we can remotely update the code, as the WiFi sleeps
        reconnect_count = 0;
        System.enterSafeMode();
      } else if (response.status == 200)
      {
        // All is well
        reconnect_count = 0;
      }
      last_flask = millis();
      if (debug_serial) {  Serial.println("Done logging to flask."); }
      // So, if we've logged, we can go to sleep for a little while
      // interval_flask - 5000 (ms)
      // we only sleep if the valve is closed
      if ((valve_status == 0) && (interval_flask > 15000))
      {
        //System.sleep(uint16_t wakeUpPin, uint16_t edgeTriggerMode, long seconds)
        // A3 is unused
        // sleeping flag not used as this is STOP
        //delay(250);
        //System.sleep(A3, RISING, (interval_flask - 5000));

        // Or perhaps just used WiFi sleep
        // and power down usonic via ULN2003 - TODO
        //System.sleep(interval_flask - 10000);
        //sleeping = true;
        //digitalWrite(usonic_enable, LOW);
        System.sleep(sleep_wakeupPin, RISING, interval_sleep);
        //PhotonWdgs::tickle();
        delay(1000);
        //digitalWrite(usonic_enable, HIGH);
      }

    }
    // Section for logging to Particle Cloud - called once a minute, which is more than enough
    if ( Particle.connected() && not(sleeping))
    {
      if ( (millis() - last_cloud > interval_cloud) && good_measurement)
      {
        if (debug_serial) {  Serial.println("Starting logging to Particle Cloud. DISABLED"); }
        int myrssi  = 20;
        uint32_t freemem = System.freeMemory();
        //PhotonWdgs::tickle();
        //sprintf(resultstr, "{\"reconnect_count\":%lu,\"RSSI\":%d}",
        //reconnect_count,
        //myrssi
        //);
        last_cloud = millis();
        if (debug_serial) {  Serial.println("Finished logging to Particle Cloud."); }
      }
    }
    else if (millis() - last_cloud_connection_check > 60000 ) //WiFi connected but Particle Cloud not...
    {
      // Attempt Recovery of Particle Cloud Connection not more than once a minute though
      if (debug_serial) {  Serial.println("Not connected to Particle Cloud and over 60s since last check.."); }
      // Only here if Particle not connected to Cloud
      // This relies on us running the System Thread.
      // Note this will still be subject to our Watchdog (I think)
      //waitUntil(Particle.connected);
      // wait for the cloud connection to be connected or timeout after 8 seconds
      // Must be a shorter interval than the watchdog timer!
      // RGB.control(false);
      //PhotonWdgs::tickle();
      if (not(waitFor(Particle.connected, 8000))) {
        reconnect_count++;
        if (debug_serial) {
          Serial.printf("Particle not connected in 8000 msec... oh dear... this is  an incident: %d\r\n",reconnect_count);
        }
        // Some sort of network problem - oh what to do, what to do...
        // Just start another measurement asap, got to assume this one is out of date now
        good_measurement = false; // should be a pointless thing, but can't be too careful
        measuring_flag = false; // without resetting the last_measurement time this should result in an immediate re-measure
        if (reconnect_count > 15)  // Just to be on the safe side, if we're seeing lots of these, let's reset.
        {
          if (debug_serial) {
            Serial.println("Resetting due to cloud connection issues");
            Serial.flush();
            delay(100);
          }
          System.reset();
        }
      } else {
        // Reset connection count when we get a good connection
        reconnect_count = 0;
      }
      // RGB.control(true);
      last_cloud_connection_check = millis();
    }
  }
  else if ( (millis() - last_connection_check > interval_connection_check) && not(sleeping) )  // This is the else for the WiFi connectivity check
  {
    if (debug_serial) {
      Serial.print("Not connected to WiFi and Xs since last check..");
      Serial.printlnf("System Memory is: %d",System.freeMemory());
    }
    // Only here if WiFi not connected and we're not sleeping.
    // This relies on us running the System Thread.
    // Note this will still be subject to our Watchdog (I think)
    //waitUntil(Particle.connected);
    // wait for the cloud connection to be connected or timeout after 8 seconds
    // Must be a shorter interval than the watchdog timer!
    // RGB.control(false);
    if ( !WiFi.connecting() ) { WiFi.connect(); }
    last_connection_check = millis();
  }
  // Need to check for memory leaks (poss in TCP Client, especially on reconnets)
  // Probably don't need to do this EVERY loop...
  if ( initial_mem_usage < System.freeMemory() )
  {
    initial_mem_usage = System.freeMemory();            // Allows for released memory (seems necessary since 4.9), as really just want to check for memory leaks
    if (debug_serial) {  Serial.println("Init mem bump up."); }
  }
  if ( initial_mem_usage - System.freeMemory() > 9128 ) // This is an arbitrary figure, chosen by experiment, may need to change
  {
    if (debug_serial) {  Serial.println("resetting due to memory leak"); }
    delay(250);
    System.reset();
  }
  if (millis() - last_idle_blip > 100) {
    if (debug_serial) {  Serial.print("."); }
    last_idle_blip = millis();
  }
}
