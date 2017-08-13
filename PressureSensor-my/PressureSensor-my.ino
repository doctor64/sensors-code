/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Pressure sensor example using BMP085 module  
 * http://www.mysensors.org/build/pressure
 *
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 
#define DEBUG_PRINT


// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL  113
//#define MY_RADIO_RFM69
//#define MY_NODE_ID 2

#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <Sodaq_BMP085.h>
#include <Sodaq_SHT2x.h>

#define BARO_BMP_CHILD 0
#define TEMP0_BMP_CHILD 1
#define TEMP1_SHT_CHILD 2
#define TEMP2_SHT_CHILD 3
#define HUM0_SHT_CHILD 4
#define BAR1_BMP_CHILD 5

const float ALTITUDE = 580; // <-- adapt this value to your own location's altitude. Sofia home
float BMP_CAL = 2.2;
float SHT_CAL = 1.27;

// Sleep time between reads (in seconds). Do not change this value as the forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000; 

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
	STABLE = 0,			// "Stable Weather Pattern"
	SUNNY = 1,			// "Slowly rising Good Weather", "Clear/Sunny "
	CLOUDY = 2,			// "Slowly falling L-Pressure ", "Cloudy/Rain "
	UNSTABLE = 3,		// "Quickly rising H-Press",     "Not Stable"
	THUNDERSTORM = 4,	// "Quickly falling L-Press",    "Thunderstorm"
	UNKNOWN = 5			// "Unknown (More Time needed)
};

//Adafruit_BMP085 bmp = Adafruit_BMP085();      // Digital Pressure Sensor 
Sodaq_BMP085 bmp;// = Adafruit_BMP085();      // Digital Pressure Sensor 

float lastPressure = -1;
float lastPressure_raw = -1;
float lastTemp_bmp = -1;
float lastTemp_sht = -1;
float lastHum = -1;
float lastDew = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10 
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;
bool metric;
MyMessage tempBMPMsg(TEMP0_BMP_CHILD, V_TEMP);
MyMessage pressureBMPMsg(BARO_BMP_CHILD, V_PRESSURE);
MyMessage pressureBMP_rawMsg(BAR1_BMP_CHILD, V_PRESSURE);
MyMessage forecastBMPMsg(BARO_BMP_CHILD, V_FORECAST);
MyMessage tempSHTMsg(TEMP1_SHT_CHILD, V_TEMP);
MyMessage humSHTMsg(HUM0_SHT_CHILD, V_HUM);
MyMessage dewSHTMsg(TEMP2_SHT_CHILD, V_TEMP);

void setup() 
{
  bmp.begin(BMP085_ULTRAHIGHRES);
	/*if (!bmp.begin()) 
	{
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
		while (1) {}
	}*/
	metric = getControllerConfig().isMetric;
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Pressure/temp/hum Sensor", "1.2");

  // Register sensors to gw (they will be created as child devices)
  present(BARO_BMP_CHILD, S_BARO, "Barometer");
  present(BAR1_BMP_CHILD, S_BARO, "Barometer raw");
  present(TEMP0_BMP_CHILD, S_TEMP, "Temperature bmp");
  present(TEMP1_SHT_CHILD, S_TEMP, "Temperature sht" );
  present(HUM0_SHT_CHILD, S_HUM, "Humidity");
  present(TEMP2_SHT_CHILD, S_TEMP, "Dew point");
}

void loop() 
{
	float pressure = bmp.readPressure(ALTITUDE) / 100.0;
  float pressure_raw = bmp.readPressure() / 100.0;
	float temperature_bmp = bmp.readTemperature() + BMP_CAL;
  float temperature_sht = SHT2x.GetTemperature() + SHT_CAL;
  float dew_sht = SHT2x.GetDewPoint();
  float hum_sht = SHT2x.GetHumidity();
  
	if (!metric) 
	{
		// Convert to fahrenheit
		temperature_bmp = temperature_bmp * 9.0 / 5.0 + 32.0;
    temperature_sht = temperature_sht * 9.0 / 5.0 + 32.0;
    dew_sht = dew_sht * 9.0 / 5.0 + 32.0;
	}

	int forecast = sample(pressure);
  
#ifdef DEBUG_PRINT
	Serial.print("Temperature BMP = ");
	Serial.print(temperature_bmp);
	Serial.println(metric ? " *C" : " *F");
	Serial.print("Pressure = ");
	Serial.print(pressure);
	Serial.println(" hPa");
  Serial.print("Pressure raw = ");
  Serial.print(pressure_raw);
  Serial.println(" hPa");
	Serial.print("Forecast = ");
	Serial.println(weather[forecast]);
  Serial.print("Temperature SHT = ");
  Serial.print(temperature_sht);
  Serial.println(metric ? " *C" : " *F");
  Serial.print("Dew point = ");
  Serial.print(dew_sht);
  Serial.println(metric ? " *C" : " *F");
  Serial.print("Humidity SHT = ");
  Serial.print(hum_sht);
  Serial.println(" %");
#endif

	if (temperature_bmp != lastTemp_bmp) 
	{
		send(tempBMPMsg.set(temperature_bmp, 1));
		lastTemp_bmp = temperature_bmp;
	}
  
  if (temperature_sht != lastTemp_sht) 
  {
    send(tempSHTMsg.set(temperature_sht, 1));
    lastTemp_sht = temperature_sht;
  }
  
  if (dew_sht != lastDew) 
  {
    send(dewSHTMsg.set(dew_sht, 1));
    lastDew = dew_sht;
  }

  if (hum_sht != lastHum) 
  {
    send(humSHTMsg.set(hum_sht, 1));
    lastHum = hum_sht;
  }  

	if (pressure != lastPressure) 
	{
		send(pressureBMPMsg.set(pressure, 2));
		lastPressure = pressure;
	}
  if (pressure_raw != lastPressure_raw) 
  {
    send(pressureBMP_rawMsg.set(pressure_raw, 2));
    lastPressure = pressure;
  }

	if (forecast != lastForecast)
	{
		send(forecastBMPMsg.set(weather[forecast]));
		lastForecast = forecast;
	}

	//smartSleep(SLEEP_TIME);
  wait(SLEEP_TIME);
}

float getLastPressureSamplesAverage()
{
	float lastPressureSamplesAverage = 0;
	for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
	{
		lastPressureSamplesAverage += lastPressureSamples[i];
	}
	lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

	return lastPressureSamplesAverage;
}



// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
	// Calculate the average of the last n minutes.
	int index = minuteCount % LAST_SAMPLES_COUNT;
	lastPressureSamples[index] = pressure;

	minuteCount++;
	if (minuteCount > 185)
	{
		minuteCount = 6;
	}

	if (minuteCount == 5)
	{
		pressureAvg = getLastPressureSamplesAverage();
	}
	else if (minuteCount == 35)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change * 2; // note this is for t = 0.5hour
		}
		else
		{
			dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
		}
	}
	else if (minuteCount == 65)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) //first time initial 3 hour
		{
			dP_dt = change; //note this is for t = 1 hour
		}
		else
		{
			dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 95)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 1.5; // note this is for t = 1.5 hour
		}
		else
		{
			dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 125)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		pressureAvg2 = lastPressureAvg; // store for later use.
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2; // note this is for t = 2 hour
		}
		else
		{
			dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 155)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2.5; // note this is for t = 2.5 hour
		}
		else
		{
			dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 185)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 3; // note this is for t = 3 hour
		}
		else
		{
			dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
	}

	int forecast = UNKNOWN;
	if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
	{
		forecast = UNKNOWN;
	}
	else if (dP_dt < (-0.25))
	{
		forecast = THUNDERSTORM;
	}
	else if (dP_dt > 0.25)
	{
		forecast = UNSTABLE;
	}
	else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
	{
		forecast = CLOUDY;
	}
	else if ((dP_dt > 0.05) && (dP_dt < 0.25))
	{
		forecast = SUNNY;
	}
	else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
	{
		forecast = STABLE;
	}
	else
	{
		forecast = UNKNOWN;
	}

	// uncomment when debugging
 #ifdef DEBUG_PRINT
	Serial.print(F("Forecast at minute "));
	Serial.print(minuteCount);
	Serial.print(F(" dP/dt = "));
	Serial.print(dP_dt);
	Serial.print(F("kPa/h --> "));
	Serial.println(weather[forecast]);
  Serial.print(F("Pressure average "));
  Serial.print(pressureAvg);
  Serial.print(F(" Pressure average 2 "));
  Serial.println(pressureAvg2);
  
 #endif

	return forecast;
}
