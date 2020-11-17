/*  
 *  ------------  [GP_v30_01] - Electrochemical gas sensors  -------------- 
 *  
 *  Explanation: This is the basic code to manage and read an electrochemical
 *  gas sensor. These sensors include: CO, O2, O3, NO, NO2, SO2, NH3, H2, H2S,
 *  HCl, HCN, PH3, ETO and Cl2. Cycle time: 2 minutes
 *  
 *  Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify  
 *  it under the terms of the GNU General Public License as published by  
 *  the Free Software Foundation, either version 3 of the License, or  
 *  (at your option) any later version.  
 *   
 *  This program is distributed in the hope that it will be useful,  
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of  
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
 *  GNU General Public License for more details.  
 *   
 *  You should have received a copy of the GNU General Public License  
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  
 * 
 *  Version:           3.1
 *  Design:            David Gascón 
 *  Implementation:    Alejandro Gállego
 */

#include <WaspSensorGas_Pro.h>

/*
 * Define object for sensor: gas_PRO_sensor
 * Input to choose board socket. 
 * Waspmote OEM. Possibilities for this sensor:
 *   - SOCKET_1 
 *  - SOCKET_2
 *  - SOCKET_3
 *  - SOCKET_4
 *  - SOCKET_5
 *  - SOCKET_6
 * P&S! Possibilities for this sensor:
 *  - SOCKET_A
 *  - SOCKET_B
 *  - SOCKET_C
 *  - SOCKET_F
 */
//Gas sensor_co2(SOCKET_1);
Gas sensor_h2s(SOCKET_2);
Gas sensor_so2(SOCKET_3);
Gas sensor_co(SOCKET_4);
Gas sensor_nh3(SOCKET_5);
Gas sensor_no2(SOCKET_6);

bmeGasesSensor  bme;

float concentration_co2, concentration_h2s, concentration_so2, concentration_co, concentration_nh3, concentration_no2;  // Stores the concentration level in ppm
float temperature;  // Stores the temperature in ºC
float humidity;   // Stores the realitve humidity in %RH
float pressure;   // Stores the pressure in Pa

void setup()
{
    //USB.println(F("Electrochemical gas sensor example"));
  
    ///////////////////////////////////////////
    // 1. Turn on the sensors
    /////////////////////////////////////////// 

    // Power on the electrochemical sensor. 
    // If the gases PRO board is off, turn it on automatically.
    //sensor_co2.ON();
    sensor_h2s.ON();
    sensor_so2.ON();
    sensor_co.ON();
    sensor_nh3.ON();
    sensor_no2.ON();
  
    // First sleep time
    // After 2 minutes, Waspmote wakes up thanks to the RTC Alarm
    PWR.deepSleep("00:00:02:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_ON);
  
} 


void loop()
{

    ///////////////////////////////////////////
    // 2. Read sensors
    ///////////////////////////////////////////  

    // Read the electrochemical sensor and compensate with the temperature internally
    //concentration_co2 = sensor_co2.getConc();
    concentration_h2s = sensor_h2s.getConc();
    concentration_so2 = sensor_so2.getConc();
    concentration_co = sensor_co.getConc();
    concentration_nh3 = sensor_nh3.getConc();
    concentration_no2 = sensor_no2.getConc();

    // Read enviromental variables PAREI AQUI
    temperature = bme.getTemperature();
    humidity = bme.getHumidity();
    pressure = bme.getPressure();

    // And print the values via USB
    //USB.println(F("*************"));
    USB.print(F("Gas concentration:"));
    USB.print(concentration_h2s);
    USB.print(F(","));
    USB.print(concentration_so2);
    USB.print(F(","));
    USB.print(concentration_co);
    USB.print(F(","));
    USB.print(concentration_nh3);
    USB.print(F(","));
    USB.print(concentration_no2);
    USB.print(F(","));
    //USB.println(F(" ppm"));
    //USB.print(F("Temperature: "));
    USB.print(temperature);
    USB.print(F(","));
    //USB.print(F("RH: "));
    USB.print(humidity);
    USB.print(F(","));
    //USB.print(F("Pressure: "));
    USB.print(pressure);
    USB.println(F(","));
    //USB.println(F(" Pa"));

    ///////////////////////////////////////////
    // 3. Power off sensors
    ///////////////////////////////////////////  

    // Power off the NDIR sensor. If there aren't more gas sensors powered,
    // turn off the board automatically
    //sensor_co2.OFF();
    ///////////////////////////////////////////
    // 5. Sleep
    /////////////////////////////////////////// 
    
    // Go to deepsleep  
    // After 2 minutes, Waspmote wakes up thanks to the RTC Alarm
    PWR.deepSleep("00:00:00:02", RTC_OFFSET, RTC_ALM1_MODE1, ALL_ON);

}
