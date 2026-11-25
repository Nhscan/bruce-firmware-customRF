#ifndef RF_WEATHER_STATION_H
#define RF_WEATHER_STATION_H

// Native live weather-sensor monitor: decodes AcuRite-606TX, the AcuRite TXR
// PWM family (Tower/3n1/5n1/6045M/Atlas), Nexus/Nexus-TH, Prologue and
// LaCrosse TX141TH-Bv2 directly from CC1101 captures and shows a
// Flipper-Zero-style station list.
//
// Also provides an on-device "Decode .sub file" browser for Flipper/Bruce RAW
// captures. See WEATHER_STATION.md for supported protocols and tuning.
void rf_weather_station();

#endif
