#include <Arduino.h>
#include <ModbusMaster.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <config.h>
#ifdef InfluxDb
  #include <RunningAverage.h>
#endif
#ifdef telnetDebug
    #include <TelnetStream.h>
#endif

static constexpr int avSamples = 60;
ModbusMaster Growatt;
uint8_t MODBUSresult;
unsigned long lastUpdate;
int failures; //The number of failed WiFi or send attempts. Will automatically restart the ESP if too many failures occurr in a row.
byte initialBoot = 1; // Used to track reboot and reconnects

struct modBusHoldingRegistersStruct{
  const char *topic;
  const byte address;
  const byte length;
  const byte type; // 0 = word (uint16-t) output as 4 CHARs (i.e. 0xFA03), 1 = int (0-65535), 2 = ASCII
  const byte MQTTsubscribe;
};

static constexpr modBusHoldingRegistersStruct modBusHoldingRegisters[] = {
  // topic, address, length, type, MQTTsubscribe
  {"On_Off", 0, 1, 0, 1},
  {"OutputConfig", 1, 1, 0, 1},
  {"ChargeConfig", 2, 1, 0, 1},
  {"UtiOutStart", 3, 1, 0, 1},
  {"UtiOutEnd", 4, 1, 0, 1},
  {"UtiChargeStart", 5, 1, 0, 1},
  {"UtiChargeEnd", 6, 1, 0, 1},
  {"PVModel", 7, 1, 0, 1},
  {"ACInModel", 8, 1, 0, 1},
  {"Fw1_ver", 9, 3, 2, 0},
  // {"Fw1_ver_M", 10, 1, 2, 0},
  // {"Fw1_ver_L", 11, 1, 2, 0},
  {"Fw2_Cont_ver", 12, 3, 2, 0},
  // {"Fw2_Cont_ver_M", 13, 1, 2, 0},
  // {"Fw2_Cont_ver_L", 14, 1, 2, 0},
  {"LCDlanguage", 15, 1, 0, 1},
  // {"", 16, 1, 0, 0},
  // {"", 17, 1, 0, 0},
  {"OutputVoltType", 18, 1, 0, 1},
  {"OutputFreqType", 19, 1, 0, 1},
  {"OverLoadRestart", 20, 1, 0, 1},
  {"OverTempRestart", 21, 1, 0, 1},
  {"BuzzerEN", 22, 1, 0, 1},
  {"SerialNo", 23, 5, 0, 0},
  // {"SerialNo4", 24, 1, 2, 0},
  // {"SerialNo3", 25, 1, 2, 0},
  // {"SerialNo2", 26, 1, 2, 0},
  // {"SerialNo1", 27, 1, 2, 0},
  {"InvMoudle_H", 28, 1, 0, 0},
  {"InvMoudle_L", 29, 1, 0, 0},
  {"ComAddress", 30, 1, 0, 0},
  {"FlashStart", 31, 1, 0, 0},
  {"Reset_User_Info", 32, 1, 0, 0},
  {"Reset_to_factory", 33, 1, 0, 0},
  {"MaxChargeCurr", 34, 1, 0, 1},
  {"BulkChargeVolt", 35, 1, 0, 1},
  {"FloatChargeVolt", 36, 1, 0, 1},
  {"BatLowToUtiVolt", 37, 1, 0, 1},
  {"FloatChargeCurr", 38, 1, 0, 1},
  {"Battery_Type", 39, 1, 0, 1},
  {"Aging_Mode", 40, 1, 0, 1},
  {"", 41, 1, 0, 0},
  {"", 42, 1, 0, 0},
  {"DTC", 43, 1, 2, 0},
  // {"", 44, 1, 0, 0},
  {"SysYear", 45, 1, 0, 1},
  {"SysMonth", 46, 1, 0, 1},
  {"SysDay", 47, 1, 0, 1},
  {"SysHour", 48, 1, 0, 1},
  {"SysMinute", 49, 1, 0, 1},
  {"SysSecond", 50, 1, 0, 1},
  {"", 51, 1, 0, 0},
  // {"", 52, 1, 0, 0},
  // {"", 53, 1, 0, 0},
  // {"", 54, 1, 0, 0},
  // {"", 55, 1, 0, 0},
  // {"", 56, 1, 0, 0},
  // {"", 57, 1, 0, 0},
  {"", 58, 1, 0, 0},
  {"MfrInfo8_H", 59, 8, 0, 0},
  {"MfrInfo7_M", 60, 0, 0, 0},
  {"MfrInfo6_L", 61, 0, 0, 0},
  {"MfrInfo5_H", 62, 0, 0, 0},
  {"MfrInfo4_M", 63, 0, 0, 0},
  {"MfrInfo3_L", 64, 0, 0, 0},
  {"MfrInfo2_L", 65, 0, 0, 0},
  {"MfrInfo1_H", 66, 0, 0, 0},
  {"Fw2_Cont_Build", 67, 2, 0, 0},
  // {"Fw2_Cont_Build1", 68, 0, 0, 0},
  {"Fw_Com_Build", 69, 2, 0, 0},
  // {"Fw_Com_Build1", 70, 0, 0, 0},
  // {"", 71, 1, 0, 0},
  {"Sys_Weekly", 72, 1, 0, 1},
  {"ModbusVer", 73, 1, 0, 0},
  // {"", 74, 1, 0, 0},
  // {"", 75, 1, 0, 0},
  {"RateWatt", 76, 2, 1, 0},
  // {"RateWatt_L", 77, 1, 0, 0},
  {"RateVA", 78, 2, 1, 0},
  // {"RateVA_L", 79, 1, 0, 0},
  {"ODM_Info", 80, 1, 0, 0},
  // {"", 81, 1, 0, 0},
  // {"", 82, 1, 0, 0},
  // {"", 83, 1, 0, 0},
  // {"", 84, 1, 0, 0},
  // {"", 85, 1, 0, 0},
  // {"", 86, 1, 0, 0},
  // {"", 87, 1, 0, 0},
  // {"", 88, 1, 0, 0},
  // {"", 89, 1, 0, 0},
  // {"", 90, 1, 0, 0},
  // {"", 91, 1, 0, 0},
  // {"", 92, 1, 0, 0},
  // {"", 93, 1, 0, 0},
  // {"", 94, 1, 0, 0},
  // {"", 95, 1, 0, 0},
  // {"", 96, 1, 0, 0},
  // {"", 97, 1, 0, 0},
  // {"", 98, 1, 0, 0},
  // {"", 99, 1, 0, 0},
  // {"", 100, 1, 0, 0},
  // {"", 101, 1, 0, 0},
  // {"", 102, 1, 0, 0},
  // {"", 103, 1, 0, 0},
  // {"", 104, 1, 0, 0},
  {"", 105, 1, 0, 0},
  {"", 106, 1, 0, 0},
  // {"", 107, 1, 0, 0},
  // {"", 108, 1, 0, 0},
  {"", 109, 1, 0, 0},
  // {"", 110, 1, 0, 0},
  // {"", 111, 1, 0, 0},
  // {"", 112, 1, 0, 0},
  // {"", 113, 1, 0, 0},
  // {"", 114, 1, 0, 0},
  // {"", 115, 1, 0, 0},
  // {"", 116, 1, 0, 0},
  // {"", 117, 1, 0, 0},
  // {"", 118, 1, 0, 0},
  // {"", 119, 1, 0, 0},
  // {"", 120, 1, 0, 0},
  // {"", 121, 1, 0, 0},
  // {"", 122, 1, 0, 0},
  // {"", 123, 1, 0, 0},
  // {"", 124, 1, 0, 0},
  // {"", 125, 1, 0, 0},
  {"", 126, 1, 0, 0},
  {"", 127, 1, 0, 0},
  // {"", 128, 1, 0, 0},
  {"", 129, 1, 0, 0},
  {"", 130, 1, 0, 0},
  {"", 131, 1, 0, 0},
  {"", 132, 1, 0, 0},
  // {"", 133, 1, 0, 0},
  {"", 134, 1, 0, 0},
  {"", 135, 1, 0, 0},
  {"", 136, 1, 0, 0},
  {"", 137, 1, 0, 0},
  // {"", 138, 1, 0, 0},
  // {"", 139, 1, 0, 0},
  // {"", 140, 1, 0, 0},
  // {"", 141, 1, 0, 0},
  // {"", 142, 1, 0, 0},
  // {"", 143, 1, 0, 0},
  {"", 144, 1, 0, 0},
  // {"", 145, 1, 0, 0},
  // {"", 146, 1, 0, 0},
  // {"", 147, 1, 0, 0},
  // {"", 148, 1, 0, 0},
  {"", 149, 1, 0, 0},
  {"", 150, 1, 0, 0},
  // {"", 151, 1, 0, 0},
  // {"", 152, 1, 0, 0},
  {"", 153, 1, 0, 0},
  // {"", 154, 1, 0, 0},
  // {"", 155, 1, 0, 0},
  // {"", 156, 1, 0, 0},
  // {"", 157, 1, 0, 0},
  // {"", 158, 1, 0, 0},
  // {"", 159, 1, 0, 0},
  {"", 160, 1, 0, 0},
  // {"", 161, 1, 0, 0},
  {"BL_Ver2", 162, 1, 0, 0},
  // {"", 163, 1, 0, 0},
  // {"", 164, 1, 0, 0},
  // {"", 165, 1, 0, 0},
  // {"", 166, 1, 0, 0},
  // {"", 167, 1, 0, 0},
  {"", 168, 1, 0, 0},
  // {"", 169, 1, 0, 0},
  // {"", 170, 1, 0, 0},
  // {"", 171, 1, 0, 0},
  {"", 172, 1, 0, 0},
  // {"", 173, 1, 0, 0},
  // {"", 174, 1, 0, 0},
  // {"", 175, 1, 0, 0},
  // {"", 176, 1, 0, 0},
  // {"", 177, 1, 0, 0},
  // {"", 178, 1, 0, 0},
  // {"", 179, 1, 0, 0},
  // {"", 180, 1, 0, 0},
  // {"", 181, 1, 0, 0},
  // {"", 182, 1, 0, 0},
  // {"", 183, 1, 0, 0},
  // {"", 184, 1, 0, 0},
  // {"", 185, 1, 0, 0},
  {"", 186, 1, 0, 0},
  {"", 187, 1, 0, 0},
  // {"", 188, 1, 0, 0},
  // {"", 189, 1, 0, 0},
  // {"", 190, 1, 0, 0},
  // {"", 191, 1, 0, 0},
  {"", 192, 1, 0, 0},
  // {"", 193, 1, 0, 0},
  // {"", 194, 1, 0, 0},
  // {"", 195, 1, 0, 0},
  // {"", 196, 1, 0, 0},
  {"", 197, 1, 0, 0},
  // {"", 198, 1, 0, 0},
  // {"", 199, 1, 0, 0},
  {"", 200, 1, 0, 0},
  {"", 201, 1, 0, 0},
  // {"", 202, 1, 0, 0},
  {"", 203, 1, 0, 0},
  // {"", 204, 1, 0, 0},
  // {"", 205, 1, 0, 0},
  // {"", 206, 1, 0, 0},
  // {"", 207, 1, 0, 0},
  // {"", 208, 1, 0, 0},
  {"", 209, 1, 0, 0},
  // {"", 210, 1, 0, 0},
  {"", 211, 1, 0, 0},
  {"", 212, 1, 0, 0},
  {"", 213, 1, 0, 0},
  {"", 214, 1, 0, 0},
  {"", 215, 1, 0, 0},
  // {"", 216, 1, 0, 0},
  {"", 217, 1, 0, 0},
  {"", 218, 1, 0, 0},
  // {"", 219, 1, 0, 0},
  // {"", 220, 1, 0, 0},
  // {"", 221, 1, 0, 0},
  // {"", 222, 1, 0, 0},
  // {"", 223, 1, 0, 0},
  // {"", 224, 1, 0, 0},
  // {"", 225, 1, 0, 0},
  // {"", 226, 1, 0, 0},
  // {"", 227, 1, 0, 0},
  // {"", 228, 1, 0, 0},
  // {"", 229, 1, 0, 0},
  // {"", 230, 1, 0, 0},
  // {"", 231, 1, 0, 0},
  // {"", 232, 1, 0, 0},
  // {"", 233, 1, 0, 0},
  // {"", 234, 1, 0, 0},
  // {"", 235, 1, 0, 0},
  // {"", 236, 1, 0, 0},
  // {"", 237, 1, 0, 0},
  // {"", 238, 1, 0, 0},
  // {"", 239, 1, 0, 0},
  // {"", 240, 1, 0, 0},
  // {"", 241, 1, 0, 0},
  // {"", 242, 1, 0, 0},
  // {"", 243, 1, 0, 0},
  // {"", 244, 1, 0, 0},
  // {"", 245, 1, 0, 0},
  // {"", 246, 1, 0, 0},
  // {"", 247, 1, 0, 0},
  // {"", 248, 1, 0, 0},
  // {"", 249, 1, 0, 0},
  // {"", 250, 1, 0, 0}
};

#ifdef InfluxDb
  #include <InfluxDbClient.h>
  #ifdef verifyCerts
    InfluxDBClient InfluxDBclient(SECRET_INFLUXDB_URL, SECRET_INFLUXDB_ORG, SECRET_INFLUXDB_BUCKET, SECRET_INFLUXDB_TOKEN, INFLUXDB_CA_CERT);
  #else
    InfluxDBClient InfluxDBclient(SECRET_INFLUXDB_URL, SECRET_INFLUXDB_ORG, SECRET_INFLUXDB_BUCKET, SECRET_INFLUXDB_TOKEN);
  #endif

  struct stats{
    const char *name;
    const byte address;
    const byte type; //Whether the result is 16 or 32 bit number.
    float value;
    RunningAverage average;
    const float multiplier;
    Point measurement;
  };

  stats arrstats[] = {
    //Register name, MODBUS address, integer type (0 = uint16_t​, 1 = uint32_t​, 2 = int32_t​), value, running average, multiplier, InfluxDB measurement)
    {"System_Status", 0, 0, 0.0, RunningAverage(avSamples), 1.0, Point("System_Status")},
    {"PV_Voltage", 1, 0, 0.0, RunningAverage(avSamples), 0.1, Point("PV_Voltage")},
    {"PV_Power", 3, 1, 0.0, RunningAverage(avSamples), 0.1, Point("PV_Power")},
    {"Buck_Converter_Current", 7, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Buck_Converter_Current")},
    {"Output_Watts", 9, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Output_Watts")},
    {"Output_VA", 11, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Output_VA")},
    {"AC_Charger_Watts", 13, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Charger_Watts")},
    {"AC_Charger_VA", 15, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Charger_VA")},
    {"Battery_Voltage", 17, 0, 0.0, RunningAverage(avSamples), 0.01, Point("Battery_Voltage")},
    {"Battery_SOC", 18, 0, 0.0, RunningAverage(avSamples), 1.0, Point("Battery_SOC")},
    {"Bus_Voltage", 19, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Bus_Voltage")},
    {"AC_In_Voltage", 20, 0, 0.0, RunningAverage(avSamples), 0.1, Point("AC_In_Voltage")},
    {"AC_In_Freq", 21, 0, 0.0, RunningAverage(avSamples), 0.01, Point("AC_In_Freq")},
    {"AC_Out_Voltage", 22, 0, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Out_Voltage")},
    {"AC_Out_Freq", 23, 0, 0.0, RunningAverage(avSamples), 0.01, Point("AC_Out_Freq")},
    {"Inverter_Temp", 25, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Inverter_Temp")},
    {"DC_to_DC_Converter_Temp", 26, 0, 0.0, RunningAverage(avSamples), 0.1, Point("DC_to_DC_Converter_Temp")},
    {"Load_Percentage", 27, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Load_Percentage")},
    {"Buck_Converter_Temp", 32, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Buck_Converter_Temp")},
    {"Output_Current", 34, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Output_Current")},
    {"Inverter_Current", 35, 0, 0.0, RunningAverage(avSamples), 0.1, Point("Inverter_Current")},
    {"AC_In_Watts", 36, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_In_Watts")},
    {"AC_In_VA", 38, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_In_VA")},
    {"PV_Energy_Today", 48, 1, 0.0, RunningAverage(avSamples), 0.1, Point("PV_Energy_Today")},
    {"PV_Energy_Total", 50, 1, 0.0, RunningAverage(avSamples), 0.1, Point("PV_Energy_Total")},
    {"AC_Charger_Today", 56, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Charger_Today")},
    {"AC_Charger_Total", 58, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Charger_Total")},
    {"Battery_Discharge_Today", 60, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Battery_Discharge_Today")},
    {"Battery_Discharge_Total", 62, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Battery_Discharge_Total")},
    {"AC_Charger_Battery_Current", 68, 0, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Charger_Battery_Current")},
    {"AC_Discharge_Watts", 69, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Discharge_Watts")},
    {"AC_Discharge_VA", 71, 1, 0.0, RunningAverage(avSamples), 0.1, Point("AC_Discharge_VA")},
    {"Battery_Discharge_Watts", 73, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Battery_Discharge_Watts")},
    {"Battery_Discharge_VA", 75, 1, 0.0, RunningAverage(avSamples), 0.1, Point("Battery_Discharge_VA")},
    {"Battery_Watts", 77, 2, 0.0, RunningAverage(avSamples), 0.1, Point("Battery_Watts")}, //This is a signed INT32
    {"Inverter_Fan_Speed", 82, 0, 0.0, RunningAverage(avSamples), 1.0, Point("Inverter_Fan_Speed")},
    {"MPPT_Fan_Speed", 83, 0, 0.0, RunningAverage(avSamples), 1.0, Point("MPPT_Fan_Speed")}
  };
#else
  struct stats{
    const char *name;
    const byte address;
    const byte type; //Whether the result is 16 or 32 bit number.
    // float value;
    const float multiplier;
  };

  static constexpr stats arrstats[] = {
    //Register name, MODBUS address, integer type (0 = uint16_t​, 1 = uint32_t​, 2 = int32_t​), multiplier)
    {"System_Status", 0, 0, 1.0},
    {"PV_Voltage", 1, 0, 0.1},
    {"PV_Power", 3, 1, 0.1},
    {"Buck_Converter_Current", 7, 0, 0.1},
    {"Output_Watts", 9, 1, 0.1},
    {"Output_VA", 11, 1, 0.1},
    {"AC_Charger_Watts", 13, 1, 0.1},
    {"AC_Charger_VA", 15, 1, 0.1},
    {"Battery_Voltage", 17, 0, 0.01},
    {"Battery_SOC", 18, 0, 1.0},
    {"Bus_Voltage", 19, 0, 0.1},
    {"AC_In_Voltage", 20, 0, 0.1},
    {"AC_In_Freq", 21, 0, 0.01},
    {"AC_Out_Voltage", 22, 0, 0.1},
    {"AC_Out_Freq", 23, 0, 0.01},
    {"Inverter_Temp", 25, 0, 0.1},
    {"DC_to_DC_Conv_Temp", 26, 0, 0.1},
    {"Load_Percentage", 27, 0, 0.1},
    {"Buck_Converter_Temp", 32, 0, 0.1},
    {"Output_Current", 34, 0, 0.1},
    {"Inverter_Current", 35, 0, 0.1},
    {"AC_In_Watts", 36, 1, 0.1},
    {"AC_In_VA", 38, 1, 0.1},
    {"PV_Energy_Today", 48, 1, 0.1},
    {"PV_Energy_Total", 50, 1, 0.1},
    {"AC_Charger_Today", 56, 1, 0.1},
    {"AC_Charger_Total", 58, 1, 0.1},
    {"Battery_Discharge_Today", 60, 1, 0.1},
    {"Battery_Discharge_Total", 62, 1, 0.1},
    {"AC_Charger_Battery_Current", 68, 0, 0.1},
    {"AC_Discharge_Watts", 69, 1, 0.1},
    {"AC_Discharge_VA", 71, 1, 0.1},
    {"Battery_Discharge_Watts", 73, 1, 0.1},
    {"Battery_Discharge_VA", 75, 1, 0.1},
    {"Battery_Watts", 77, 2, 0.1},
    {"Inverter_Fan_Speed", 82, 0, 1.0},
    {"MPPT_Fan_Speed", 83, 0, 1.0}
  };
#endif

#ifdef MQTT
  #include <PubSubClient.h>
  #ifdef verifyCerts
    X509List MQTTcert(MQTT_CA_CERT);
  #endif
  WiFiClientSecure espClient;
  PubSubClient MQTTclient(espClient);
  constexpr char *MQTTclientId = SECRET_MQTT_CLIENT_ID;
  constexpr char *MQTTtopicPrefix = SECRET_MQTT_TOPIC_PREFIX;
  char MQTTtopic[50];
  
  void MQTTcallback(char* topic, byte* payload, unsigned int length) {
    byte received_payload[length];
    memcpy(received_payload, payload, length);
    
    #ifdef serialDebug
      Serial.printf("MQTT message arrived [%s]: %s \r\n", topic, received_payload);
    #endif
    #ifdef telnetDebug
      TelnetStream.printf("MQTT message arrived [%s]: %s \r\n", topic, received_payload);
    #endif

    for (byte i = 0; i < (sizeof(modBusHoldingRegisters)/sizeof(modBusHoldingRegisters[0])); i++) {
      sprintf(MQTTtopic, "%s/%s/%d_%s", MQTTtopicPrefix, "WriteHoldReg", modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].topic);
      if(strcmp(topic, MQTTtopic) == 0 && modBusHoldingRegisters[i].MQTTsubscribe == 1){

        if(length <= 4){
          for (byte i = 0; i < length; i++){
            if(received_payload[i]>47&&received_payload[i]<58){
              received_payload[i]-=48;
            }else if(received_payload[i]>64&&received_payload[i]<71){
              received_payload[i]-=55;
            }else if(received_payload[i]>96&&received_payload[i]<103){
              received_payload[i]-=87;
            }else{
              return;
            };
          };
        }else{
          return;
        };

        word fullPayload = 0; // Max value is 65535 / 0xFFFF
        for (byte i = 0; i < length; i++){
          fullPayload = (fullPayload<<4) + received_payload[i];
        };

        MODBUSresult = Growatt.writeSingleRegister(modBusHoldingRegisters[i].address, fullPayload);
        #ifdef serialDebug
          Serial.printf("MODBUS write: %s %d 0x%X\r\n", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address, fullPayload);
          if (MODBUSresult != Growatt.ku8MBSuccess) {
            Serial.printf("MODBUS write failed: %d \r\n", MODBUSresult);
          }
        #endif
        #ifdef telnetDebug
          TelnetStream.printf("MODBUS write: %s %d 0x%X\r\n", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address, fullPayload);
          if (MODBUSresult != Growatt.ku8MBSuccess) {
            TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
          }
        #endif
        return;
      }
    }
  }
#endif

void setup()
{
  Serial.begin(9600);
  Serial.flush(); //Make sure the hardware serial buffer is empty before communicating over MODBUS.
  #ifdef telnetDebug
    // Telnet log is accessible at port 23
    TelnetStream.begin();
  #endif

  // ****Start ESP8266 OTA and Wifi Configuration****
  #ifdef serialDebug
    Serial.println();
    Serial.print("Connecting to "); Serial.println(SECRET_SSID);
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS); //Edit include/secrets.h to update this data.

  unsigned long connectTime = millis();
  #ifdef serialDebug
    Serial.print("Waiting for WiFi to connect");
  #endif
  while (!WiFi.isConnected() && (unsigned long)(millis() - connectTime) < 7500) { //Wait for the wifi to connect for up to 7.5 seconds.
    delay(100);
    #ifdef serialDebug
      Serial.print(".");
    #endif
  }
  if (!WiFi.isConnected()) {
    #ifdef serialDebug
      Serial.println();
      Serial.println("WiFi didn't connect, restarting...");
      delay(10);
    #endif
    ESP.restart(); //Restart if the WiFi still hasn't connected.
  }
  #ifdef serialDebug
    Serial.println();
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
  #endif

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[MAC]
  // ArduinoOTA.setHostname("esp8266-Growatt-monitor");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    #ifdef serialDebug
      Serial.println("\nStart updating");
    #endif
    #ifdef telnetDebug
      TelnetStream.println("\nStart updating");
    #endif
  });
  ArduinoOTA.onEnd([]() {
    #ifdef serialDebug
      Serial.println("\nEnd");
    #endif
    #ifdef telnetDebug
      TelnetStream.println("\nEnd");
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #ifdef serialDebug
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
    #ifdef telnetDebug
      TelnetStream.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });
  ArduinoOTA.onError([](ota_error_t error) {
    #ifdef serialDebug
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    #endif
    #ifdef telnetDebug
      TelnetStream.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        TelnetStream.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        TelnetStream.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        TelnetStream.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        TelnetStream.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        TelnetStream.println("End Failed");
      }
    #endif
  });
  ArduinoOTA.begin();

  unsigned long otaTime = millis();
  // Serial.print("Wait 4 upd");
  // byte OTAiteration = 0;
  // byte OTActdn = 4;
  while (WiFi.isConnected() && (unsigned long)(millis() - otaTime) < 5000) { // Failsafe, wait 5 seconds before proceeding
    ArduinoOTA.handle();
    // if (((millis() - otaTime) / 1000) > OTAiteration){
    //   OTAiteration++;
    //   Serial.print(" "); Serial.print(OTActdn);
    //   OTActdn--;
    // }
    yield();
  }
  #ifdef serialDebug
    Serial.println("\r\nNo update received");
  #endif
  // ****End ESP8266 OTA and Wifi Configuration****

  // Growatt Device ID 1
  Growatt.begin(1, Serial);

  #if defined(InfluxDb) || defined(MQTT)
    #ifdef verifyCerts
        // BearSSL::CertStore certStore;

        // Accurate time is necessary for certificate validation
        // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
        // timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
        configTzTime(TZ_INFO, TIME_NTP_SRVR);

        // Wait till time is synced
        #ifdef serialDebug
            Serial.print("Syncing time");
        #endif
        #ifdef telnetDebug
          TelnetStream.print("Syncing time");
        #endif
        int i = 0;
        while (time(nullptr) < 1000000000l && i < 40) {
          #ifdef serialDebug
            Serial.print(".");
          #endif
          #ifdef telnetDebug
            TelnetStream.print(".");
          #endif
          delay(500);
          i++;
        }
        #ifdef serialDebug
          Serial.println();
        #endif
        #ifdef telnetDebug
          TelnetStream.println();
        #endif

        // Time
        time_t now = time(nullptr);
        const struct tm *tm = localtime(&now);

        #ifdef serialDebug
          Serial.print("Synchronized time: ");
          Serial.println(ctime(&now));
        #endif
        #ifdef telnetDebug
          TelnetStream.print("Synchronized time: ");
          TelnetStream.println(ctime(&now));
        #endif

        // Serial.flush(); //Make sure the hardware serial buffer is empty before communicating over MODBUS.
        // MODBUSresult = Growatt.writeSingleRegister(45, (tm->tm_year-100));
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 45, (tm->tm_year-100));
        //   }
        // #endif
        // MODBUSresult = Growatt.writeSingleRegister(46, tm->tm_mon);
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 46, tm->tm_mon);
        //   }
        // #endif
        // MODBUSresult = Growatt.writeSingleRegister(47, tm->tm_mday);
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 47, tm->tm_mday);
        //   }
        // #endif
        // MODBUSresult = Growatt.writeSingleRegister(48, tm->tm_hour);
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 48, tm->tm_hour);
        //   }
        // #endif
        // MODBUSresult = Growatt.writeSingleRegister(49, tm->tm_min);
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 49, tm->tm_min);
        //   }
        // #endif
        // MODBUSresult = Growatt.writeSingleRegister(50, tm->tm_sec);
        // #ifdef telnetDebug
        //   if (MODBUSresult != Growatt.ku8MBSuccess) {
        //     TelnetStream.printf("MODBUS write failed: %d \r\n", MODBUSresult);
        //   }else{
        //     TelnetStream.printf("MODBUS write: %d 0x%X\r\n", 50, tm->tm_sec);
        //   }
        // #endif

    #else
        #ifdef InfluxDB
          // Alternatly you can disable certificate validation
          InfluxDBclient.setInsecure();
        #endif
    #endif
  #endif

  #ifdef MQTT
    #ifdef verifyCerts
      espClient.setTrustAnchors(&MQTTcert);
    #else
      espClient.setInsecure();
    #endif
    // BearSSL::Session session; // Seems to cause intermitent crashes
    // espClient.setSession(&session); // Seems to cause intermitent crashes
    // espClient.setSSLVersion(BR_TLS11,BR_TLS11);
    MQTTclient.setServer(SECRET_MQTT_SERVER, SECRET_MQTT_PORT);
    MQTTclient.setCallback(MQTTcallback);
  #endif

  #ifdef InfluxDb
    InfluxDBclient.setStreamWrite(true);

    // Check the InfluxDB connection
    if (InfluxDBclient.validateConnection()) {
      #ifdef serialDebug
        Serial.print("Connected to InfluxDB: ");
        Serial.println(InfluxDBclient.getServerUrl());
      #endif
      #ifdef telnetDebug
        TelnetStream.print("Connected to InfluxDB: ");
        TelnetStream.println(InfluxDBclient.getServerUrl());
      #endif
    } else {
      #ifdef serialDebug
        Serial.print("InfluxDB connection failed: ");
        Serial.println(InfluxDBclient.getLastErrorMessage());
      #endif
      #ifdef telnetDebug
        TelnetStream.print("InfluxDB connection failed: ");
        TelnetStream.println(InfluxDBclient.getLastErrorMessage());
      #endif
      failures++;
    }
    #ifdef telnetDebug
      TelnetStream.print(InfluxDBclient.isConnected());
    #endif
    #ifdef serialDebug
      Serial.print(InfluxDBclient.isConnected());
    #endif
  #endif
}

void readMODBUSinputRegisters() {
  float value = 0;
  Growatt.clearResponseBuffer();

  for (byte i = 0; i < (sizeof(arrstats)/sizeof(arrstats[0])); i++) {  //Iterate through each of the MODBUS queries and obtain their values.
    ArduinoOTA.handle();
    #ifdef MQTT
      MQTTclient.loop();
    #endif

    MODBUSresult = Growatt.readInputRegisters(arrstats[i].address, 2); //Query each of the MODBUS registers.
    if (MODBUSresult == Growatt.ku8MBSuccess) {
      if (failures >= 1) {
        failures--; //Decrement the failure counter if we've received a response.
      }

      switch (arrstats[i].type) {
        case 0:
          // #ifdef telnetDebug
          //   TelnetStream.print("Raw MODBUS for address: "); TelnetStream.print(arrstats[i].address); TelnetStream.print(": "); TelnetStream.println(Growatt.getResponseBuffer(0));
          // #endif
          value = (Growatt.getResponseBuffer(0) * arrstats[i].multiplier); //Calculatge the actual value.
          break;
        case 1:
          value = ((Growatt.getResponseBuffer(0) << 8) | Growatt.getResponseBuffer(1)) * arrstats[i].multiplier;  //Calculatge the actual value.
          break;
        case 2: //Signed INT32
          value = (Growatt.getResponseBuffer(1) + (Growatt.getResponseBuffer(0) << 16)) * arrstats[i].multiplier;  //Calculatge the actual value.
          break;
        // default:
        //   // if nothing else matches, do the default
        //   // default is optional
        //   break;
      }

      // if (arrstats[i].address == 69) {
      //   if (arrstats[i].value > 6000) { //AC_Discharge_Watts will return very large, invalid results when the inverter has been in stanby mode. Ignore the result if the number is greater than 6kW.
      //     arrstats[i].value = 0;
      //   }
      // }
      // #ifdef serialDebug
      //   Serial.printf("inputReg '%s' address '%d' data: ", arrstats[i].name, arrstats[i].address); Serial.println(arrstats[i].value);
      // #endif
      // #ifdef telnetDebug
      //   TelnetStream.printf("inputReg '%s' address '%d' data: 0x%X\r\n", arrstats[i].name, arrstats[i].address, Growatt.getResponseBuffer(0));
      //   // TelnetStream.printf("inputReg '%s' address '%d' data: ", arrstats[i].name, arrstats[i].address); TelnetStream.println(arrstats[i].value);
      // #endif
      #ifdef InfluxDb      
        arrstats[i].average.addValue(arrstats[i].value); //Add the value to the running average.
        #ifdef serialDebug
          Serial.print("Values collected: "); Serial.println(arrstats[i].average.getCount());
        #endif
        #ifdef telnetDebug
          TelnetStream.print("Values collected: "); TelnetStream.println(arrstats[i].average.getCount());
        #endif
        if (arrstats[i].average.getCount() >= avSamples) { //If we have enough samples added to the running average, send the data and clear the average.
          arrstats[i].measurement.clearFields();
          arrstats[i].measurement.clearTags();
          arrstats[i].measurement.addTag("sensor", "Growatt-SPF");
          arrstats[i].measurement.addField("value", arrstats[i].average.getAverage());

          #ifdef serialDebug
            Serial.print("Sending data to InfluxDB: ");
            Serial.println(InfluxDBclient.pointToLineProtocol(arrstats[i].measurement));
          #endif
          #ifdef telnetDebug
            TelnetStream.print("Sending data to InfluxDB: ");
            TelnetStream.println(InfluxDBclient.pointToLineProtocol(arrstats[i].measurement));
          #endif

          if (!InfluxDBclient.writePoint(arrstats[i].measurement)) { //Write the data point to InfluxDB
            failures++;
            #ifdef serialDebug
              Serial.print("InfluxDB write failed: ");
              Serial.println(InfluxDBclient.getLastErrorMessage());
            #endif
            #ifdef telnetDebug
              TelnetStream.print("InfluxDB write failed: ");
              TelnetStream.println(InfluxDBclient.getLastErrorMessage());
            #endif
          }

          else if (failures >= 1) failures --;
          arrstats[i].average.clear();
        }
      #endif

      #ifdef MQTT
        sprintf(MQTTtopic, "%s/%s/%d_%s", MQTTtopicPrefix, "InputReg", arrstats[i].address, arrstats[i].name);
        if (MQTTclient.connect(MQTTclientId, SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
          char statString[8];
          dtostrf(value, 1, 2, statString);
          #ifdef serialDebug
            Serial.printf("MQTT post '%s' '%s'\r\n", MQTTtopic, statString);
          #endif
          #ifdef telnetDebug
            TelnetStream.printf("MQTT post '%s' '%s'\r\n", MQTTtopic, statString);
          #endif
          MQTTclient.publish(MQTTtopic, statString, (bool)1);
          // MQTTclient.disconnect();

          if (failures >= 1) failures--;
        } else {
          #ifdef serialDebug
            Serial.print("MQTT connection failed: "); Serial.println(MQTTclient.state());
          #endif
          #ifdef telnetDebug
            TelnetStream.print("MQTT connection failed: "); TelnetStream.println(MQTTclient.state());
          #endif
          failures++;
        }
      #endif
    } else {
      #ifdef serialDebug
        Serial.print("MODBUS read failed. Returned value: "); Serial.println(MODBUSresult);
      #endif
      #ifdef telnetDebug
        TelnetStream.print("MODBUS read failed. Returned value: "); TelnetStream.println(MODBUSresult);
      #endif
      failures++;
      #ifdef serialDebug
        Serial.print("Failure counter: "); Serial.println(failures);
      #endif
      #ifdef telnetDebug
        TelnetStream.print("Failure counter: "); TelnetStream.println(failures);
      #endif
    }
    yield();
  }
}

void readMODBUSholdingRegisters() {
  uint64_t num = 0;
  char MQTTdata[41];
  char tmp[4];

  for (byte i = 0; i < (sizeof(modBusHoldingRegisters)/sizeof(modBusHoldingRegisters[0])); i++) {  //Iterate through each of the MODBUS queries and obtain their values.
    ArduinoOTA.handle();
    #ifdef MQTT
      MQTTclient.loop();
    #endif
    Growatt.clearResponseBuffer();

    if(modBusHoldingRegisters[i].length){
      MODBUSresult = Growatt.readHoldingRegisters(modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].length);
      if (MODBUSresult == Growatt.ku8MBSuccess) {
        // char MQTTdata[(modBusHoldingRegisters[i].length*5)+1];
        // char MQTTdata[41];
        // char tmp[4];
        for (byte i = 0; i < (sizeof(MQTTdata)/sizeof(MQTTdata[0])); i++){
          MQTTdata[i] = '\0';
        }
        num = 0;

        switch (modBusHoldingRegisters[i].type) {
          case 0:
          // 0 = word (uint16-t) output as 4 CHARs (i.e. 0xFA03)
            for (byte j = 0; j < modBusHoldingRegisters[i].length; j++){
              sprintf(tmp, "%X", Growatt.getResponseBuffer(j));
              sprintf(MQTTdata, "%s%s", MQTTdata, tmp);
            }
            break;
          case 1:
          // 1 = unsigned long long
            for (byte j = 0; j < modBusHoldingRegisters[i].length; j++){
              num = (num<<16)+Growatt.getResponseBuffer(j);
            }
            sprintf(MQTTdata, "%llu", num);
            break;
          case 2:
          // 2 = ASCII output as up to 2 CHARs (i.e. ZP)
            for (byte j = 0; j < modBusHoldingRegisters[i].length; j++){
              byte nip[3] = {(Growatt.getResponseBuffer(j)>>8),Growatt.getResponseBuffer(j)};
              sprintf(tmp, "%s", nip);
              sprintf(MQTTdata, "%s%s", MQTTdata, tmp);
            }
            break;
          // default:
          //   // if nothing else matches, do the default
          //   // default is optional
          //   break;
        }

        // #ifdef telnetDebug
        //   TelnetStream.printf("holdingReg '%s' address '%d' length '%d' data: %s\r\n", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].length, MQTTdata);
        // #endif
        // #ifdef serialDebug
        //   Serial.printf("holdingReg '%s' address '%d' length '%d' data: %s\r\n", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].length, MQTTdata);
        // #endif

        #ifdef MQTT
          sprintf(MQTTtopic, "%s/%s/%d_%s", MQTTtopicPrefix, "HoldReg", modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].topic);
          if (MQTTclient.connected()) {
            #ifdef serialDebug
              Serial.printf("MQTT post '%s' '%s'\r\n", MQTTtopic, MQTTdata);
            #endif
            #ifdef telnetDebug
              TelnetStream.printf("MQTT post '%s' '%s'\r\n", MQTTtopic, MQTTdata);
            #endif
            MQTTclient.publish(MQTTtopic, MQTTdata, (bool)1);
          } else {
            #ifdef serialDebug
              Serial.print("MQTT connection failed: "); Serial.println(MQTTclient.state());
            #endif
            #ifdef telnetDebug
              TelnetStream.print("MQTT connection failed: "); TelnetStream.println(MQTTclient.state());
            #endif
          }
        #endif
      }else{
        #ifdef serialDebug
          Serial.printf("MODBUS read for '%s' address '%d' failed. Returned value: ", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address); Serial.println(MODBUSresult);
        #endif
        #ifdef telnetDebug
          TelnetStream.printf("MODBUS read for '%s' address '%d' failed. Returned value: ", modBusHoldingRegisters[i].topic, modBusHoldingRegisters[i].address); TelnetStream.println(MODBUSresult);
        #endif
      }
    }
    yield();
  }
}

void loop()
{
  time_t tnow;
  ArduinoOTA.handle();
  #ifdef MQTT
    MQTTclient.loop();
  #endif

  if (WiFi.status() != WL_CONNECTED) {
    #ifdef serialDebug
      Serial.println("WiFi disconnected. Attempting to reconnect... ");
    #endif
    failures++;
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(1000);
  }

  if ((millis() - lastUpdate) >= UPDATE_INTERVAL || initialBoot) { //Get a MODBUS reading every 5 seconds.
    #ifdef MQTT
      if (!MQTTclient.connected()) {
        #ifdef serialDebug
          Serial.println("MQTT disconnected. Attempting to connect...");
        #endif
        #ifdef telnetDebug
          TelnetStream.println("MQTT disconnected. Attempting to connect...");
        #endif
        if (MQTTclient.connect(MQTTclientId, SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
          if (failures >= 1) failures--;
          #ifdef serialDebug
            Serial.println("MQTT Connected.");
          #endif          
          #ifdef telnetDebug
            TelnetStream.println("MQTT Connected.");
          #endif
          if (initialBoot){
            initialBoot = 0;
            sprintf(MQTTtopic, "%s/%s", MQTTtopicPrefix, "Shine_Boot_Time");
            // Show time
            tnow = time(nullptr);
            MQTTclient.publish(MQTTtopic, ctime(&tnow), (bool)1);
          }
          for (byte i = 0; i < (sizeof(modBusHoldingRegisters)/sizeof(modBusHoldingRegisters[0])); i++) {
            if (modBusHoldingRegisters[i].MQTTsubscribe == 1){
              sprintf(MQTTtopic, "%s/%s/%d_%s", MQTTtopicPrefix, "WriteHoldReg", modBusHoldingRegisters[i].address, modBusHoldingRegisters[i].topic);
              MQTTclient.subscribe(MQTTtopic);
              #ifdef serialDebug
                Serial.printf("MQTT Subscribing to: [%s]\r\n", MQTTtopic);
              #endif
              #ifdef telnetDebug
                TelnetStream.printf("MQTT Subscribing to: [%s]\r\n", MQTTtopic);
              #endif
            }
          }
        } else {
          #ifdef serialDebug
            Serial.print("MQTT connection failed: "); Serial.println(MQTTclient.state());
          #endif
          #ifdef telnetDebug
            TelnetStream.print("MQTT connection failed: "); TelnetStream.println(MQTTclient.state());
          #endif
          unsigned long tstTime = millis();
          while ((unsigned long)(millis() - tstTime) < 5000) { // Failsafe, wait 5 seconds before proceeding
            ArduinoOTA.handle();
            #ifdef serialDebug
              Serial.printf("Rebooting: %lu%%\r", (unsigned long)(millis() - tstTime) / 50);
            #endif
            #ifdef telnetDebug
              TelnetStream.printf("Rebooting: %lu%%\r", (unsigned long)(millis() - tstTime) / 50);
            #endif
          }
          ESP.restart();
        }
      }
    #endif
    #if defined(serialDebug) || defined(telnetDebug)
      // Show time
      tnow = time(nullptr);
    #endif
    #ifdef serialDebug
      Serial.print("Time: ");
      Serial.print(ctime(&tnow));
      Serial.printf("\r\n\r\nWiFi signal strength is: %d\r\n", WiFi.RSSI());
      Serial.println("Reading the MODBUS...");
    #endif
    #ifdef telnetDebug
      TelnetStream.print("Time: ");
      TelnetStream.print(ctime(&tnow));
      TelnetStream.printf("\r\n\r\nWiFi signal strength is: %d\r\n", WiFi.RSSI());
      TelnetStream.println("Reading the MODBUS...");
    #endif
    readMODBUSholdingRegisters();
    readMODBUSinputRegisters();
    lastUpdate = millis();
  }

  if (failures >= FAILURE_COUNT) {  //Reboot the ESP if there's been too many problems retrieving or sending the data.
    #ifdef serialDebug
      Serial.print("Failure counter has reached: "); Serial.print(failures); Serial.println(". Rebooting...");
    #endif
    #ifdef telnetDebug
      TelnetStream.print("Failure counter has reached: "); TelnetStream.print(failures); TelnetStream.println(". Rebooting...");
    #endif
    ESP.restart();
  }
  yield();
}