#include "driver/uart.h"
#include "BluetoothSerial.h"
#include <Bounce2.h>

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// ESP32 Pinout Reference: Which GPIO pins should you use?
// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.cpp
#define RTS0 22
#define CTS0 19
#define RXD0 SOC_RX0  // 3
#define TXD0 SOC_TX0  // 1

#if CONFIG_IDF_TARGET_ESP32
#define RTS1 11
#define CTS1 6
#define RXD1 9
#define TXD1 10

#define RTS2 14
#define CTS2 15
#define RXD2 16
#define TXD2 17
#endif

#define USEHWFLOWCTRL

#define PAIR_CMD 36
#define DISCONNECT_CMD 39
#define CONNECTED_IND 32

namespace {

HardwareSerial& rSerial(
#if defined     IO_On_SERIAL_PORT_MONITOR
  Serial
#else
  Serial2
#endif
);
BluetoothSerial SerialBT;
Bounce          PairCmd;
Bounce          DisconnectCmd;

const String slaveName = "ICOM BT(IC-705)";
const String masterName = "Hardplace 705+";

#if 0
esp_spp_api.h
typedef enum {
    ESP_SPP_INIT_EVT                    = 0,                /*!< When SPP is inited, the event comes */
    ESP_SPP_UNINIT_EVT                  = 1,                /*!< When SPP is uninited, the event comes */
    ESP_SPP_DISCOVERY_COMP_EVT          = 8,                /*!< When SDP discovery complete, the event comes */
    ESP_SPP_OPEN_EVT                    = 26,               /*!< When SPP Client connection open, the event comes */
    ESP_SPP_CLOSE_EVT                   = 27,               /*!< When SPP connection closed, the event comes */
    ESP_SPP_START_EVT                   = 28,               /*!< When SPP server started, the event comes */
    ESP_SPP_CL_INIT_EVT                 = 29,               /*!< When SPP client initiated a connection, the event comes */
    ESP_SPP_DATA_IND_EVT                = 30,               /*!< When SPP connection received data, the event comes, only for ESP_SPP_MODE_CB */
    ESP_SPP_CONG_EVT                    = 31,               /*!< When SPP connection congestion status changed, the event comes, only for ESP_SPP_MODE_CB */
    ESP_SPP_WRITE_EVT                   = 33,               /*!< When SPP write operation completes, the event comes, only for ESP_SPP_MODE_CB */
    ESP_SPP_SRV_OPEN_EVT                = 34,               /*!< When SPP Server connection open, the event comes */
    ESP_SPP_SRV_STOP_EVT                = 35,               /*!< When SPP server stopped, the event comes */
    ESP_SPP_VFS_REGISTER_EVT            = 36,               /*!< When SPP VFS register, the event comes */
    ESP_SPP_VFS_UNREGISTER_EVT          = 37,               /*!< When SPP VFS unregister, the event comes */
} esp_spp_cb_event_t;
#endif

// Bluetooth Event Handler CallBack Function Definition esp_ssp_api.h
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_CLOSE_EVT) {
    digitalWrite(CONNECTED_IND, LOW);
  } else if (event == ESP_SPP_OPEN_EVT) {
    digitalWrite(CONNECTED_IND, HIGH);
  }
  // Serial.println(String(event));
  // ESP_SPP_CLOSE_EVT
  // ESP_SPP_DISCOVERY_COMP_EVT, ESP_SPP_CL_INIT_EVT, ESP_SPP_OPEN_EVT
}

// Clear bond device list
void clearBondDeviceList(void) {
  int cDevices(esp_bt_gap_get_bond_device_num());

  if (cDevices && cDevices != ESP_FAIL) {
    esp_bd_addr_t* pairedDeviceBtAddrList(new esp_bd_addr_t[cDevices]);

    if (ESP_OK == esp_bt_gap_get_bond_device_list(&cDevices, pairedDeviceBtAddrList)) {
      for (int nIndex(0); nIndex < cDevices; nIndex++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddrList[nIndex]);
      }
    }
    delete[] pairedDeviceBtAddrList;
  }
}

}

void setup() {
  PairCmd.attach(PAIR_CMD, INPUT_PULLDOWN);
  PairCmd.interval(25);
  DisconnectCmd.attach(DISCONNECT_CMD, INPUT_PULLDOWN);
  DisconnectCmd.interval(25);
  pinMode(CONNECTED_IND, OUTPUT), digitalWrite(CONNECTED_IND, LOW);

#if !defined IO_On_SERIAL_PORT_MONITOR
#if defined USEHWFLOWCTRL
  rSerial.setPins(RXD2, TXD2, CTS2, RTS2);
  rSerial.setHwFlowCtrlMode();
#else
  rSerial.setPins(RXD2, TXD2);
#endif
#endif
  rSerial.begin(115200);

  SerialBT.begin(masterName, true);
  SerialBT.register_callback(BT_EventHandler);

  int cDevices(esp_bt_gap_get_bond_device_num());

  if (cDevices != ESP_FAIL && cDevices > 1) {
    clearBondDeviceList();
    cDevices = esp_bt_gap_get_bond_device_num();
  }

  if (!cDevices || cDevices == ESP_FAIL) {
    // Serial.println("Pairing with slave BT device with name");
    while (!SerialBT.connect(slaveName)) {
      delay(1);
    }
  }
  //ESP.restart();
}

void loop() {
  PairCmd.update();
  if (PairCmd.rose()) {
    if (SerialBT.connected()) {
      SerialBT.disconnect();
    }
    clearBondDeviceList();
  }

  DisconnectCmd.update();
  if (DisconnectCmd.rose()) {
    SerialBT.disconnect();
  }

  if (SerialBT.connected()) {
    while (rSerial.available() > 0) {
      SerialBT.write(rSerial.read());
    }
    while (SerialBT.available() > 0) {
      rSerial.write(SerialBT.read());
    }

  } else {
    while (rSerial.available() > 0) {
      rSerial.read();
    }
    while (SerialBT.available() > 0) {
      SerialBT.read();
    }

    int cDevices(esp_bt_gap_get_bond_device_num());

    if (cDevices && cDevices != ESP_FAIL) {
      esp_bd_addr_t* pairedDeviceBtAddrList(new esp_bd_addr_t[cDevices]);

      if (ESP_OK == esp_bt_gap_get_bond_device_list(&cDevices, pairedDeviceBtAddrList)) {
        SerialBT.connect(pairedDeviceBtAddrList[0]);
      }
      delete[] pairedDeviceBtAddrList;
    } else if (cDevices == 0) {
      SerialBT.connect(slaveName);
    }
  }
}