
#include "freematics_plus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace freematics {

static const char *TAG = "freematics.sensor";

void FreematicsPlus::dump_config() {
  ESP_LOGCONFIG(TAG, "FreematicsPlus:");
  LOG_SENSOR("", "FreematicsPlus", this);
}

void FreematicsPlus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up FreematicsPlus...");

  bool obd_detected = false;

  // for (int i = 0; i < 3; i++) {
  //   uint8_t version = obd.begin();
  //   if (version > 0) {
  //     ESP_LOGD(TAG, "Detected. OBD firmware version %d.%d", version / 10, version % 10);
  //     obd_detected = true;
  //     break;
  //   }
  //   delay(100);
  // }

  if (!obd_detected) {
    ESP_LOGD(TAG, "Not detected!");
    this->mark_failed();
  }

  // static const char cmds[][6] = {"ATZ\r", "ATI\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
  // char buf[128];

  // for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
  //   const char *cmd = cmds[i];
  //   mySerial.print("Sending ");
  //   mySerial.println(cmd);
  //   if (obd.sendCommand(cmd, buf, sizeof(buf))) {
  //     char *p = strstr(buf, cmd);
  //     if (p)
  //       p += strlen(cmd);
  //     else
  //       p = buf;
  //     while (*p == '\r')
  //       p++;
  //     while (*p) {
  //       mySerial.write(*p);
  //       if (*p == '\r' && *(p + 1) != '\r')
  //         mySerial.write('\n');
  //       p++;
  //     }
  //     mySerial.println();
  //   } else {
  //     mySerial.println("Timeout");
  //   }
  //   delay(1000);
  // }
  // mySerial.println();

  ESP_LOGCONFIG(TAG, "Setup done.");
}

void FreematicsPlus::update() {}

}  // namespace freematics
}  // namespace esphome
