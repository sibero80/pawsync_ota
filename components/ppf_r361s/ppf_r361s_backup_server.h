#pragma once

#include <cstdint>
#include <cstddef>
#include "esphome/core/log.h"

#if defined(USE_ESP_IDF)
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace esphome {
namespace ppf_r361s {

class PPFR361SComponent;

class PPFR361SBackupServer {
 public:
  explicit PPFR361SBackupServer(PPFR361SComponent *parent);
  ~PPFR361SBackupServer();

  void start();
  void stop();
  bool is_running() const { return this->server_handle_ != nullptr; }

 private:
  PPFR361SComponent *parent_{nullptr};
  void *server_handle_{nullptr};
};

}  // namespace ppf_r361s
}  // namespace esphome
