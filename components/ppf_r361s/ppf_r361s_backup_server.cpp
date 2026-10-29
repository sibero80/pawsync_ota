#include "ppf_r361s_backup_server.h"
#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s.backup_server";

#if defined(USE_ESP_IDF)

static esp_err_t http_export_status_handler(httpd_req_t *req) {
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  bool intact = false;
  const char *slot_label = "unknown";
  size_t slot_size = 0;
  char version_buf[32] = "unknown";

  if (other != nullptr) {
    slot_label = other->label;
    slot_size = other->size;
    esp_app_desc_t app_desc;
    if (esp_ota_get_partition_description(other, &app_desc) == ESP_OK) {
      if (strstr(app_desc.project_name, "pawsync-feeder") == nullptr) {
        intact = true;
        strncpy(version_buf, app_desc.version, sizeof(version_buf) - 1);
        version_buf[sizeof(version_buf) - 1] = '\0';
      }
    }
  }

  auto *comp = static_cast<PPFR361SComponent *>(req->user_ctx);
  if (comp != nullptr) {
    comp->set_stock_partition_state(intact ? STATE_STOCK_INTACT : STATE_STOCK_OVERWRITTEN);
  }
  uint8_t stock_state = (comp != nullptr) ? comp->get_stock_partition_state() : static_cast<uint8_t>(intact ? STATE_STOCK_INTACT : STATE_STOCK_OVERWRITTEN);
  bool allow_overwrite = (comp != nullptr) ? comp->is_allow_overwrite_stock() : false;

  const esp_partition_t *fac = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "fac_data");
  size_t fac_size = (fac != nullptr) ? fac->size : 0;

  char resp[300];
  snprintf(resp, sizeof(resp),
           "{\"stock_intact\":%s,\"stock_state\":%d,\"allow_overwrite\":%s,\"slot\":\"%s\",\"slot_size\":%u,\"fac_size\":%u,\"version\":\"%s\"}",
           intact ? "true" : "false", (int)stock_state, allow_overwrite ? "true" : "false", slot_label, (unsigned)slot_size, (unsigned)fac_size, version_buf);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

static esp_err_t http_export_stock_bin_handler(httpd_req_t *req) {
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  if (other == nullptr) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"stock_partition.bin\"");

  const size_t chunk_size = 4096;
  uint8_t *chunk = (uint8_t *) malloc(chunk_size);
  if (chunk == nullptr) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t offset = 0;
  while (offset < other->size) {
    size_t to_read = std::min<size_t>(chunk_size, other->size - offset);
    if (esp_partition_read(other, offset, chunk, to_read) != ESP_OK) {
      free(chunk);
      if (offset == 0) {
        httpd_resp_send_500(req);
      }
      return ESP_FAIL;
    }
    if (httpd_resp_send_chunk(req, (const char *)chunk, to_read) != ESP_OK) {
      free(chunk);
      return ESP_FAIL;
    }
    offset += to_read;
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  free(chunk);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

static esp_err_t http_export_calibration_handler(httpd_req_t *req) {
  const esp_partition_t *fac = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "fac_data");
  if (fac == nullptr) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"fac_calibration.bin\"");

  const size_t chunk_size = 4096;
  uint8_t *chunk = (uint8_t *) malloc(chunk_size);
  if (chunk == nullptr) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t offset = 0;
  while (offset < fac->size) {
    size_t to_read = std::min<size_t>(chunk_size, fac->size - offset);
    if (esp_partition_read(fac, offset, chunk, to_read) != ESP_OK) {
      free(chunk);
      if (offset == 0) {
        httpd_resp_send_500(req);
      }
      return ESP_FAIL;
    }
    if (httpd_resp_send_chunk(req, (const char *)chunk, to_read) != ESP_OK) {
      free(chunk);
      return ESP_FAIL;
    }
    offset += to_read;
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  free(chunk);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}
#endif

PPFR361SBackupServer::PPFR361SBackupServer(PPFR361SComponent *parent) : parent_(parent) {}

PPFR361SBackupServer::~PPFR361SBackupServer() {
  this->stop();
}

void PPFR361SBackupServer::start() {
#if defined(USE_ESP_IDF)
  if (this->server_handle_ != nullptr)
    return;

  // Initialize stock intact flag and NVS state machine by inspecting ground truth
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  esp_app_desc_t app_desc;
  uint8_t current_state = STATE_STOCK_OVERWRITTEN;
  if (other != nullptr && esp_ota_get_partition_description(other, &app_desc) == ESP_OK) {
    bool is_pawsync = (strstr(app_desc.project_name, "pawsync-feeder") != nullptr);
    current_state = is_pawsync ? STATE_STOCK_OVERWRITTEN : STATE_STOCK_INTACT;
  }
  if (this->parent_ != nullptr) {
    this->parent_->set_stock_partition_state(current_state);
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 8088;
  config.ctrl_port = 32769;
  config.max_open_sockets = 1;
  config.lru_purge_enable = true;
  config.stack_size = 4096;

  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) == ESP_OK) {
    this->server_handle_ = (void *)server;
    ESP_LOGI(TAG, "Factory backup HTTP streaming server listening on port %d", config.server_port);

    httpd_uri_t uri_status = {
      .uri = "/export_status",
      .method = HTTP_GET,
      .handler = http_export_status_handler,
      .user_ctx = this->parent_
    };
    httpd_register_uri_handler(server, &uri_status);

    httpd_uri_t uri_stock = {
      .uri = "/export_stock_bin",
      .method = HTTP_GET,
      .handler = http_export_stock_bin_handler,
      .user_ctx = this->parent_
    };
    httpd_register_uri_handler(server, &uri_stock);

    httpd_uri_t uri_cal = {
      .uri = "/export_calibration",
      .method = HTTP_GET,
      .handler = http_export_calibration_handler,
      .user_ctx = this->parent_
    };
    httpd_register_uri_handler(server, &uri_cal);
  } else {
    ESP_LOGW(TAG, "Failed to start factory backup HTTP server on port 8088");
  }
#endif
}

void PPFR361SBackupServer::stop() {
#if defined(USE_ESP_IDF)
  if (this->server_handle_ != nullptr) {
    httpd_stop((httpd_handle_t)this->server_handle_);
    this->server_handle_ = nullptr;
    ESP_LOGI(TAG, "Factory backup HTTP streaming server stopped.");
  }
#endif
}

}  // namespace ppf_r361s
}  // namespace esphome
