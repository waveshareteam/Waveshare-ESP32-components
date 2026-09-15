#ifndef TEST_STUB_ESP_LOG_H
#define TEST_STUB_ESP_LOG_H

#include <stdio.h>

#define ESP_LOGE(tag, ...) do { (void)(tag); } while (0)
#define ESP_LOGI(tag, ...) do { (void)(tag); } while (0)

#endif
