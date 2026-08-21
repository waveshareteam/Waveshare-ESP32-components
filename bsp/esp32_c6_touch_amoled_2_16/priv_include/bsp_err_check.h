#pragma once

#include "esp_check.h"
#include "sdkconfig.h"

#if CONFIG_BSP_ERROR_CHECK
#define BSP_ERROR_CHECK_RETURN_ERR(x)  ESP_ERROR_CHECK(x)
#define BSP_ERROR_CHECK_RETURN_NULL(x) ESP_ERROR_CHECK(x)
#define BSP_NULL_CHECK(x, ret)         assert(x)
#else
#define BSP_ERROR_CHECK_RETURN_ERR(x) do { \
        const esp_err_t err_rc_ = (x); \
        if (unlikely(err_rc_ != ESP_OK)) { \
            return err_rc_; \
        } \
    } while (0)
#define BSP_ERROR_CHECK_RETURN_NULL(x) do { \
        if (unlikely((x) != ESP_OK)) { \
            return NULL; \
        } \
    } while (0)
#define BSP_NULL_CHECK(x, ret) do { \
        if ((x) == NULL) { \
            return ret; \
        } \
    } while (0)
#endif
