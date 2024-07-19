#include <prefs.h>
#include "nvs_flash.h"
#include "nvs.h"

#define TAG "prefs"
#define PREF_REFRESH_RATE "refreshRate"
#define PREF_LAST_RESET_REASON "lastResetReason"

struct MyPreferences prefs;

void initFromPrefs()
{
    // put default values
    prefs.lastResetReason = 0;
    prefs.refreshRate = 2;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("prefs", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        // Read
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(handle, PREF_REFRESH_RATE, &prefs.refreshRate));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(handle, PREF_LAST_RESET_REASON, &prefs.lastResetReason));
    }
}

void savePrefs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("prefs", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        // Write
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(handle, PREF_REFRESH_RATE, prefs.refreshRate));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(handle, PREF_LAST_RESET_REASON, prefs.lastResetReason));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(handle));
        nvs_close(handle);
    }
}