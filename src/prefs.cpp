#include <prefs.h>
#include <Preferences.h>

#define PREF_REFRESH_RATE "refreshRate"
#define PREF_LAST_RESET_REASON "lastResetReason"

struct MyPreferences prefs;

void initFromPrefs()
{
    Preferences preferences;

    preferences.begin("preferences");
    prefs.refreshRate = preferences.getFloat(PREF_REFRESH_RATE, 2);
    prefs.lastResetReason = preferences.getUInt(PREF_LAST_RESET_REASON, 0);
    preferences.end();
}

void savePrefs()
{
    ESP_LOGI(TAG_PREFS, "Saving preferences");
    Preferences preferences;

    preferences.begin("preferences");
    preferences.putUInt(PREF_REFRESH_RATE, prefs.refreshRate);
    preferences.end();
}