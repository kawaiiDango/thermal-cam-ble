#ifndef H_PREFERENCES_
#define H_PREFERENCES_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define TAG_PREFS "prefs"

    struct MyPreferences
    {
        float refreshRate;
        unsigned int lastResetReason;
    };

    extern struct MyPreferences prefs;

    void initFromPrefs();
    void savePrefs();

#ifdef __cplusplus
}
#endif

#endif
