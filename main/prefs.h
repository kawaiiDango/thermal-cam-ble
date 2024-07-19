#ifndef H_PREFERENCES_
#define H_PREFERENCES_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


    struct MyPreferences
    {
        uint8_t refreshRate;
        uint8_t lastResetReason;
    };

    extern struct MyPreferences prefs;

    void initFromPrefs();
    void savePrefs();

#ifdef __cplusplus
}
#endif

#endif
