#ifndef H_LED_INDICATOR_
#define H_LED_INDICATOR_

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum 
    {
        INDICATOR_NOT_NOTIFYING,
        INDICATOR_BONDING_REQ,
        INDICATOR_NOTIFYING,
        INDICATOR_TRANSFER_SUCCESS,
        INDICATOR_READ_FAILED,
        INDICATOR_TRANSFER_FAILED,
    } indicator_state;

    extern indicator_state current_indicator_state;

#ifdef __cplusplus
}
#endif

#endif
