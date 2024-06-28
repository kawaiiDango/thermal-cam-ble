#ifndef H_LED_INDICATOR_
#define H_LED_INDICATOR_

#ifdef __cplusplus
extern "C"
{
#endif

    enum indicator_state
    {
        INDICATOR_MLX_NOT_INITED,
        INDICATOR_NOT_NOTIFYING,
        INDICATOR_BONDING_REQ,
        INDICATOR_NOTIFYING,
        INDICATOR_TRANSFER_SUCCESS,
        INDICATOR_READ_FAILED,
        INDICATOR_TRANSFER_FALED,
    };

    extern enum indicator_state current_indicator_state;

    void indicator_task(void *pvParameter);

#ifdef __cplusplus
}
#endif

#endif
