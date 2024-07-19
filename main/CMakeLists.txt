cmake_minimum_required(VERSION 3.16.0)

idf_component_register(SRCS "gap_svr.c" "adc_stuff.c" "ble_host.c" "gatt_svr.c" "./mlx90640-library/functions/MLX90640_API.c" "MLX90640_I2C_Driver.c" "prefs.c" "Adafruit_MLX90640.c" "main.c" 
                    INCLUDE_DIRS "." "./mlx90640-library/headers")

add_custom_command(OUTPUT "patched.txt"
                   COMMAND patch --forward --reject-file=- ${IDF_PATH}/components/bt/host/nimble/nimble/nimble/host/store/config/include/store/config/ble_store_config.h ${CMAKE_CURRENT_SOURCE_DIR}/../ble_store_config.h.patch || true
                   )
add_custom_target(patch DEPENDS patched.txt)
add_dependencies(${COMPONENT_LIB} patch)
