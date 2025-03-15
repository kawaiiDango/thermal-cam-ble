cmake_minimum_required(VERSION 3.16.0)

idf_component_register(SRCS "gap_svr.c" "adc_stuff.c" "ble_host.c" "gatt_svr.c" "MLX90640_API.c" "MLX90640_I2C_Driver.c" "prefs.c" "Adafruit_MLX90640.c" "main.c" 
                    INCLUDE_DIRS ".")

add_custom_command(OUTPUT "patched1.txt"
                   COMMAND patch --forward --reject-file=- ${IDF_PATH}/components/bt/host/nimble/nimble/nimble/host/store/config/include/store/config/ble_store_config.h ${CMAKE_CURRENT_SOURCE_DIR}/../ble_store_config.h.patch || true
                   )
add_custom_target(patch1 DEPENDS patched1.txt)
add_dependencies(${COMPONENT_LIB} patch1)


add_custom_command(OUTPUT "patched2.txt"
                   COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/../mlx90640-library/**/*.{h,c} ${CMAKE_CURRENT_SOURCE_DIR}/ && patch --forward --reject-file=- ${CMAKE_CURRENT_SOURCE_DIR}/MLX90640_API.h ${CMAKE_CURRENT_SOURCE_DIR}/../mlx_lib.patch || true
                   )
add_custom_target(patch2 DEPENDS patched2.txt)
add_dependencies(${COMPONENT_LIB} patch2)