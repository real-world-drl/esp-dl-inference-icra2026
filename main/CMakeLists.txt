# `main` component build description.
#
# The .espdl model is embedded as flash-resident read-only data via
# target_add_aligned_binary_data().  Drop your model in main/models/model.espdl
# (or change the path here) before building.

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)

set(embed_files models/model.espdl)

idf_component_register(
    SRCS
        "main.cpp"
        "src/inference_handler.cpp"
        "src/led_blink.c"
        "src/mqtt_handler.c"
        "src/mqtt_parsers.c"
        "src/observations.c"
        "src/observation_task.c"
        "src/model_variant.cpp"
    INCLUDE_DIRS
        "include"
    PRIV_REQUIRES
        spi_flash
        esp-dl
        driver
        esp_wifi
        mqtt
        nvs_flash
)

target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)
