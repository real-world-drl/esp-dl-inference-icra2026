# Smoke-test app component.  Embeds the same model.espdl as the main firmware,
# but uses the model_variant pure helper from the parent main/src tree so the
# auto-detection logic is exercised by the same code path.

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)

set(embed_files ../../main/models/model.espdl)

idf_component_register(
    SRCS
        "smoke_test.cpp"
        "../../main/src/model_variant.cpp"
    INCLUDE_DIRS
        "../../main/include"
    PRIV_REQUIRES
        spi_flash
        esp-dl
)

target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)
