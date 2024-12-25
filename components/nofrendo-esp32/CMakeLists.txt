idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    PRIV_REQUIRES nofrendo driver esp_timer nvs_flash spi_flash
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-xor-used-as-pow)