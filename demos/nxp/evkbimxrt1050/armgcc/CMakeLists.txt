INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

 
# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})


SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -DDEBUG")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -D__STARTUP_INITIALIZE_NONCACHEDATA")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -g")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -Wall")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -fno-common")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -fdata-sections")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -ffreestanding")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -fno-builtin")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -mthumb")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -mapcs")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -std=gnu99")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -mcpu=cortex-m7")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_DEBUG} -mfpu=fpv5-d16")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -DNDEBUG")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -D__STARTUP_INITIALIZE_NONCACHEDATA")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -Wall")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -fno-common")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -fdata-sections")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -ffreestanding")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -fno-builtin")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -mthumb")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -mapcs")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -std=gnu99")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_ASM_FLAGS_FLEXSPI_NOR_RELEASE} -mfpu=fpv5-d16")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DXIP_EXTERNAL_FLASH=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DXIP_BOOT_HEADER_ENABLE=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DDEBUG")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DCPU_MIMXRT1052DVL6B")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DFSL_SDK_ENABLE_DRIVER_CACHE_CONTROL=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DHIGH_SPEED_SDIO_CLOCK=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DUSE_RTOS=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DLWIP_NETIF_API=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DMEMP_NUM_NETCONN=4")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DFSL_RTOS_FREE_RTOS")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -DWIFI_WICED")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -g")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -O0")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -Wall")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -fno-common")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -ffunction-sections")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -fdata-sections")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -ffreestanding")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -fno-builtin")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -mthumb")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -mapcs")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -std=gnu99")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -mcpu=cortex-m7")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -mfpu=fpv5-d16")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -MMD")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -MP")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_C_FLAGS_FLEXSPI_NOR_DEBUG} -fomit-frame-pointer")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DXIP_EXTERNAL_FLASH=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DXIP_BOOT_HEADER_ENABLE=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DNDEBUG")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DCPU_MIMXRT1052DVL6B")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DFSL_SDK_ENABLE_DRIVER_CACHE_CONTROL=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DHIGH_SPEED_SDIO_CLOCK=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DUSE_RTOS=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DLWIP_NETIF_API=1")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DMEMP_NUM_NETCONN=4")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DFSL_RTOS_FREE_RTOS")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -DWIFI_WICED")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -Os")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -Wall")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -fno-common")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -ffunction-sections")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -fdata-sections")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -ffreestanding")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -fno-builtin")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -mthumb")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -mapcs")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -std=gnu99")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -mfpu=fpv5-d16")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -MMD")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -MP")

SET(CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_C_FLAGS_FLEXSPI_NOR_RELEASE} -fomit-frame-pointer")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -DDEBUG")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -g")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -O0")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -Wall")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -fno-common")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -fdata-sections")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -ffreestanding")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -fno-builtin")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -mthumb")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -mapcs")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -fno-rtti")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -fno-exceptions")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -mcpu=cortex-m7")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -mfpu=fpv5-d16")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -MMD")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_DEBUG} -MP")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -DNDEBUG")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -Os")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -Wall")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -fno-common")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -fdata-sections")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -ffreestanding")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -fno-builtin")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -mthumb")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -mapcs")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -fno-rtti")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -fno-exceptions")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -mfpu=fpv5-d16")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -MMD")

SET(CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_CXX_FLAGS_FLEXSPI_NOR_RELEASE} -MP")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -g")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -static")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -z")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Map=output.map")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -mcpu=cortex-m7")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -mfpu=fpv5-d16")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} --defsym=__stack_size__=0x200")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} --defsym=__heap_size__=0x2000")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -static")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -z")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Map=output.map")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mcpu=cortex-m7")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -mfpu=fpv5-d16")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} --defsym=__stack_size__=0x200")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} --defsym=__heap_size__=0x2000")

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/GCC/ARM_CM4F)

include_directories(${ProjDirPath}/../../../common/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk)

include_directories(${ProjDirPath}/../../../../lib/include)

include_directories(${ProjDirPath}/../../../../lib/include/private)

include_directories(${ProjDirPath}/../common)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers)

include_directories(${ProjDirPath}/../common/application_code/nxp_code)

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers)

include_directories(${ProjDirPath}/../../../../lib/third_party/pkcs11)

include_directories(${ProjDirPath}/../../../../lib/third_party/jsmn)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/platforms/NEB1DX_01)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/port)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/port)

include_directories(${ProjDirPath}/../../../../lib/FreeRTOS-Plus-TCP/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/str)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/debug_console)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/uart)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/serial_manager)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/lists)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/xip)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/boards/evkbimxrt1050/xip)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/network/LwIP/WWD)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/LPC)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/LPC/WWD)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/RTOS/FreeRTOS/WWD)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/network)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/RTOS)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/bus_protocols/SDIO)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/inc)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/src)

add_executable(aws_demos.elf 
"${ProjDirPath}/../common/application_code/main.c"
"${ProjDirPath}/../common/lwipopts.h"
"${ProjDirPath}/../../../common/include/aws_application_version.h"
"${ProjDirPath}/../../../common/include/aws_clientcredential.h"
"${ProjDirPath}/../../../common/include/aws_clientcredential_keys.h"
"${ProjDirPath}/../../../common/include/aws_defender_demo.h"
"${ProjDirPath}/../../../common/include/aws_demo.h"
"${ProjDirPath}/../../../common/include/aws_demo_runner.h"
"${ProjDirPath}/../../../common/include/aws_dev_mode_key_provisioning.h"
"${ProjDirPath}/../../../common/include/aws_greengrass_discovery_demo.h"
"${ProjDirPath}/../../../common/include/aws_hello_world.h"
"${ProjDirPath}/../../../common/include/aws_logging_task.h"
"${ProjDirPath}/../../../common/include/aws_ota_codesigner_certificate.h"
"${ProjDirPath}/../../../common/include/aws_ota_update_demo.h"
"${ProjDirPath}/../../../common/include/aws_shadow_lightbulb_on_off.h"
"${ProjDirPath}/../../../common/include/aws_subscribe_publish_loop.h"
"${ProjDirPath}/../../../common/include/aws_tcp_echo_client_single_tasks.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_drv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_drv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_file.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_file.c"
"${ProjDirPath}/../../../common/demo_runner/aws_demo_runner.c"
"${ProjDirPath}/../../../common/devmode_key_provisioning/aws_dev_mode_key_provisioning.c"
"${ProjDirPath}/../../../common/greengrass_connectivity/aws_greengrass_discovery_demo.c"
"${ProjDirPath}/../../../common/mqtt/aws_hello_world.c"
"${ProjDirPath}/../../../common/logging/aws_logging_task_dynamic_buffers.c"
"${ProjDirPath}/../../../common/shadow/aws_shadow_lightbulb_on_off.c"
"${ProjDirPath}/../../../common/mqtt/aws_subscribe_publish_loop.c"
"${ProjDirPath}/../../../common/tcp/aws_tcp_echo_client_single_task.c"
"${ProjDirPath}/../../../common/defender/aws_defender_demo.c"
"${ProjDirPath}/../../../common/tcp/aws_tcp_echo_client_separate_tasks.c"
"${ProjDirPath}/../common/application_code/nxp_code/board.c"
"${ProjDirPath}/../common/application_code/nxp_code/board.h"
"${ProjDirPath}/../common/application_code/nxp_code/pin_mux.c"
"${ProjDirPath}/../common/application_code/nxp_code/pin_mux.h"
"${ProjDirPath}/../common/application_code/nxp_code/clock_config.h"
"${ProjDirPath}/../common/application_code/nxp_code/clock_config.c"
"${ProjDirPath}/../common/config_files/aws_bufferpool_config.h"
"${ProjDirPath}/../common/config_files/aws_demo_config.h"
"${ProjDirPath}/../common/config_files/aws_ggd_config.h"
"${ProjDirPath}/../common/config_files/aws_mqtt_agent_config.h"
"${ProjDirPath}/../common/config_files/aws_mqtt_config.h"
"${ProjDirPath}/../common/config_files/aws_pkcs11_config.h"
"${ProjDirPath}/../common/config_files/aws_secure_sockets_config.h"
"${ProjDirPath}/../common/config_files/aws_shadow_config.h"
"${ProjDirPath}/../common/config_files/aws_wifi_config.h"
"${ProjDirPath}/../common/config_files/FreeRTOSIPConfig.h"
"${ProjDirPath}/../common/config_files/FreeRTOSConfig.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_trng.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_trng.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_cache.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_cache.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexspi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexspi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_drv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_drv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_file.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/drivers/mflash_file.c"
"${ProjDirPath}/../../../../lib/wifi/portable/nxp/wiced/aws_wifi.c"
"${ProjDirPath}/../../../../lib/secure_sockets/portable/nxp/lwip/aws_secure_sockets.c"
"${ProjDirPath}/../../../../lib/bufferpool/aws_bufferpool_static_thread_safe.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_greengrass_discovery.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_helper_secure_connect.c"
"${ProjDirPath}/../../../../lib/shadow/aws_shadow.c"
"${ProjDirPath}/../../../../lib/shadow/aws_shadow_json.c"
"${ProjDirPath}/../../../../lib/utils/aws_system_init.c"
"${ProjDirPath}/../../../../lib/tls/aws_tls.c"
"${ProjDirPath}/../../../../lib/pkcs11/mbedtls/aws_pkcs11_mbedtls.c"
"${ProjDirPath}/../../../../lib/pkcs11/portable/nxp/common/aws_pkcs11_pal.c"
"${ProjDirPath}/../../../../lib/crypto/aws_crypto.c"
"${ProjDirPath}/../../../../lib/mqtt/aws_mqtt_agent.c"
"${ProjDirPath}/../../../../lib/mqtt/aws_mqtt_lib.c"
"${ProjDirPath}/../../../../lib/third_party/pkcs11/pkcs11t.h"
"${ProjDirPath}/../../../../lib/third_party/pkcs11/pkcs11f.h"
"${ProjDirPath}/../../../../lib/third_party/pkcs11/pkcs11.h"
"${ProjDirPath}/../../../../lib/third_party/jsmn/jsmn.c"
"${ProjDirPath}/../../../../lib/third_party/jsmn/jsmn.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/platforms/NEB1DX_01/wifi_nvram_image.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/port/sys_arch.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/port/arch/cc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/port/arch/perf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/port/arch/sys_arch.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/api_lib.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/api_msg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/err.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/if_api.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/netbuf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/netdb.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/netifapi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/sockets.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/api/tcpip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/altcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/altcp_alloc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/altcp_tcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/def.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/dns.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/inet_chksum.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/init.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/mem.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/memp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/netif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/pbuf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/raw.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/stats.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/sys.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/tcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/tcp_in.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/tcp_out.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/timeouts.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/udp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/autoip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/dhcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/etharp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/icmp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/igmp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/ip4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/ip4_addr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv4/ip4_frag.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/dhcp6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/ethip6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/icmp6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/inet6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/ip6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/ip6_addr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/ip6_frag.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/mld6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/core/ipv6/nd6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/compat/posix/netdb.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/compat/posix/arpa/inet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/compat/posix/net/if.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/compat/posix/sys/socket.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/compat/stdc/errno.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/altcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/altcp_tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/altcp_tls.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/api.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/arch.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/autoip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/debug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/def.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/dhcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/dhcp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/dns.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/err.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/errno.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ethip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/icmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/icmp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/if_api.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/igmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/inet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/inet_chksum.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/init.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip4_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip4_frag.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip6_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip6_frag.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/ip6_zone.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/mem.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/memp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/mld6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/nd6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/netbuf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/netdb.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/netif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/netifapi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/opt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/pbuf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/raw.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/sio.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/snmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/sockets.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/stats.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/sys.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/tcpbase.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/tcpip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/timeouts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/udp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/altcp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/api_msg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/mem_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/memp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/memp_std.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/nd6_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/raw_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/sockets_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/tcp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/priv/tcpip_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/autoip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/dhcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/dhcp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/dns.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/ethernet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/iana.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/icmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/icmp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/ieee.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/igmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/ip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/ip4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/ip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/mld6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/nd6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/lwip/prot/udp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/bridgeif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/bridgeif_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ethernet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ieee802154.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/lowpan6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/lowpan6_ble.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/lowpan6_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/lowpan6_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/slipif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/zepif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ccp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/chap_ms.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/chap-md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/chap-new.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/eap.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ecp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/eui64.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/fsm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ipcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ipv6cp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/lcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/magic.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/mppe.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ppp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ppp_impl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/ppp_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppapi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppcrypt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppdebug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppoe.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppol2tp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/pppos.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/upap.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/vj.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/polarssl/arc4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/polarssl/des.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/polarssl/md4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/polarssl/md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/include/netif/ppp/polarssl/sha1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/bridgeif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/bridgeif_fdb.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ethernet.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/lowpan6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/lowpan6_ble.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/lowpan6_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/slipif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/zepif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/auth.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/ccp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/chap_ms.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/chap-md5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/chap-new.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/demand.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/eap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/eui64.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/fsm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/ipcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/ipv6cp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/lcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/lwip_ecp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/magic.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/mppe.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/multilink.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/ppp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/pppapi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/pppcrypt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/pppoe.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/pppol2tp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/pppos.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/upap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/utils.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/lwip/src/netif/ppp/vj.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/src/fsl_sdio.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/inc/fsl_sdio.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/inc/fsl_sdmmc_host.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/inc/fsl_sdmmc_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/src/fsl_sdmmc_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/port/fsl_sdmmc_event.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/port/usdhc/freertos/fsl_sdmmc_event.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/port/usdhc/freertos/fsl_sdmmc_host.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_usdhc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_usdhc.h"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/MemMang/heap_4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/aes.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/aesni.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/arc4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/aria.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/asn1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/asn1write.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/base64.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/bignum.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/blowfish.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/bn_mul.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/camellia.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ccm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/certs.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/chacha20.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/chachapoly.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/check_config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/cipher.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/cipher_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/cmac.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/compat-1.3.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ctr_drbg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/debug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/des.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/dhm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ecdh.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ecdsa.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ecjpake.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ecp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ecp_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/entropy.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/entropy_poll.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/error.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/gcm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/havege.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/hkdf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/hmac_drbg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/md.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/md_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/md2.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/md4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/memory_buffer_alloc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/net.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/net_sockets.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/nist_kw.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/oid.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/padlock.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pem.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pk.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pk_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pkcs5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pkcs11.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/pkcs12.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/platform.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/platform_time.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/platform_util.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/poly1305.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ripemd160.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/rsa.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/rsa_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/sha1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/sha256.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/sha512.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl_cache.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl_ciphersuites.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl_cookie.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/ssl_ticket.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/threading.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/timing.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/version.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/x509.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/x509_crl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/x509_crt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/x509_csr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/include/mbedtls/xtea.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/aes.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/aesni.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/arc4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/aria.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/asn1parse.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/asn1write.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/base64.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/bignum.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/blowfish.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/camellia.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ccm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/certs.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/cipher.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/cipher_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/chacha20.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/chachapoly.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/cmac.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ctr_drbg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/debug.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/des.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/dhm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ecdh.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ecdsa.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ecjpake.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ecp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ecp_curves.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/entropy.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/entropy_poll.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/error.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/gcm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/havege.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/hkdf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/hmac_drbg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/md.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/md_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/md2.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/md4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/md5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/memory_buffer_alloc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/net_sockets.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/nist_kw.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/oid.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/padlock.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pem.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pk.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pk_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pkcs5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pkcs12.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pkparse.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/pkwrite.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/platform.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/platform_util.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/poly1305.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ripemd160.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/rsa.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/rsa_internal.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/sha1.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/sha256.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/sha512.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_cache.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_ciphersuites.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_cli.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_cookie.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_srv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_ticket.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/ssl_tls.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/threading.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/timing.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/version.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/version_features.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509_create.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509_crl.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509_crt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509_csr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509write_crt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/x509write_csr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/library/xtea.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ksdk_mbedtls.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ksdk_mbedtls.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ksdk_mbedtls_config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/sha1_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/sha256_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/des_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/des_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/aes_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/aes_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ecp_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ecp_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ecp_curves_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/mbedtls/port/ksdk/ecp_alt_ksdk.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_adc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_adc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_adc_etc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_adc_etc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_aipstz.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_aipstz.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_aoi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_aoi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_bee.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_bee.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_clock.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_clock.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_cmp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_cmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_csi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_csi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dcdc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dcdc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dmamux.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_dmamux.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_elcdif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_elcdif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_enc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_enc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_enet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_enet.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_ewm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_ewm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexcan.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexcan.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2c_master.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2c_master.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2s.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2s.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2s_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_i2s_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_spi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_spi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_spi_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_spi_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_uart.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_uart_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio_uart_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexram.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexram.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpio.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_gpio.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_iomuxc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_kpp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_kpp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpi2c.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpi2c.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpi2c_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpi2c_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpspi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpspi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpspi_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpspi_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpuart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpuart.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpuart_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_lpuart_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pit.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pit.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pmu.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pmu.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pwm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pwm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pxp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_pxp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_qtmr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_qtmr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_rtwdog.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_rtwdog.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_sai.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_sai.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_sai_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_sai_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_semc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_semc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_snvs_hp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_snvs_hp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_snvs_lp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_snvs_lp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_spdif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_spdif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_spdif_edma.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_spdif_edma.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_src.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_src.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_tempmon.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_tempmon.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_tsc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_tsc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_wdog.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_wdog.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_xbara.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_xbara.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_xbarb.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_xbarb.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/fsl_device_registers.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/MIMXRT1052.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/MIMXRT1052_features.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/system_MIMXRT1052.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/system_MIMXRT1052.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/str/fsl_str.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/str/fsl_str.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/debug_console/fsl_debug_console.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/debug_console/fsl_debug_console.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/debug_console/fsl_debug_console_conf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/uart/uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/uart/lpuart_adapter.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/serial_manager/serial_manager.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/serial_manager/serial_manager.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/serial_manager/serial_port_uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/serial_manager/serial_port_uart.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/lists/generic_list.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/components/lists/generic_list.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/gcc/startup_MIMXRT1052.S"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/utilities/fsl_assert.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/xip/fsl_flexspi_nor_boot.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/xip/fsl_flexspi_nor_boot.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/boards/evkbimxrt1050/xip/evkbimxrt1050_flexspi_nor_config.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/boards/evkbimxrt1050/xip/evkbimxrt1050_flexspi_nor_config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/boards/evkbimxrt1050/xip/evkbimxrt1050_sdram_ini_dcd.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/boards/evkbimxrt1050/xip/evkbimxrt1050_sdram_ini_dcd.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/core_cm7.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_armcc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_armclang.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_gcc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_compiler.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_version.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/mpu_armv7.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/core_armv8mbl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/core_armv8mml.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/arm_common_tables.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/arm_const_structs.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/arm_math.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/cmsis_iccarm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/CMSIS/Include/mpu_armv8.h"
"${ProjDirPath}/../../../../LICENSE"
"${ProjDirPath}/../../../../lib/include/event_groups.h"
"${ProjDirPath}/../../../../lib/include/FreeRTOS.h"
"${ProjDirPath}/../../../../lib/include/message_buffer.h"
"${ProjDirPath}/../../../../lib/include/queue.h"
"${ProjDirPath}/../../../../lib/include/semphr.h"
"${ProjDirPath}/../../../../lib/include/stream_buffer.h"
"${ProjDirPath}/../../../../lib/include/task.h"
"${ProjDirPath}/../../../../lib/include/timers.h"
"${ProjDirPath}/../../../../lib/include/private/deprecated_definitions.h"
"${ProjDirPath}/../../../../lib/include/private/list.h"
"${ProjDirPath}/../../../../lib/include/private/mpu_wrappers.h"
"${ProjDirPath}/../../../../lib/include/private/mpu_prototypes.h"
"${ProjDirPath}/../../../../lib/include/private/portable.h"
"${ProjDirPath}/../../../../lib/include/private/projdefs.h"
"${ProjDirPath}/../../../../lib/include/private/stack_macros.h"
"${ProjDirPath}/../../../../lib/include/private/threading_alt.h"
"${ProjDirPath}/../../../../lib/FreeRTOS/event_groups.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/list.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/queue.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/stream_buffer.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/tasks.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/timers.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/GCC/ARM_CM4F/fsl_tickless_generic.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/GCC/ARM_CM4F/fsl_tickless_systick.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/4343WA1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/4343WA1_clm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/4343WA1_mfg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/4343WA1_clm_mfg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/chip_constants.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/resources.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/resources.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/wwd_ap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/chips/4343W/wwd_chip_specific_functions.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/include/wiced_resource.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/internal/wifi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/network/LwIP/WWD/wwd_buffer.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/network/LwIP/WWD/wwd_buffer.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/network/LwIP/WWD/wwd_network.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/network/LwIP/WWD/wwd_network.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/include/platform_cache_def.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/include/platform_peripheral.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/wwd_platform_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/wwd_platform_separate_mcu.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/wwd_resources.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/LPC/platform_mcu_peripheral.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/LPC/WWD/wwd_platform.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/MCU/LPC/WWD/wwd_SDIO.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/platform/platform_toolchain.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/RTOS/FreeRTOS/WWD/wwd_FreeRTOS_systick.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/RTOS/FreeRTOS/WWD/wwd_rtos.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/RTOS/FreeRTOS/WWD/wwd_rtos.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/network/wwd_buffer_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/network/wwd_network_constants.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/network/wwd_network_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform/wwd_bus_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform/wwd_platform_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform/wwd_resource_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform/wwd_sdio_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/platform/wwd_spi_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/RTOS/wwd_rtos_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_assert.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_constants.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_debug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_eapol.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_events.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_management.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_poll.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_structures.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_wifi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_wifi_sleep.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/include/wwd_wlioctl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/bus_protocols/SDIO/wwd_bus_protocol.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/bus_protocols/SDIO/wwd_bus_protocol.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/bus_protocols/wwd_bus_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/bus_protocols/wwd_bus_protocol_interface.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/tlv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/tlv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_ap.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_ap_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_ap_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_bcmendian.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_clm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_clm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_debug.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_eapol.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_internal.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_logging.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_management.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_rtos_interface.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_sdpcm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_sdpcm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_thread.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_thread.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_thread_internal.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_thread_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_wifi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_wifi_chip_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_wifi_chip_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/internal/wwd_wifi_sleep.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/wwd.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/wwd_wiced.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/wiced/43xxx_Wi-Fi/WICED/generated_mac_address.txt"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/middleware/sdmmc/inc/fsl_sdmmc_spec.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MIMXRT1052/devices/MIMXRT1052/drivers/fsl_flexio.h"
)


set(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_DEBUG} -T${ProjDirPath}/MIMXRT1052xxxxx_flexspi_nor.ld -static")

set(CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE "${CMAKE_EXE_LINKER_FLAGS_FLEXSPI_NOR_RELEASE} -T${ProjDirPath}/MIMXRT1052xxxxx_flexspi_nor.ld -static")

TARGET_LINK_LIBRARIES(aws_demos.elf -Wl,--start-group)
target_link_libraries(aws_demos.elf optimized m)

target_link_libraries(aws_demos.elf optimized c)

target_link_libraries(aws_demos.elf optimized gcc)

target_link_libraries(aws_demos.elf optimized nosys)

target_link_libraries(aws_demos.elf optimized m)

target_link_libraries(aws_demos.elf optimized c)

target_link_libraries(aws_demos.elf optimized gcc)

target_link_libraries(aws_demos.elf optimized nosys)

TARGET_LINK_LIBRARIES(aws_demos.elf -Wl,--end-group)

