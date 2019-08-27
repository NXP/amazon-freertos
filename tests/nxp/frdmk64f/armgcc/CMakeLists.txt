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


SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -Wall")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-common")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mthumb")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mapcs")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -Wall")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-common")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mthumb")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mapcs")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSDK_DEBUGCONSOLE=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSDK_I2C_BASED_COMPONENT_USED=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DUSE_RTOS=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DLWIP_NETIF_API=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DMEMP_NUM_NETCONN=4")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DAMAZON_FREERTOS_ENABLE_UNIT_TESTS=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DUNITY_INCLUDE_CONFIG_H=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFRDM_K64F")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFREEDOM")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFSL_RTOS_FREE_RTOS")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-common")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mthumb")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mapcs")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MMD")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MP")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fomit-frame-pointer")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSDK_DEBUGCONSOLE=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSDK_I2C_BASED_COMPONENT_USED=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DUSE_RTOS=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DLWIP_NETIF_API=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DMEMP_NUM_NETCONN=4")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DAMAZON_FREERTOS_ENABLE_UNIT_TESTS=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DUNITY_INCLUDE_CONFIG_H=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFRDM_K64F")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFREEDOM")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFSL_RTOS_FREE_RTOS")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-common")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mthumb")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mapcs")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MMD")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MP")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fomit-frame-pointer")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-common")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mthumb")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mapcs")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-rtti")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-exceptions")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MMD")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MP")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-common")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mthumb")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mapcs")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-rtti")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-exceptions")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MMD")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MP")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -static")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -z")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Map=output.map")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --defsym=__stack_size__=0x200")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --defsym=__heap_size__=0x200")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -static")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -z")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Map=output.map")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --defsym=__stack_size__=0x200")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --defsym=__heap_size__=0x200")

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/GCC/ARM_CM4F)

include_directories(${ProjDirPath}/../../../common/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk)

include_directories(${ProjDirPath}/../../../../lib/include)

include_directories(${ProjDirPath}/../../../../lib/include/private)

include_directories(${ProjDirPath}/../common)

include_directories(${ProjDirPath}/../../../../lib/third_party/unity/src)

include_directories(${ProjDirPath}/../../../../lib/third_party/unity/extras/fixture/src)

include_directories(${ProjDirPath}/../common/config_files)

include_directories(${ProjDirPath}/../common/application_code/nxp_code)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/phyksz8081)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/drivers)

include_directories(${ProjDirPath}/../../../../lib/third_party/pkcs11)

include_directories(${ProjDirPath}/../../../../lib/third_party/jsmn)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include)

include_directories(${ProjDirPath}/../../../../lib/FreeRTOS-Plus-TCP/include)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/str)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/debug_console)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/uart)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/serial_manager)

include_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/lists)

add_executable(aws_tests.elf 
"${ProjDirPath}/../common/application_code/main.c"
"${ProjDirPath}/../../../common/crypto/aws_test_crypto.c"
"${ProjDirPath}/../../../common/framework/aws_test_framework.c"
"${ProjDirPath}/../../../common/greengrass/aws_test_greengrass_discovery.c"
"${ProjDirPath}/../../../common/greengrass/aws_test_helper_secure_connect.c"
"${ProjDirPath}/../../../common/include/aws_application_version.h"
"${ProjDirPath}/../../../common/include/aws_clientcredential.h"
"${ProjDirPath}/../../../common/include/aws_clientcredential_keys.h"
"${ProjDirPath}/../../../common/include/aws_dev_mode_key_provisioning.h"
"${ProjDirPath}/../../../common/include/aws_greengrass_discovery_test_access_declare.h"
"${ProjDirPath}/../../../common/include/aws_greengrass_discovery_test_access_define.h"
"${ProjDirPath}/../../../common/include/aws_logging_task.h"
"${ProjDirPath}/../../../common/include/aws_mqtt_lib_test_access_declare.h"
"${ProjDirPath}/../../../common/include/aws_mqtt_lib_test_access_define.h"
"${ProjDirPath}/../../../common/include/aws_test_framework.h"
"${ProjDirPath}/../../../common/include/aws_test_runner.h"
"${ProjDirPath}/../../../common/include/aws_test_tcp.h"
"${ProjDirPath}/../../../common/include/aws_test_tls.h"
"${ProjDirPath}/../../../common/include/aws_test_utils.h"
"${ProjDirPath}/../../../common/memory_leak/aws_memory_leak.c"
"${ProjDirPath}/../../../common/mqtt/aws_test_mqtt_agent.c"
"${ProjDirPath}/../../../common/mqtt/aws_test_mqtt_lib.c"
"${ProjDirPath}/../../../common/pkcs11/aws_test_pkcs11.c"
"${ProjDirPath}/../../../common/secure_sockets/aws_test_tcp.c"
"${ProjDirPath}/../../../common/shadow/aws_test_shadow.c"
"${ProjDirPath}/../../../common/test_runner/aws_test_runner.c"
"${ProjDirPath}/../../../common/tls/aws_test_tls.c"
"${ProjDirPath}/../../../../lib/include/aws_appversion32.h"
"${ProjDirPath}/../../../../lib/third_party/unity/src/unity.c"
"${ProjDirPath}/../../../../lib/third_party/unity/src/unity.h"
"${ProjDirPath}/../../../../lib/third_party/unity/src/unity_internals.h"
"${ProjDirPath}/../../../../lib/third_party/unity/extras/fixture/src/unity_fixture.c"
"${ProjDirPath}/../../../../lib/third_party/unity/extras/fixture/src/unity_fixture.h"
"${ProjDirPath}/../../../../lib/third_party/unity/extras/fixture/src/unity_fixture_internals.h"
"${ProjDirPath}/../../../../lib/third_party/unity/extras/fixture/src/unity_fixture_malloc_overrides.h"
"${ProjDirPath}/../../../../demos/common/devmode_key_provisioning/aws_dev_mode_key_provisioning.c"
"${ProjDirPath}/../../../../demos/common/logging/aws_logging_task_dynamic_buffers.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_greengrass_discovery.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_helper_secure_connect.c"
"${ProjDirPath}/../common/config_files/aws_bufferpool_config.h"
"${ProjDirPath}/../common/config_files/aws_ggd_config.h"
"${ProjDirPath}/../common/config_files/aws_mqtt_agent_config.h"
"${ProjDirPath}/../common/config_files/aws_mqtt_config.h"
"${ProjDirPath}/../common/config_files/aws_pkcs11_config.h"
"${ProjDirPath}/../common/config_files/aws_secure_sockets_config.h"
"${ProjDirPath}/../common/config_files/aws_shadow_config.h"
"${ProjDirPath}/../common/config_files/aws_test_pkcs11_config.h"
"${ProjDirPath}/../common/config_files/aws_test_runner_config.h"
"${ProjDirPath}/../common/config_files/aws_test_tcp_config.h"
"${ProjDirPath}/../common/config_files/aws_test_wifi_config.h"
"${ProjDirPath}/../common/config_files/FreeRTOSConfig.h"
"${ProjDirPath}/../common/config_files/FreeRTOSIPConfig.h"
"${ProjDirPath}/../common/config_files/unity_config.h"
"${ProjDirPath}/../common/lwipopts.h"
"${ProjDirPath}/../common/application_code/nxp_code/board.c"
"${ProjDirPath}/../common/application_code/nxp_code/board.h"
"${ProjDirPath}/../common/application_code/nxp_code/pin_mux.c"
"${ProjDirPath}/../common/application_code/nxp_code/pin_mux.h"
"${ProjDirPath}/../common/application_code/nxp_code/clock_config.h"
"${ProjDirPath}/../common/application_code/nxp_code/clock_config.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/phyksz8081/fsl_phy.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/phyksz8081/fsl_phy.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_rnga.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_rnga.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/cau_api.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/fsl_mmcau.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/fsl_mmcau.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/drivers/mflash_drv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/drivers/mflash_drv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/drivers/mflash_file.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/drivers/mflash_file.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/ethernetif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/ethernetif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_enet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_enet.c"
"${ProjDirPath}/../../../../lib/secure_sockets/portable/nxp/lwip/aws_secure_sockets.c"
"${ProjDirPath}/../../../../lib/bufferpool/aws_bufferpool_static_thread_safe.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_greengrass_discovery.c"
"${ProjDirPath}/../../../../lib/greengrass/aws_helper_secure_connect.c"
"${ProjDirPath}/../../../../lib/utils/aws_system_init.c"
"${ProjDirPath}/../../../../lib/shadow/aws_shadow.c"
"${ProjDirPath}/../../../../lib/shadow/aws_shadow_json.c"
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
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/sys_arch.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/arch/cc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/arch/perf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/port/arch/sys_arch.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/api_lib.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/api_msg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/err.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/if_api.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/netbuf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/netdb.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/netifapi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/sockets.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/api/tcpip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/altcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/altcp_alloc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/altcp_tcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/def.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/dns.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/inet_chksum.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/init.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/mem.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/memp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/netif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/pbuf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/raw.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/stats.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/sys.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/tcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/tcp_in.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/tcp_out.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/timeouts.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/udp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/autoip.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/dhcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/etharp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/icmp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/igmp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/ip4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/ip4_addr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv4/ip4_frag.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/dhcp6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/ethip6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/icmp6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/inet6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/ip6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/ip6_addr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/ip6_frag.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/mld6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/core/ipv6/nd6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/compat/posix/netdb.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/compat/posix/arpa/inet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/compat/posix/net/if.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/compat/posix/sys/socket.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/compat/stdc/errno.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/altcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/altcp_tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/altcp_tls.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/api.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/arch.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/autoip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/debug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/def.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/dhcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/dhcp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/dns.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/err.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/errno.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ethip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/icmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/icmp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/if_api.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/igmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/inet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/inet_chksum.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/init.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip4_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip4_frag.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip6_addr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip6_frag.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/ip6_zone.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/mem.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/memp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/mld6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/nd6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/netbuf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/netdb.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/netif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/netifapi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/opt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/pbuf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/raw.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/sio.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/snmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/sockets.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/stats.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/sys.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/tcpbase.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/tcpip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/timeouts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/udp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/altcp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/api_msg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/mem_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/memp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/memp_std.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/nd6_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/raw_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/sockets_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/tcp_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/priv/tcpip_priv.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/autoip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/dhcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/dhcp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/dns.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/ethernet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/iana.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/icmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/icmp6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/ieee.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/igmp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/ip.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/ip4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/ip6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/mld6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/nd6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/tcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/lwip/prot/udp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/bridgeif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/bridgeif_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/etharp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ethernet.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ieee802154.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/lowpan6.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/lowpan6_ble.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/lowpan6_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/lowpan6_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/slipif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/zepif.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ccp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/chap_ms.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/chap-md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/chap-new.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/eap.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ecp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/eui64.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/fsm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ipcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ipv6cp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/lcp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/magic.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/mppe.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ppp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ppp_impl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/ppp_opts.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppapi.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppcrypt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppdebug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppoe.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppol2tp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/pppos.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/upap.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/vj.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/polarssl/arc4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/polarssl/des.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/polarssl/md4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/polarssl/md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/include/netif/ppp/polarssl/sha1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/bridgeif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/bridgeif_fdb.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ethernet.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/lowpan6.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/lowpan6_ble.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/lowpan6_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/slipif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/zepif.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/auth.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/ccp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/chap_ms.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/chap-md5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/chap-new.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/demand.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/eap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/eui64.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/fsm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/ipcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/ipv6cp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/lcp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/lwip_ecp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/magic.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/mppe.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/multilink.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/ppp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/pppapi.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/pppcrypt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/pppoe.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/pppol2tp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/pppos.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/upap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/utils.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/lwip/src/netif/ppp/vj.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/MemMang/heap_4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/aes.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/aesni.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/arc4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/aria.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/asn1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/asn1write.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/base64.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/bignum.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/blowfish.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/bn_mul.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/camellia.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ccm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/certs.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/chacha20.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/chachapoly.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/check_config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/cipher.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/cipher_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/cmac.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/compat-1.3.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ctr_drbg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/debug.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/des.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/dhm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ecdh.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ecdsa.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ecjpake.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ecp.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ecp_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/entropy.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/entropy_poll.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/error.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/gcm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/havege.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/hkdf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/hmac_drbg.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/md.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/md_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/md2.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/md4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/md5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/memory_buffer_alloc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/net.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/net_sockets.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/nist_kw.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/oid.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/padlock.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pem.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pk.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pk_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pkcs5.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pkcs11.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/pkcs12.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/platform.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/platform_time.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/platform_util.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/poly1305.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ripemd160.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/rsa.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/rsa_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/sha1.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/sha256.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/sha512.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl_cache.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl_ciphersuites.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl_cookie.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl_internal.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/ssl_ticket.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/threading.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/timing.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/version.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/x509.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/x509_crl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/x509_crt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/x509_csr.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/include/mbedtls/xtea.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/aes.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/aesni.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/arc4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/aria.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/asn1parse.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/asn1write.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/base64.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/bignum.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/blowfish.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/camellia.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ccm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/certs.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/cipher.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/cipher_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/chacha20.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/chachapoly.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/cmac.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ctr_drbg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/debug.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/des.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/dhm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ecdh.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ecdsa.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ecjpake.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ecp.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ecp_curves.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/entropy.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/entropy_poll.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/error.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/gcm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/havege.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/hkdf.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/hmac_drbg.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/md.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/md_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/md2.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/md4.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/md5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/memory_buffer_alloc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/net_sockets.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/nist_kw.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/oid.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/padlock.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pem.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pk.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pk_wrap.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pkcs5.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pkcs12.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pkparse.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/pkwrite.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/platform.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/platform_util.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/poly1305.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ripemd160.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/rsa.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/rsa_internal.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/sha1.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/sha256.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/sha512.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_cache.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_ciphersuites.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_cli.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_cookie.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_srv.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_ticket.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/ssl_tls.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/threading.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/timing.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/version.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/version_features.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509_create.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509_crl.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509_crt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509_csr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509write_crt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/x509write_csr.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/library/xtea.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ksdk_mbedtls.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ksdk_mbedtls.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ksdk_mbedtls_config.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/sha1_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/sha256_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/des_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/des_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/aes_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/aes_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ecp_alt.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ecp_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ecp_curves_alt.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mbedtls/port/ksdk/ecp_alt_ksdk.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_adapter.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_utilities.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_features.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_controller.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_controller.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_flash.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_flash.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_cache.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_cache.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_flexnvm.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_ftfx_flexnvm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_flash.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_gpio.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_gpio.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_clock.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_clock.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/str/fsl_str.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/str/fsl_str.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/debug_console/fsl_debug_console.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/debug_console/fsl_debug_console.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/debug_console/fsl_debug_console_conf.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_port.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_uart.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_smc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_smc.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/fsl_device_registers.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/MK64F12.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/MK64F12_features.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/system_MK64F12.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/system_MK64F12.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/uart/uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/uart/uart_adapter.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/serial_manager/serial_manager.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/serial_manager/serial_manager.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/serial_manager/serial_port_uart.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/serial_manager/serial_port_uart.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/lists/generic_list.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/components/lists/generic_list.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_common.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/drivers/fsl_common.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/gcc/startup_MK64F12.S"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/devices/MK64F12/utilities/fsl_assert.c"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/core_cm4.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_armcc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_armclang.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_gcc.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_compiler.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_version.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/mpu_armv7.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/core_armv8mbl.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/core_armv8mml.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/arm_common_tables.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/arm_const_structs.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/arm_math.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/cmsis_iccarm.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/CMSIS/Include/mpu_armv8.h"
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
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/GCC/ARM_CM4F/fsl_tickless_generic.h"
"${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/GCC/ARM_CM4F/fsl_tickless_systick.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
"${ProjDirPath}/../../../../lib/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h"
)


set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${ProjDirPath}/MK64FN1M0xxx12_flash_data.ld -static")

set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/MK64FN1M0xxx12_flash_data.ld -static")

TARGET_LINK_LIBRARIES(aws_tests.elf -Wl,--start-group)
target_link_libraries(aws_tests.elf debug m)

target_link_libraries(aws_tests.elf debug c)

target_link_libraries(aws_tests.elf debug gcc)

target_link_libraries(aws_tests.elf debug nosys)

target_link_libraries(aws_tests.elf optimized m)

target_link_libraries(aws_tests.elf optimized c)

target_link_libraries(aws_tests.elf optimized gcc)

target_link_libraries(aws_tests.elf optimized nosys)

link_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/asm-cm4-cm7)

target_link_libraries(aws_tests.elf debug ${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/asm-cm4-cm7/lib_mmcau.a)

link_directories(${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/asm-cm4-cm7)

target_link_libraries(aws_tests.elf optimized ${ProjDirPath}/../../../../lib/third_party/mcu_vendor/nxp/MK64F12/middleware/mmcau/asm-cm4-cm7/lib_mmcau.a)

TARGET_LINK_LIBRARIES(aws_tests.elf -Wl,--end-group)

