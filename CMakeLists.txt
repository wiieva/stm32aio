cmake_minimum_required (VERSION 3.0)
INCLUDE (CMakeForceCompiler)

SET (CMAKE_SYSTEM_NAME Generic)
SET (CMAKE_SYSTEM_PROCESSOR arm)
CMAKE_FORCE_C_COMPILER (arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER (arm-none-eabi-g++ GNU)
set (CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set (CMAKE_OBJCOPY arm-none-eabi-objcopy)
set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set (DFU_UPLOAD_DEV /dev/tty.usbmodem*)
enable_language (ASM)

set (PRJ stm32aio)
project (${PRJ})

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


file (GLOB_RECURSE stm32aio_SRC "src/*" )
file (GLOB stdperiph_SRC 
        "lib/STM32F10x_StdPeriph_Driver/src/*" )
file (GLOB speex_SRC 
        "lib/STM32F10x_Speex_Lib/libspeex/*" 
        "lib/STM32F10x_Speex_Lib/STM32/libspeex/*" 
        "lib/STM32F10x_Speex_Lib/STM32/libspeex/gcc/*")
file (GLOB usb_SRC 
        "lib/STM32_USB_OTG_Driver/src/usb_core.c"
        "lib/STM32_USB_OTG_Driver/src/usb_dcd.c"
        "lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c"
        "lib/STM32_USB_Device_Library/Core/src/*"
        "lib/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c"
	"lib/STM32_USB_Device_Library/Class/msc/src/usbd_msc_core.c"
	"lib/STM32_USB_Device_Library/Class/msc/src/usbd_msc_bot.c"
	"lib/STM32_USB_Device_Library/Class/msc/src/usbd_msc_scsi.c"
	"lib/STM32_USB_Device_Library/Class/msc/src/usbd_msc_data.c"
	"lib/STM32F10x_StdPeriph_Driver/src/*")
file (GLOB webrtc_SRC
        "lib/webrtc_vad/vad/*"
        "lib/webrtc_vad/spl/spl_core.c"
        "lib/webrtc_vad/spl/spl_init.c"
        "lib/webrtc_vad/spl/min_max_operations.c")
file (GLOB helix_SRC
        "lib/helix/real/*.c"
        "lib/helix/*.c")
file (GLOB pdm_SRC
        "lib/pdm/*")
file (GLOB startup_SRC
        "lib/CMSIS/Device/ST/STM32F10x/Source/Templates/gcc_ride7/startup_stm32f10x_cl.s")

set (SRCS ${stm32aio_SRC} ${stdperiph_SRC} ${speex_SRC} ${usb_SRC} ${webrtc_SRC} ${helix_SRC} ${pdm_SRC} ${startup_SRC})

set (INCLUDES
        "src" "src/common" "src/stm32f" "src/usbuart" "src/usbstorage"
        "lib/CMSIS/Include"
        "lib/STM32F10x_Speex_Lib/include"
        "lib/STM32F10x_Speex_Lib/STM32/include"
        "lib/STM32F10x_Speex_Lib/STM32"
        "lib/STM32F10x_Speex_Lib/STM32/libspeex/gcc"
        "lib/STM32F10x_Speex_Lib/libspeex"
        "lib/webrtc_vad/vad" "lib/webrtc_vad/include" "lib/webrtc_vad/spl/include"
        "lib/helix/pub" "lib/helix/real" "lib/helix/"
        "lib/STM32_USB_OTG_Driver/inc"
        "lib/STM32_USB_Device_Library/Core/inc"
        "lib/STM32_USB_Device_Library/Class/cdc/inc"
        "lib/STM32_USB_Device_Library/Class/msc/inc"
        "lib/STM32F10x_StdPeriph_Driver/inc"
        "lib/CMSIS/Device/ST/STM32F10x/Include"
        "lib/pdm")

add_definitions (-DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -DHAVE_CONFIG_H -DUSE_SMALLFT -DUSE_USB_OTG_FS -DHSE_VALUE=8000000)
set (CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-mcpu=cortex-m3 -mthumb -fdata-sections -ffunction-sections -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable -DNDEBUG")
set (CMAKE_EXE_LINKER_FLAGS ${CMAKE_C_FLAGS} "-mcpu=cortex-m3 -mthumb -Wl,-script=${CMAKE_SOURCE_DIR}/ld/stm32f105rb_flash.ld -Wl,--gc-sections -Wl,-nostdlib")
set (CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m3")

add_executable (${PRJ} ${SRCS})
target_include_directories (${PRJ} PRIVATE ${INCLUDES})
add_custom_command (TARGET ${PRJ} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS "--output-format=binary" "${PRJ}" "${PRJ}.bin" )
add_custom_command (TARGET ${PRJ} POST_BUILD COMMAND dfu-suffix ARGS "--vid=0483" "--pid=df11" "--add=${PRJ}.bin" )

add_custom_target (upload COMMAND st-flash write ${PRJ}.bin 0x8000000 )
add_custom_target (upload-dfu COMMAND dfu-util -w 15 -a 0 -D ${PRJ}.bin -s 0x8000000 -r -W ${DFU_UPLOAD_DEV} DEPENDS ${PRJ})

