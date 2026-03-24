add_library(canopennode STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopen.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_Emergency.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_HBconsumer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_NMT_Heartbeat.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_Node_Guarding.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_ODinterface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_PDO.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_SDOclient.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_SDOserver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_SYNC.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/CO_TIME.c
    ${CMAKE_CURRENT_SOURCE_DIR}/301/crc16-ccitt.c
)
target_include_directories(canopennode
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../App/                       # OD.h 位置（协议栈需要）
        ${CMAKE_CURRENT_SOURCE_DIR}/../Hardware/CanOpenDriver/    # CO_driver_target.h 接口
)

target_link_libraries(canopennode
    PRIVATE
        # stm32cubemx
)
target_compile_definitions(canopennode PRIVATE
    CO_CONFIG_LEDS=0
    CO_CONFIG_LSS=0
)
