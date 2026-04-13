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
    ${CMAKE_CURRENT_SOURCE_DIR}/storage/CO_storage.c
)
target_include_directories(canopennode
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../App/canopen_app/                       # OD.h 位置（协议栈需要）
)

target_link_libraries(canopennode
    PUBLIC
        canopennode_driver
)
target_compile_definitions(canopennode PRIVATE
    CO_CONFIG_LEDS=0
    CO_CONFIG_LSS=0
)
target_compile_definitions(canopennode PUBLIC
    CO_CONFIG_CRC16=1  # 启用CRC16功能
    CO_CONFIG_STORAGE=1  # 启用存储功能
)

