idf_component_register(
    SRCS
        "port_main.c"
        "../../src/motion.c"
        "../../src/gripper.c"
        "../../src/safety.c"
        "../../src/modbus.c"
        "../../src/mqtt_pkt.c"
        "../../src/json_msg.c"
        "../../src/cell.c"
    INCLUDE_DIRS "../../include"
)
