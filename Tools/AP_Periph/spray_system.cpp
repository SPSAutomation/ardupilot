#include "AP_Periph.h"
#include <dronecan_msgs.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

void AP_Periph_FW::spray_system_init() {
    spray_system.init();
}

void AP_Periph_FW::spray_system_update() {
    spray_system.update();
}

void AP_Periph_FW::spray_system_send_status()
{
    uint8_t buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_MAX_SIZE];
    com_spsautomation_spraysystem_PressureSensorStatus status;
    status.pressure_psi = spray_system.get_current_pressure_mbar();
    status.temperature_c = spray_system.get_current_temperature_c();

    uint16_t total_size = com_spsautomation_spraysystem_PressureSensorStatus_encode(&status, buffer, !periph.canfdout());

    canard_broadcast(COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_SIGNATURE,
                    COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    total_size);

}



#endif