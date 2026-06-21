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
    uint8_t buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_MAX_SIZE];
    com_spsautomation_spraysystem_FlowNozStatus status;
    status.flow_rate_ml_min = spray_system.get_current_flow_rate_ml_min();

    uint16_t total_size = com_spsautomation_spraysystem_FlowNozStatus_encode(&status, buffer, !periph.canfdout());

    canard_broadcast(COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_SIGNATURE,
                    COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    total_size);

}



#endif