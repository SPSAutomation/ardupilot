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
    /* Encode and send out status messages */
    /* Pressure sensor */
    uint8_t pressure_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_MAX_SIZE];
    com_spsautomation_spraysystem_PressureSensorStatus pressure_status;
    pressure_status.pressure_psi = spray_system.get_current_pressure_mbar();
    pressure_status.temperature_c = spray_system.get_current_temperature_c();

    uint16_t total_size = com_spsautomation_spraysystem_PressureSensorStatus_encode(
            &pressure_status,
            pressure_buffer,
            !periph.canfdout());

    canard_broadcast(COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_SIGNATURE,
                    COM_SPSAUTOMATION_SPRAYSYSTEM_PRESSURESENSORSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    pressure_buffer,
                    total_size);

    /* Flow controller status */
    uint8_t flow_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_MAX_SIZE];
    com_spsautomation_spraysystem_FlowNozStatus flow_status;
    flow_status.flow_rate_ml_min = spray_system.get_current_flow_rate_ml_min();
    flow_status.amount_flowed_ml = spray_system.get_amount_flowed_ml();
    flow_status.time_spraying_ms = spray_system.get_current_spray_time_ms();
    flow_status.nozzle_state = spray_system.get_spray_nozzle_state();

    total_size = com_spsautomation_spraysystem_FlowNozStatus_encode(
            &flow_status,
            flow_buffer,
            !periph.canfdout());

    canard_broadcast(COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_SIGNATURE,
                     COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZSTATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     flow_buffer,
                     total_size);

    /* Pump status */
    uint8_t pump_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPSTATUS_MAX_SIZE];
    com_spsautomation_spraysystem_PumpStatus pump_status;
    pump_status.speed = spray_system.get_current_pump_speed();
    pump_status.pump_state = spray_system.get_pump_enabled();

    total_size = com_spsautomation_spraysystem_PumpStatus_encode(
            &pump_status,
            pump_buffer,
            !periph.canfdout());

    canard_broadcast(COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPSTATUS_SIGNATURE,
                     COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPSTATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     pump_buffer,
                     total_size);
}

#endif