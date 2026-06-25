#include "AP_Periph.h"
#include <dronecan_msgs.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

void AP_Periph_FW::spray_system_init()
{
    spray_system.init(spray_system_send_routine_complete_message);
}

void AP_Periph_FW::spray_system_update()
{
    spray_system.update();
}

void AP_Periph_FW::spray_system_send_routine_complete_message(float amount_sprayed_ml,
                                                              uint32_t time_taken_ms,
                                                              bool within_range)
{
    uint8_t routine_complete_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_SPRAYROUTINECOMPLETE_MAX_SIZE];
    com_spsautomation_spraysystem_SprayRoutineComplete routine_complete_msg;
    routine_complete_msg.avg_flow_rate_ml_min = static_cast<uint16_t>(amount_sprayed_ml * 60000 / (time_taken_ms));
    routine_complete_msg.amount_sprayed_ml = amount_sprayed_ml;
    routine_complete_msg.time_taken_ms = time_taken_ms;
    routine_complete_msg.routine_within_threshold = within_range;
    routine_complete_msg.source_node_id = 0; // Not required for message

    uint16_t total_size = com_spsautomation_spraysystem_SprayRoutineComplete_encode(
            &routine_complete_msg,
            routine_complete_buffer,
            !periph.canfdout());

    periph.canard_broadcast(
            COM_SPSAUTOMATION_SPRAYSYSTEM_SPRAYROUTINECOMPLETE_SIGNATURE,
            COM_SPSAUTOMATION_SPRAYSYSTEM_SPRAYROUTINECOMPLETE_ID,
            CANARD_TRANSFER_PRIORITY_LOW,
            routine_complete_buffer,
            total_size);
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
    flow_status.source_node_id = 0; // Not required for message

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
    pump_status.source_node_id = 0; // Not required for message

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

void AP_Periph_FW::spray_system_handle_global_timesync_message(CanardInstance * canard_instance,
                                                     CanardRxTransfer * transfer)
{
    uavcan_protocol_GlobalTimeSync msg;
    static uint64_t last_sync_rx_timestamp = 0;
    int64_t time_offset_us = 0;
    uint64_t rx_timestamp;

    if (uavcan_protocol_GlobalTimeSync_decode(transfer, &msg))
    {
        /* Failed to decode message */
        return;
    }

    rx_timestamp = transfer->timestamp_usec;

    /* If the timestamp is 0, this is the first time sync message we've received */
    if (msg.previous_transmission_timestamp_usec != 0) {
        time_offset_us = last_sync_rx_timestamp - msg.previous_transmission_timestamp_usec;
        spray_system.set_time_offset(time_offset_us);
    }

    last_sync_rx_timestamp = rx_timestamp - time_offset_us;
}

void AP_Periph_FW::spray_system_handle_pump_control_message(CanardInstance * canard_instance,
                                              CanardRxTransfer * transfer)
{
    uint8_t response_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPCONTROL_RESPONSE_MAX_SIZE];
    com_spsautomation_spraysystem_PumpControlRequest msg;
    com_spsautomation_spraysystem_PumpControlResponse response_msg;

    if (com_spsautomation_spraysystem_PumpControlRequest_decode(transfer, &msg)) {
        /* Unable to decode message */
        return;
    }

    response_msg.success = true;

    if (!msg.pump_active)
    {
        /* Ignore speed if disabling the pump */
        spray_system.set_pump_enabled(false);
    }
    else
    {
        if (!spray_system.set_pump_speed(msg.speed))
        {
            /* Invalid speed setting */
            response_msg.success = false;
        }
        else
        {
            /* Speed setting was accepted, turn on pump */
            spray_system.set_pump_enabled(true);
        }
    }

    /* Encode and send response */
    uint16_t total_size = com_spsautomation_spraysystem_PumpControlResponse_encode(
            &response_msg,
            response_buffer,
            !periph.canfdout());

    canard_respond(canard_instance,
                   transfer,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPCONTROL_RESPONSE_SIGNATURE,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_PUMPCONTROL_RESPONSE_ID,
                   response_buffer,
                   total_size);
}

void AP_Periph_FW::spray_system_handle_nozzle_control_message(CanardInstance * canard_instance,
                                                    CanardRxTransfer * transfer)
{
    uint8_t response_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_NOZZLEMANUALCONTROL_RESPONSE_MAX_SIZE];
    com_spsautomation_spraysystem_NozzleManualControlRequest msg;
    com_spsautomation_spraysystem_NozzleManualControlResponse response;

    /* Decode request message */
    if (com_spsautomation_spraysystem_NozzleManualControlRequest_decode(transfer, &msg))
    {
        /* Failed to decode message */
        return;
    }

    /* Frequency and duty cycle fields are deprecated, only care about actuate_nozzle */
    if (msg.actuate_nozzle) {
        /* Open spray nozzle, close return line */
        spray_system.set_spray_nozzle_open(true);
        spray_system.set_return_line_open(false);
    }
    else
    {
        /* Close spray nozzle, open return line */
        spray_system.set_spray_nozzle_open(false);
        spray_system.set_return_line_open(true);
    }

    /* Encode and send response */
    response.success = true;

    uint16_t total_size = com_spsautomation_spraysystem_NozzleManualControlResponse_encode(
            &response,
            response_buffer,
            !periph.canfdout());

    canard_respond(canard_instance,
                   transfer,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_NOZZLEMANUALCONTROL_RESPONSE_SIGNATURE,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_NOZZLEMANUALCONTROL_RESPONSE_ID,
                   response_buffer,
                   total_size);
}

void AP_Periph_FW::spray_system_handle_schedule_routine_message(CanardInstance * canard_instance,
                                                  CanardRxTransfer * transfer)
{
    uint8_t response_buffer[COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZCONTROL_RESPONSE_MAX_SIZE];
    com_spsautomation_spraysystem_FlowNozControlRequest msg;
    com_spsautomation_spraysystem_FlowNozControlResponse response;

    /* Decode incoming mesage */
    if (com_spsautomation_spraysystem_FlowNozControlRequest_decode(transfer, &msg))
    {
        /* Failed to decode message */
        return;
    }

    SprayRoutine new_routine = {
            .desired_spray_ml = msg.desired_spray_ml,
            .desired_flow_rate_ml_min = msg.desired_flow_rate_ml_min,
            .time_allowed_ms = msg.time_allowed_ms,
            .start_time_ms = msg.start_time_utc_ms
    };

    bool success = true;

    if (spray_system.enqueue_spray_routine(new_routine) != SprayScheduleResult::SUCCESS)
    {
        success = false;
    }

    /* Encode and send response */
    response.success = success;

    uint16_t total_size = com_spsautomation_spraysystem_FlowNozControlResponse_encode(
            &response,
            response_buffer,
            !periph.canfdout());

    canard_respond(canard_instance,
                   transfer,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZCONTROL_RESPONSE_SIGNATURE,
                   COM_SPSAUTOMATION_SPRAYSYSTEM_FLOWNOZCONTROL_RESPONSE_ID,
                   response_buffer,
                   total_size);
}

#endif