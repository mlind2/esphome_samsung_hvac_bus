#include <queue>
#include <map>
#include <cmath>
#include <string>
#include "esphome/core/hal.h"
#include "util.h"
#include "log.h"
#include "protocol_non_nasa.h"

std::map<std::string, esphome::samsung_ac::NonNasaCommand20> last_command20s_;
// Track last preset command sent per device to help interpret 0x28 status responses
// Hints persist until explicitly cleared (AltMode=0) or overwritten by confirmed status
std::map<std::string, esphome::samsung_ac::AltMode> last_preset_sent_;

esphome::samsung_ac::NonNasaDataPacket nonpacket_;

namespace esphome
{
    namespace samsung_ac
    {
        static bool pending_keepalive_ = false;
        static uint32_t pending_keepalive_due_ms_ = 0;
        static uint32_t last_keepalive_sent_ms_ = 0;
        constexpr uint32_t KEEPALIVE_DELAY_MS = 30;
        constexpr uint32_t KEEPALIVE_MIN_INTERVAL_MS = 5000;

        // Track cumulative energy calculation per device address
        // Note: Energy tracker persists across device reconnections. This is intentional to maintain
        // cumulative energy across device restarts. The tracker is keyed by device address, so if
        // a device is removed and re-added with the same address, energy continues accumulating.
        // Note: Uses double for accumulated_energy_kwh to maintain precision during long-term accumulation,
        // matching NASA protocol approach. Converted to float only when publishing (API requirement).
        struct CumulativeEnergyTracker
        {
            double accumulated_energy_kwh = 0.0; // Accumulated energy in kWh (double for precision)
            uint32_t last_update_time_ms = 0;    // Last time power was updated (milliseconds)
            float last_power_w = 0.0f;           // Last power value in Watts
            bool has_previous_update = false;    // Track if we've had at least one update (handles millis()=0 edge case)
        };

        std::map<std::string, CumulativeEnergyTracker> cumulative_energy_trackers_;

        // Energy calculation constants
        // MIN_DELTA_MS: Minimum time delta between energy updates (100ms)
        //   - Skips calculations for very small intervals (negligible energy)
        //   - Reduces unnecessary CPU usage and improves precision
        // MAX_DELTA_MS: Maximum time delta between energy updates (1 hour)
        //   - Caps maximum delta to avoid huge increments from stale data
        //   - Prevents unrealistic energy calculations from long gaps
        constexpr uint32_t MIN_DELTA_MS = 100;     // Minimum 100ms between energy updates
        constexpr uint32_t MAX_DELTA_MS = 3600000; // Maximum 1 hour (3600000 ms) between updates

        // Helper function to update cumulative energy tracker
        // This function handles all edge cases: wraparound, first update, time delta validation, and energy calculation
        // Uses trapezoidal rule for energy calculation: Energy = Average_Power (W) × Time (hours)
        // Returns true if energy was calculated and tracker was updated, false otherwise
        static bool update_cumulative_energy_tracker(CumulativeEnergyTracker &tracker, float current_power_w, uint32_t now)
        {
            // Clamp power to non-negative: HVAC systems consume power (positive values)
            // Negative values would indicate measurement error or sensor issues
            if (current_power_w < 0.0f)
            {
                if (debug_log_messages)
                {
                    LOGW("Cmd8D: Negative power detected (%.2f W), clamping to 0", current_power_w);
                }
                current_power_w = 0.0f;
            }

            // Handle first update: just store values, don't calculate energy yet
            if (!tracker.has_previous_update)
            {
                tracker.last_power_w = current_power_w;
                tracker.last_update_time_ms = now;
                tracker.has_previous_update = true;
                return false; // No energy calculated on first update
            }

            // Calculate time delta, handling millis() wraparound (occurs every ~49.7 days)
            uint32_t delta_ms;
            if (now >= tracker.last_update_time_ms)
            {
                delta_ms = now - tracker.last_update_time_ms;
            }
            else
            {
                // Wraparound detected: calculate from last_update_time_ms to UINT32_MAX, then from 0 to now
                delta_ms = (UINT32_MAX - tracker.last_update_time_ms) + now + 1;
            }

            // Validate time delta: skip if too small (negligible energy) or too large (stale data)
            if (delta_ms < MIN_DELTA_MS)
            {
                return false; // Skip calculation for very small intervals
            }
            if (delta_ms > MAX_DELTA_MS)
            {
                if (debug_log_messages)
                {
                    LOGW("Cmd8D: Large time delta detected (%u ms, ~%.1f hours), capping to 1 hour", delta_ms, delta_ms / 3600000.0f);
                }
                delta_ms = MAX_DELTA_MS; // Cap to 1 hour
            }

            // Calculate energy using trapezoidal rule: Energy = Average_Power (W) × Time (hours)
            // Average power = (last_power + current_power) / 2
            // Time in hours = delta_ms / (1000 * 3600)
            // Energy in kWh = (Average_Power (W) × Time (hours)) / 1000
            double average_power_w = (static_cast<double>(tracker.last_power_w) + static_cast<double>(current_power_w)) / 2.0;
            double time_hours = static_cast<double>(delta_ms) / 3600000.0; // Convert ms to hours
            double energy_kwh = (average_power_w * time_hours) / 1000.0;   // Convert W×h to kWh

            // Accumulate energy
            tracker.accumulated_energy_kwh += energy_kwh;

            // Update tracker state
            tracker.last_power_w = current_power_w;
            tracker.last_update_time_ms = now;

            return true; // Energy was calculated and tracker was updated
        }
        std::list<NonNasaRequestQueueItem> nonnasa_requests;
        bool controller_registered = false;
        bool indoor_unit_awake = true;

        uint8_t build_checksum(std::vector<uint8_t> &data)
        {
            uint8_t sum = data[1];
            for (uint8_t i = 2; i < 12; i++)
            {
                sum = sum ^ data[i];
            }
            return sum;
        }

        std::string NonNasaCommand20::to_string()
        {
            std::string str;
            str += "target_temp:" + std::to_string(target_temp) + "; ";
            str += "room_temp:" + std::to_string(room_temp) + "; ";
            str += "pipe_in:" + std::to_string(pipe_in) + "; ";
            str += "pipe_out:" + std::to_string(pipe_out) + "; ";
            str += "power:" + std::to_string(power ? 1 : 0) + "; ";
            str += "wind_direction:" + std::to_string((uint8_t)wind_direction) + "; ";
            str += "fanspeed:" + std::to_string((uint8_t)fanspeed) + "; ";
            str += "mode:" + long_to_hex((uint8_t)mode);
            return str;
        }

        std::string NonNasaCommandC0::to_string()
        {
            std::string str;
            str += "ou_operation_mode:" + long_to_hex((uint8_t)outdoor_unit_operation_mode) + "; ";
            str += "ou_4way_valve:" + std::to_string(outdoor_unit_4_way_valve ? 1 : 0) + "; ";
            str += "ou_hot_gas_bypass:" + std::to_string(outdoor_unit_hot_gas_bypass ? 1 : 0) + "; ";
            str += "ou_compressor:" + std::to_string(outdoor_unit_compressor ? 1 : 0) + "; ";
            str += "ou_ac_fan:" + std::to_string(outdoor_unit_ac_fan ? 1 : 0) + "; ";
            str += "ou_outdoor_temp[°C]:" + std::to_string(outdoor_unit_outdoor_temp_c) + "; ";
            str += "ou_discharge_temp[°C]:" + std::to_string(outdoor_unit_discharge_temp_c) + "; ";
            str += "ou_condenser_mid_temp[°C]:" + std::to_string(outdoor_unit_condenser_mid_temp_c);
            return str;
        }

        std::string NonNasaCommandC1::to_string()
        {
            std::string str;
            str += "ou_sump_temp[°C]:" + std::to_string(outdoor_unit_sump_temp_c);
            return str;
        }

        std::string NonNasaCommand8D::to_string()
        {
            std::string str;
            str += "inverter_current[A]:" + std::to_string(inverter_current_a) + "; ";
            str += "inverter_voltage[V]:" + std::to_string(inverter_voltage_v) + "; ";
            str += "inverter_power[W]:" + std::to_string(inverter_power_w);
            return str;
        }

        std::string NonNasaCommandF0::to_string()
        {
            std::string str;
            str += "ou_freeze_protection:" + std::to_string(outdoor_unit_freeze_protection ? 1 : 0) + "; ";
            str += "ou_heating_overload:" + std::to_string(outdoor_unit_heating_overload ? 1 : 0) + "; ";
            str += "ou_defrost_control:" + std::to_string(outdoor_unit_defrost_control ? 1 : 0) + "; ";
            str += "ou_discharge_protection:" + std::to_string(outdoor_unit_discharge_protection ? 1 : 0) + "; ";
            str += "ou_current_control:" + std::to_string(outdoor_unit_current_control ? 1 : 0) + "; ";
            str += "inverter_order_frequency[Hz]:" + std::to_string(inverter_order_frequency_hz) + "; ";
            str += "inverter_target_frequency[Hz]:" + std::to_string(inverter_target_frequency_hz) + "; ";
            str += "inverter_current_frequency[Hz]:" + std::to_string(inverter_current_frequency_hz) + "; ";
            str += "ou_bldc_fan:" + std::to_string(outdoor_unit_bldc_fan ? 1 : 0) + "; ";
            str += "ou_error_code:" + long_to_hex((uint8_t)outdoor_unit_error_code);
            return str;
        }

        std::string NonNasaCommandF1::to_string()
        {
            std::string str;
            str += "Electronic Expansion Valves: ";
            str += "EEV_A:" + std::to_string(outdoor_unit_EEV_A) + "; ";
            str += "EEV_B:" + std::to_string(outdoor_unit_EEV_B) + "; ";
            str += "EEV_C:" + std::to_string(outdoor_unit_EEV_C) + "; ";
            str += "EEV_D:" + std::to_string(outdoor_unit_EEV_D);
            return str;
        }

        std::string NonNasaCommandF3::to_string()
        {
            std::string str;
            str += "inverter_max_frequency[Hz]:" + std::to_string(inverter_max_frequency_hz) + "; ";
            str += "inverter_total_capacity_requirement[kW]:" + std::to_string(inverter_total_capacity_requirement_kw) + "; ";
            str += "inverter_current[ADC]:" + std::to_string(inverter_current_a) + "; ";
            str += "inverter_voltage[VDC]:" + std::to_string(inverter_voltage_v) + "; ";
            str += "inverter_power[W]:" + std::to_string(inverter_power_w);
            return str;
        }

        std::string NonNasaDataPacket::to_string()
        {
            std::string str;
            str += "{";
            str += "src:" + src + ";";
            str += "dst:" + dst + ";";
            str += "cmd:" + long_to_hex((uint8_t)cmd) + ";";
            switch (cmd)
            {
            case NonNasaCommand::Cmd1C:
            {
                str += "command1C:{" + command1C.to_string() + "}";
                break;
            }
            case NonNasaCommand::Cmd20:
            {
                str += "command20:{" + command20.to_string() + "}";
                break;
            }
            case NonNasaCommand::Cmd21:
            {
                str += "command21:{" + command21.to_string() + "}";
                break;
            }
            case NonNasaCommand::Cmd28:
            {
                str += "command28:{" + command28.to_string() + "}";
                break;
            }
            case NonNasaCommand::Cmd2F:
            {
                str += "command2F:{" + command2F.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdF5:
            {
                str += "commandF5:{" + commandF5.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdC0:
            {
                str += "commandC0:{" + commandC0.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdC1:
            {
                str += "commandC1:{" + commandC1.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdC6:
            {
                str += "commandC6:{" + commandC6.to_string() + "}";
                break;
            }
            case NonNasaCommand::Cmd8D:
            {
                str += "command8D:{" + command8D.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdF0:
            {
                str += "commandF0:{" + commandF0.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdF1:
            {
                str += "commandF1:{" + commandF1.to_string() + "}";
                break;
            }
            case NonNasaCommand::CmdF3:
            {
                str += "commandF3:{" + commandF3.to_string() + "}";
                break;
            }
            default:
            {
                str += "raw:" + commandRaw.to_string();
                break;
            }
            }

            str += "}";
            return str;
        }

        DecodeResult NonNasaDataPacket::decode(std::vector<uint8_t> &data)
        {
            // Stream-safe: wait until we have at least 14 bytes
            if (data.size() < 14)
                return {DecodeResultType::Fill, 0};

            // Only examine the first 14 bytes (buffer may contain more)
            // Frame format: 0x32 .... 0x34 (14 bytes total)
            if (data[0] != 0x32)
                return {DecodeResultType::Discard, 1};

            if (data[13] != 0x34)
                return {DecodeResultType::Discard, 1};

            // Validate checksum against first 14 bytes
            uint8_t crc_expected = build_checksum(data); // uses bytes [1..11]
            uint8_t crc_actual = data[12];

            if (crc_actual != crc_expected)
            {
                if (debug_log_undefined_messages)
                    LOGW("NonNASA: invalid crc - got %d but should be %d: %s",
                         crc_actual, crc_expected,
                         bytes_to_hex(std::vector<uint8_t>(data.begin(), data.begin() + 14)).c_str());

                // resync gently (consume 1) - safer in streams
                return {DecodeResultType::Discard, 1};
            }

            // Decode using first 14 bytes
            src = long_to_hex(data[1]);
            dst = long_to_hex(data[2]);

            cmd = (NonNasaCommand)data[3];
            switch (cmd)
            {
            case NonNasaCommand::Cmd1C:
                // Feature status response from indoor unit
                // Byte 4: Bit 0=Beep, Bit 3=Clean, etc.
                command1C.feature_status_byte = data[4];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::Cmd20:
                command20.target_temp = data[4] - 55;
                command20.room_temp = data[5] - 55;
                command20.pipe_in = data[6] - 55;
                command20.wind_direction = (NonNasaWindDirection)((data[7]) >> 3);
                command20.fanspeed = (NonNasaFanspeed)((data[7] & 0b00000111));
                command20.mode = (NonNasaMode)(data[8] & 0b00111111);
                command20.power = data[8] & 0b10000000;
                command20.pipe_out = data[11] - 55;

                if (command20.wind_direction == (NonNasaWindDirection)0)
                    command20.wind_direction = NonNasaWindDirection::Stop;

                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::Cmd21:
                // Display status response
                // Byte 7: Display ON/OFF status
                command21.display_status = data[7];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::Cmd28:
                // Preset/mode status response
                // Byte 4: Current preset/mode status
                command28.preset_status = data[4];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::Cmd2F:
                // Usage statistics response
                // Byte 9 and Byte 11: Usage data
                command2F.usage_byte_9 = data[9];
                command2F.usage_byte_11 = data[11];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdF5:
                // Preset control command (can be sent by us or remote control)
                // Byte 4: Encodes preset type and state
                commandF5.preset_byte4 = data[4];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdC0:
                commandC0.outdoor_unit_operation_mode = data[4];
                commandC0.outdoor_unit_4_way_valve = data[6] & 0b10000000;
                commandC0.outdoor_unit_hot_gas_bypass = data[6] & 0b00100000;
                commandC0.outdoor_unit_compressor = data[6] & 0b00000100;
                commandC0.outdoor_unit_ac_fan = data[7] & 0b00000011;
                commandC0.outdoor_unit_outdoor_temp_c = data[8] - 55;
                commandC0.outdoor_unit_discharge_temp_c = data[10] - 55;
                commandC0.outdoor_unit_condenser_mid_temp_c = data[11] - 55;
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdC1:
                commandC1.outdoor_unit_sump_temp_c = data[8] - 55;
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdC6:
                commandC6.control_status = data[4];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::Cmd8D:
                // Cmd8D from outdoor unit - contains power/energy data
                // Format: current raw value = data[8], voltage = data[10] * 2
                //
                // Power calculation consistency explanation:
                // The current sensor has a filter (multiply: 0.1) that will apply to the published current value.
                // To maintain the fundamental relationship: published_power = published_current × published_voltage,
                // we must apply the same 0.1 multiplier to the power calculation.
                //
                // Example: raw_current=100, raw_voltage=120 (raw)
                //   - Calculated current = 100 / 10 = 10A
                //   - Published current = 10 * 0.1 = 1A (after filter)
                //   - Calculated voltage = 120 * 2 = 240V
                //   - Published voltage = 240V (no filter)
                //   - Calculated power = (100 / 10) * 0.1 * (120 * 2) = 1 * 240 = 240W
                //   - Published power = 240W
                //   - Verification: 1A × 240V = 240W ✓
                //
                command8D.inverter_current_a = (float)data[8] / 10;                                              // Current in Amps (raw value / 10)
                command8D.inverter_voltage_v = (float)data[10] * 2;                                              // Voltage in Volts (raw value * 2)
                command8D.inverter_power_w = command8D.inverter_current_a * 0.1f * command8D.inverter_voltage_v; // Power in Watts
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdF0:
                commandF0.outdoor_unit_freeze_protection = data[4] & 0b10000000;
                commandF0.outdoor_unit_heating_overload = data[4] & 0b01000000;
                commandF0.outdoor_unit_defrost_control = data[4] & 0b00100000;
                commandF0.outdoor_unit_discharge_protection = data[4] & 0b00010000;
                commandF0.outdoor_unit_current_control = data[4] & 0b00001000;
                commandF0.inverter_order_frequency_hz = data[5];
                commandF0.inverter_target_frequency_hz = data[6];
                commandF0.inverter_current_frequency_hz = data[7];
                commandF0.outdoor_unit_bldc_fan = data[8] & 0b00000011;
                commandF0.outdoor_unit_error_code = data[10];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdF1:
                commandF1.outdoor_unit_EEV_A = (data[4] * 256) + data[5];
                commandF1.outdoor_unit_EEV_B = (data[6] * 256) + data[7];
                commandF1.outdoor_unit_EEV_C = (data[8] * 256) + data[9];
                commandF1.outdoor_unit_EEV_D = (data[10] * 256) + data[11];
                return {DecodeResultType::Processed, 14};

            case NonNasaCommand::CmdF3:
                commandF3.inverter_max_frequency_hz = data[4];
                commandF3.inverter_total_capacity_requirement_kw = (float)data[5] / 10;
                commandF3.inverter_current_a = (float)data[8] / 10;
                commandF3.inverter_voltage_v = (float)data[9] * 2;
                commandF3.inverter_power_w = commandF3.inverter_current_a * 0.1f * commandF3.inverter_voltage_v;
                return {DecodeResultType::Processed, 14};

            default:
                commandRaw.length = 14 - 4 - 1;
                {
                    auto begin = data.begin() + 4;
                    std::copy(begin, begin + commandRaw.length, commandRaw.data);
                }
                return {DecodeResultType::Processed, 14};
            }
        }

        uint8_t encode_request_mode(NonNasaMode value)
        {
            switch (value)
            {
            case NonNasaMode::Auto:
                return 0;
            case NonNasaMode::Cool:
                return 1;
            case NonNasaMode::Dry:
                return 2;
            case NonNasaMode::Fan:
                return 3;
            case NonNasaMode::Heat:
                return 4;
                // NORMALVENT: 7
                // EXCHANGEVENT: 15
                // AIRFRESH: 23
                // SLEEP: 31
                // AUTOVENT: 79

            default:
                return 0; // Auto
            }
        }

        uint8_t encode_request_fanspeed(NonNasaFanspeed value)
        {
            switch (value)
            {
            case NonNasaFanspeed::Auto:
                return 0;
            case NonNasaFanspeed::Low:
                return 64;
            case NonNasaFanspeed::Medium:
                return 128;
            case NonNasaFanspeed::Fresh:
            case NonNasaFanspeed::High:
                return 160;
            default:
                return 0; // Auto
            }
        }

        NonNasaWindDirection swingmode_to_wind_direction(SwingMode swing)
        {
            switch (swing)
            {
            case SwingMode::Fix:
                return NonNasaWindDirection::Stop;
            case SwingMode::Vertical:
                return NonNasaWindDirection::Vertical;
            case SwingMode::Horizontal:
                return NonNasaWindDirection::Horizontal;
            case SwingMode::All:
                return NonNasaWindDirection::FourWay;
            default:
                return NonNasaWindDirection::Stop;
            }
        }

        uint8_t encode_request_wind_direction(NonNasaWindDirection wind_dir)
        {
            switch (wind_dir)
            {
            case NonNasaWindDirection::Stop:
                return 0x1F;
            case NonNasaWindDirection::Vertical:
                return 0x1A;
            case NonNasaWindDirection::Horizontal:
                return 0x1B;
            case NonNasaWindDirection::FourWay:
                return 0x1C;
            default:
                return 0x1F; // Default: swing off
            }
        }

        std::vector<uint8_t> NonNasaRequest::encode()
        {
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xD0,                     // 01 src
                (uint8_t)hex_to_int(dst), // 02 dst
                0xB0,                     // 03 cmd
                0x1F,                     // 04 swing
                0x04,                     // 05 ?
                0,                        // 06 temp + fanmode
                0,                        // 07 operation mode
                0,                        // 08 power + individual mode
                0,                        // 09
                0,                        // 10
                0,                        // 11
                0,                        // 12 crc
                0x34                      // 13 end
            };

            // individual seems to deactivate the locale remotes with message "CENTRAL".
            // seems to be like a building management system.
            bool individual = false;

            data[4] = encode_request_wind_direction(wind_direction);
            if (room_temp > 0)
                data[5] = room_temp;
            data[6] = (target_temp & 31U) | encode_request_fanspeed(fanspeed);
            data[7] = (uint8_t)encode_request_mode(mode);
            data[8] = !power ? (uint8_t)0xC0 : (uint8_t)0xF0;
            data[8] |= (individual ? 6U : 4U);
            data[9] = (uint8_t)0x21;
            data[12] = build_checksum(data);

            return data;
        }

        // Helper function to encode 0xC9 (Clean) command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_clean_command(const std::string &dst, bool clean_on)
        {
            // Based on log analysis: Clean OFF = 0x00, Clean ON = 0x01 in Byte 4
            // Example: 00114075a6740001f6 (OFF) or 011729706cf30001d0 (ON)
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0xC9,                     // 03 cmd (Clean)
                (uint8_t)(clean_on ? 0x01 : 0x00),   // 04 Clean state (0x00=OFF, 0x01=ON)
                0x11,                     // 05 (from example)
                0x40,                     // 06 (from example)
                0x75,                     // 07 (from example)
                0xa6,                     // 08 (from example)
                0x74,                     // 09 (from example)
                0x00,                     // 10 (from example)
                0x01,                     // 11 (from example)
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            data[12] = build_checksum(data);
            return data;
        }

        // Helper function to encode 0x89 (Beep) toggle command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_beep_command(const std::string &dst)
        {
            // Based on log analysis: Beep is a toggle, same data always: 2f000000000000006e
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0x89,                     // 03 cmd (Beep)
                0x2F,                     // 04 (from example)
                0x00,                     // 05
                0x00,                     // 06
                0x00,                     // 07
                0x00,                     // 08
                0x00,                     // 09
                0x00,                     // 10
                0x00,                     // 11
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            // Note: checksum in example is 0x6e, but we'll calculate it
            data[12] = build_checksum(data);
            return data;
        }

        // Helper function to encode 0x82 (Display) command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_display_command(const std::string &dst, bool display_on)
        {
            // Based on log analysis: Display ON = 0x01, Display OFF = 0x02 in Byte 4
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0x82,                     // 03 cmd (Display)
                (uint8_t)(display_on ? 0x01 : 0x02), // 04 Display state (0x01=ON, 0x02=OFF)
                0x00,                     // 05
                0x00,                     // 06
                0x00,                     // 07
                0x00,                     // 08
                0x00,                     // 09
                0x00,                     // 10
                0x00,                     // 11
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            data[12] = build_checksum(data);
            return data;
        }

        // Helper function to encode 0xA9 (Filter Reset) command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_filter_reset_command(const std::string &dst)
        {
            // Based on log analysis: Filter Reset has Byte 8 = 0x80 (bit 7 set)
            // Example: 0000001480000000f5
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0xA9,                     // 03 cmd (Filter Reset)
                0x00,                     // 04
                0x00,                     // 05
                0x00,                     // 06
                0x14,                     // 07 (from example)
                0x80,                     // 08 (bit 7 set = 0x80) - Filter Reset flag
                0x00,                     // 09
                0x00,                     // 10
                0x00,                     // 11
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            data[12] = build_checksum(data);
            return data;
        }

        // Helper function to encode 0x80 (Usage Query) command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_usage_query_command(const std::string &dst)
        {
            // Based on log analysis: Usage query has data: 00000050020000001a
            // From log: 32c8008000000050020000001a34
            // Byte 7 = 0x50, Byte 8 = 0x02, Byte 9 = 0x00
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0x80,                     // 03 cmd (Usage Query)
                0x00,                     // 04
                0x00,                     // 05
                0x00,                     // 06
                0x50,                     // 07 (from example: 0x50)
                0x02,                     // 08 (from example: 0x02)
                0x00,                     // 09 (from example: 0x00)
                0x00,                     // 10
                0x00,                     // 11
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            data[12] = build_checksum(data);
            return data;
        }

        // Helper function to map 0x28 Byte 4 status value to AltMode
        // Since presets are mutually exclusive, we use the last preset sent to disambiguate
        // ambiguous values (0x01, 0x02 can mean different presets)
        
        // Helper function to validate if an AltMode value is a valid preset hint
        // Valid presets: 2 (Quiet), 3 (Fast), 4 (Comfort), 5 (Single User), 10 (SPi)
        static bool is_valid_preset_hint(AltMode hint)
        {
            return hint == 2 || hint == 3 || hint == 4 || hint == 5 || hint == 10;
        }
        
        // Helper function to get preset name from AltMode value
        // Returns human-readable preset name for logging
        static const char* get_preset_name(AltMode alt_mode)
        {
            static const char* preset_names[] = {"None", "", "Quiet", "Fast", "Comfort", "Single User", "", "", "", "", "SPi"};
            if (alt_mode < sizeof(preset_names)/sizeof(preset_names[0]))
                return preset_names[alt_mode];
            return "Unknown";
        }
        
        // Helper function to get preset hint for an address
        // Returns 0 if no hint exists
        static AltMode get_preset_hint(const std::string& address)
        {
            auto it = last_preset_sent_.find(address);
            if (it != last_preset_sent_.end())
                return it->second;
            return 0;
        }
        
        // Helper function to get valid preset hint for an address
        // Returns true if valid hint exists, false otherwise
        // Sets out_hint to the hint value (or 0 if invalid)
        static bool get_valid_preset_hint(const std::string& address, AltMode& out_hint)
        {
            auto it = last_preset_sent_.find(address);
            if (it != last_preset_sent_.end() && is_valid_preset_hint(it->second))
            {
                out_hint = it->second;
                return true;
            }
            out_hint = 0;
            return false;
        }
        
        // Returns AltMode value (0 = None, 2 = Quiet, 3 = Fast, 4 = Comfort, 5 = Single User, 10 = SPi)
        static AltMode preset_status_to_altmode(uint8_t preset_status, AltMode last_preset_hint = 0)
        {
            // Based on log analysis:
            // 0x00 = None (no preset active)
            // 0x03 = Comfort ON (unique - only Comfort shows 0x03)
            // 0x01, 0x02 = Ambiguous (could be Quiet, Fast, Single User, or SPi)
            // 
            // Patterns observed:
            // - Quiet: 0x00 → 0x01 → 0x02
            // - Comfort: 0x03 → 0x00
            // - Fast: 0x00 → 0x01 → 0x02
            // - Single User: 0x01 → 0x02
            // - SPi: 0x01 → 0x02 → 0x03
            
            if (preset_status == 0x00)
            {
                return 0; // None
            }
            
            if (preset_status == 0x03)
            {
                return 4; // Comfort (unique - only Comfort shows 0x03)
            }
            
            // For ambiguous values (0x01, 0x02), use last preset hint if available
            // This works because presets are mutually exclusive - only one can be active
            if (preset_status == 0x01 || preset_status == 0x02)
            {
                // If we have a valid hint from the last preset sent, use it
                if (last_preset_hint > 0 && is_valid_preset_hint(last_preset_hint))
                {
                    return last_preset_hint;
                }
                
                // Without a valid hint, we can't determine which preset is active
                // Return 0 (None) to be safe - status will update when we send a preset command
                return 0;
            }
            
            // Unknown status value
            return 0;
        }

        // Helper function to map 0xF5 Byte 4 value to AltMode (reverse of altmode_to_f5_byte4)
        // Used when we receive 0xF5 commands from remote control or other sources
        // Returns AltMode value (0 if not a valid preset)
        static AltMode f5_byte4_to_altmode(uint8_t byte4)
        {
            // Based on FEATURE_MAPPING.md analysis:
            // Quiet: 0x0c (OFF) / 0x0d (ON) - AltMode value 2
            // Fast: 0x37 (OFF) / 0x38 (ON) - AltMode value 3
            // Comfort: 0x20 (OFF) / 0x21 (ON) - AltMode value 4
            // Single User: 0x46 (OFF) / 0x47 (ON) - AltMode value 5
            // SPi (S-plasma Ion): 0x55 (OFF) / 0x56 (ON) - AltMode value 10
            
            switch (byte4)
            {
            case 0x0c:
            case 0x0d:
                return 2; // Quiet
            case 0x37:
            case 0x38:
                return 3; // Fast
            case 0x20:
            case 0x21:
                return 4; // Comfort
            case 0x46:
            case 0x47:
                return 5; // Single User
            case 0x55:
            case 0x56:
                return 10; // SPi (S-plasma Ion)
            default:
                return 0; // Unknown or not a preset command
            }
        }

        // Helper function to map AltMode value to 0xF5 Byte 4 value
        // Returns the Byte 4 value for the preset mode (OFF state = lower value, ON state = higher value)
        // AltMode values from __init__.py PRESETS:
        //   "quiet": 2, "fast": 3, "comfort": 4, "singleuser": 5, "spi": 10
        static uint8_t altmode_to_f5_byte4(AltMode alt_mode, bool on)
        {
            // Based on analysis:
            // Quiet: 0x0c (OFF) / 0x0d (ON) - AltMode value 2
            // Fast: 0x37 (OFF) / 0x38 (ON) - AltMode value 3
            // Comfort: 0x20 (OFF) / 0x21 (ON) - AltMode value 4
            // Single User: 0x46 (OFF) / 0x47 (ON) - AltMode value 5
            // SPi (S-plasma Ion): 0x55 (OFF) / 0x56 (ON) - AltMode value 10
            
            switch (alt_mode)
            {
            case 2: // Quiet
                return on ? 0x0d : 0x0c;
            case 3: // Fast
                return on ? 0x38 : 0x37;
            case 4: // Comfort
                return on ? 0x21 : 0x20;
            case 5: // Single User
                return on ? 0x47 : 0x46;
            case 10: // SPi (S-plasma Ion)
                return on ? 0x56 : 0x55;
            default:
                return 0x00; // Unknown or not a 0xF5 preset
            }
        }

        // Helper function to get Byte 5 value for 0xF5 preset command
        // Byte 5 varies by preset type and state (ON/OFF)
        static uint8_t get_f5_byte5(AltMode alt_mode, bool on)
        {
            // Based on actual packet analysis from FEATURE_MAPPING.md:
            switch (alt_mode)
            {
            case 2: // Quiet
                return on ? 0x0f : 0x46;
            case 3: // Fast
                return on ? 0x04 : 0x02;
            case 4: // Comfort
                return on ? 0x13 : 0x01;
            case 5: // Single User
                return on ? 0x1e : 0x01;
            case 10: // SPi (S-plasma Ion)
                return on ? 0x3b : 0x64;
            default:
                return 0x01; // Default fallback
            }
        }

        // Helper function to get Byte 10-11 values for 0xF5 preset command
        // These bytes vary by preset type and state
        static std::pair<uint8_t, uint8_t> get_f5_bytes_10_11(AltMode alt_mode, bool on)
        {
            // Based on actual packet analysis from FEATURE_MAPPING.md:
            switch (alt_mode)
            {
            case 2: // Quiet
                return on ? std::make_pair(0x00, 0x11) : std::make_pair(0x00, 0xeb);
            case 3: // Fast
                return on ? std::make_pair(0x00, 0x66) : std::make_pair(0x00, 0x3c);
            case 4: // Comfort
                return on ? std::make_pair(0x1b, 0x15) : std::make_pair(0x1b, 0x16);
            case 5: // Single User
                return on ? std::make_pair(0x00, 0x00) : std::make_pair(0x00, 0xa7);
            case 10: // SPi (S-plasma Ion)
                return on ? std::make_pair(0x00, 0x8a) : std::make_pair(0x00, 0x29);
            default:
                return std::make_pair(0x00, 0x00); // Default fallback
            }
        }

        // Helper function to encode 0xF5 (Preset Mode) command
        // Direction: c8 (outdoor) → 00 (indoor)
        static std::vector<uint8_t> encode_preset_command(const std::string &dst, AltMode alt_mode, bool on)
        {
            // Based on log analysis: 0xF5 with Byte 4 encoding the preset and state
            // Bytes 5, 10, and 11 vary by preset type and state
            uint8_t byte4 = altmode_to_f5_byte4(alt_mode, on);
            uint8_t byte5 = get_f5_byte5(alt_mode, on);
            auto bytes_10_11 = get_f5_bytes_10_11(alt_mode, on);
            
            std::vector<uint8_t> data{
                0x32,                     // 00 start
                0xC8,                     // 01 src (outdoor unit)
                (uint8_t)hex_to_int(dst), // 02 dst (indoor unit)
                0xF5,                     // 03 cmd (Preset)
                byte4,                   // 04 Preset type and state
                byte5,                   // 05 Varies by preset and state
                0x6e,                     // 06 Constant (from all examples)
                0x64,                     // 07 Constant (from all examples)
                0x5a,                     // 08 Constant (from all examples)
                0x3e,                     // 09 Constant (from all examples)
                bytes_10_11.first,        // 10 Varies by preset and state
                bytes_10_11.second,       // 11 Varies by preset and state
                0,                        // 12 crc (will be calculated)
                0x34                      // 13 end
            };
            data[12] = build_checksum(data);
            return data;
        }

        NonNasaRequest NonNasaRequest::create(std::string dst_address)
        {
            NonNasaRequest request;
            request.dst = dst_address;

            auto it = last_command20s_.find(dst_address);
            if (it != last_command20s_.end())
            {
                auto &last_command20_ = it->second;
                request.room_temp = last_command20_.room_temp;
                request.power = last_command20_.power;
                request.target_temp = last_command20_.target_temp;
                request.fanspeed = last_command20_.fanspeed;
                request.mode = last_command20_.mode;
                request.wind_direction = last_command20_.wind_direction;
            }

            return request;
        }

        NonNasaMode mode_to_nonnasa_mode(Mode value)
        {
            switch (value)
            {
            case Mode::Auto:
                return NonNasaMode::Auto;
            case Mode::Cool:
                return NonNasaMode::Cool;
            case Mode::Dry:
                return NonNasaMode::Dry;
            case Mode::Fan:
                return NonNasaMode::Fan;
            case Mode::Heat:
                return NonNasaMode::Heat;
            default:
                return NonNasaMode::Auto;
            }
        }

        NonNasaFanspeed fanmode_to_nonnasa_fanspeed(FanMode value)
        {
            switch (value)
            {
            case FanMode::High:
                return NonNasaFanspeed::High;
            case FanMode::Mid:
                return NonNasaFanspeed::Medium;
            case FanMode::Low:
                return NonNasaFanspeed::Low;
            case FanMode::Auto:
            default:
                return NonNasaFanspeed::Auto;
            }
        }

        void NonNasaProtocol::publish_request(MessageTarget *target, const std::string &address, ProtocolRequest &request)
        {
            // Handle feature commands first (these are sent directly, not queued)
            if (request.automatic_cleaning.has_value())
            {
                // Send Clean command (0xC9)
                auto clean_cmd = encode_clean_command(address, request.automatic_cleaning.value());
                if (debug_log_messages)
                {
                    LOGD("Sending Clean command (0xC9) to %s: %s (expecting 0x1C or 0x2D response)", 
                        address.c_str(), clean_cmd[4] == 0x01 ? "ON" : "OFF");
                    if (debug_log_undefined_messages)
                    {
                        LOGD("Clean packet: %s", bytes_to_hex(clean_cmd).c_str());
                    }
                }
                target->publish_data(0, std::move(clean_cmd));
                return; // Don't queue a 0xB0 command if we're just setting Clean
            }
            
            if (request.beep.has_value())
            {
                // Send Beep toggle command (0x89)
                // Note: 0x89 is a toggle command - same data always toggles state
                // We send the command and let the status update from 0x1C response
                auto beep_cmd = encode_beep_command(address);
                if (debug_log_messages)
                {
                    LOGD("Sending Beep toggle command (0x89) to %s (expecting 0x20 response, status in 0x1C)", address.c_str());
                    if (debug_log_undefined_messages)
                    {
                        LOGD("Beep packet: %s", bytes_to_hex(beep_cmd).c_str());
                    }
                }
                target->publish_data(0, std::move(beep_cmd));
                return; // Don't queue a 0xB0 command if we're just setting Beep
            }
            
            if (request.display.has_value())
            {
                // Send Display command (0x82)
                auto display_cmd = encode_display_command(address, request.display.value());
                if (debug_log_messages)
                {
                    LOGD("Sending Display command (0x82) to %s: %s (expecting 0x25 acknowledgment, status in 0x21)", 
                        address.c_str(), request.display.value() ? "ON" : "OFF");
                    if (debug_log_undefined_messages)
                    {
                        LOGD("Display packet: %s", bytes_to_hex(display_cmd).c_str());
                    }
                }
                target->publish_data(0, std::move(display_cmd));
                return; // Don't queue a 0xB0 command if we're just setting Display
            }
            
            if (request.filter_reset.has_value() && request.filter_reset.value())
            {
                // Send Filter Reset command (0xA9)
                // This is an action command - send it and don't queue 0xB0
                auto reset_cmd = encode_filter_reset_command(address);
                if (debug_log_messages)
                {
                    LOGD("Sending Filter Reset command (0xA9) to %s (expecting 0x2D or 0x20 response)", address.c_str());
                    if (debug_log_undefined_messages)
                    {
                        LOGD("FilterReset packet: %s", bytes_to_hex(reset_cmd).c_str());
                    }
                }
                target->publish_data(0, std::move(reset_cmd));
                // Don't return - might have other requests too, but filter_reset is action-only
            }
            
            if (request.usage_query.has_value() && request.usage_query.value())
            {
                // Send Usage query command (0x80)
                auto usage_cmd = encode_usage_query_command(address);
                if (debug_log_messages)
                    LOGD("Sending Usage query command (0x80) to %s", address.c_str());
                target->publish_data(0, std::move(usage_cmd));
                // Don't return - usage query doesn't interfere with other commands
            }
            
            auto req = NonNasaRequest::create(address);

            if (request.mode)
            {
                request.power = true; // ensure system turns on when mode is set
                req.mode = mode_to_nonnasa_mode(request.mode.value());
            }

            if (request.power)
                req.power = request.power.value();

            if (request.target_temp)
                req.target_temp = request.target_temp.value();

            if (request.fan_mode)
                req.fanspeed = fanmode_to_nonnasa_fanspeed(request.fan_mode.value());

            if (request.alt_mode)
            {
                // Send Preset command (0xF5) for mutually exclusive modes
                // AltMode value 0 means "None" (no preset), any other value means the preset is active
                AltMode alt_mode_value = request.alt_mode.value();
                if (alt_mode_value > 0)
                {
                    // Validate alt_mode_value before setting hint and sending command
                    if (is_valid_preset_hint(alt_mode_value))
                    {
                        // Check if preset is available in current mode
                        // Presets are only available in Cool and Heat modes (not Auto, Fan, Dry)
                        // Use request.mode if set (mode change), otherwise use current mode from Cmd20
                        NonNasaMode current_mode = NonNasaMode::Auto;
                        bool mode_available = false;
                        if (request.mode.has_value())
                        {
                            // Mode is being changed - use the new mode
                            current_mode = request.mode.value();
                            mode_available = (current_mode == NonNasaMode::Cool || current_mode == NonNasaMode::Heat);
                        }
                        else
                        {
                            // No mode change - check current mode from last Cmd20
                            auto cmd20_it = last_command20s_.find(address);
                            if (cmd20_it != last_command20s_.end())
                            {
                                current_mode = cmd20_it->second.mode;
                                mode_available = (current_mode == NonNasaMode::Cool || current_mode == NonNasaMode::Heat);
                            }
                            else
                            {
                                // No Cmd20 received yet - assume mode is available (will be validated by device)
                                mode_available = true;
                            }
                        }
                        
                        if (!mode_available)
                        {
                            const char* preset_name = get_preset_name(alt_mode_value);
                            // Map NonNasaMode enum to string (enum values: Heat=0x01, Cool=0x02, Dry=0x04, Fan=0x08, Auto_Heat=0x21, Auto=0x22)
                            const char* mode_name = "Unknown";
                            switch (current_mode)
                            {
                                case NonNasaMode::Heat: mode_name = "Heat"; break;
                                case NonNasaMode::Cool: mode_name = "Cool"; break;
                                case NonNasaMode::Dry: mode_name = "Dry"; break;
                                case NonNasaMode::Fan: mode_name = "Fan"; break;
                                case NonNasaMode::Auto_Heat: mode_name = "Auto_Heat"; break;
                                case NonNasaMode::Auto: mode_name = "Auto"; break;
                                default: mode_name = "Unknown"; break;
                            }
                            LOGW("Preset %s (AltMode=%d) is not available in %s mode - presets are only available in Cool and Heat modes. Not sending preset command.", 
                                preset_name, alt_mode_value, mode_name);
                            // Don't send command - device would reject it anyway
                            // Don't set hint - preset won't be active
                            // UI will show preset selected, but device won't have it (this is a limitation)
                            // TODO: Ideally, UI should disable presets when mode is not Cool/Heat
                            //       This requires UI code changes (not protocol code)
                        }
                        else
                        {
                            // Track the last preset sent BEFORE sending the command to ensure hint is available when 0x28 responses arrive
                            AltMode old_hint = get_preset_hint(address);
                            last_preset_sent_[address] = alt_mode_value;
                            
                            if (debug_log_messages)
                            {
                                const char* preset_name = get_preset_name(alt_mode_value);
                                const char* old_name = get_preset_name(old_hint);
                                LOGD("Preset hint SET for %s: %s (AltMode=%d) [was %s (AltMode=%d)]", 
                                    address.c_str(), preset_name, alt_mode_value, old_name, old_hint);
                            }
                            
                            // Send 0xF5 command with the preset encoded in Byte 4
                            auto preset_cmd = encode_preset_command(address, alt_mode_value, true);
                            if (debug_log_messages)
                            {
                                const char* preset_name = get_preset_name(alt_mode_value);
                                LOGD("Sending Preset command (0xF5) to %s: AltMode=%d (%s), Byte4=0x%02x, Byte5=0x%02x, Byte10=0x%02x, Byte11=0x%02x (expecting 0x20/0x25/0x40/0x1C response, status in 0x28)", 
                                    address.c_str(), alt_mode_value, preset_name, preset_cmd[4], preset_cmd[5], preset_cmd[10], preset_cmd[11]);
                                if (debug_log_undefined_messages)
                                {
                                    LOGD("Preset packet: %s", bytes_to_hex(preset_cmd).c_str());
                                }
                            }
                            target->publish_data(0, std::move(preset_cmd));
                        }
                        }
                    }
                    else
                    {
                        // Always log invalid preset values - this indicates a configuration issue
                        LOGW("Invalid alt_mode_value %d for device %s - not sending preset command (valid values: 2=Quiet, 3=Fast, 4=Comfort, 5=Single User, 10=SPi)", 
                            alt_mode_value, address.c_str());
                        // Don't send command for invalid values - they would cause issues
                        // Invalid values would generate malformed commands (Byte 4 = 0x00) that could confuse the device
                    }
                }
                else
                {
                    // AltMode 0 means no preset - turn off the current preset and clear the hint
                    // This is the only place we explicitly clear hints (user action)
                    AltMode old_hint = get_preset_hint(address);
                    
                    // If we have a valid preset hint, send OFF command to turn it off
                    if (old_hint > 0 && is_valid_preset_hint(old_hint))
                    {
                        // Send 0xF5 command with on=false to turn off the preset
                        auto preset_off_cmd = encode_preset_command(address, old_hint, false);
                        if (debug_log_messages)
                        {
                            const char* old_name = get_preset_name(old_hint);
                            LOGD("Sending Preset OFF command (0xF5) to %s: AltMode=%d (%s), Byte4=0x%02x, Byte5=0x%02x, Byte10=0x%02x, Byte11=0x%02x (expecting 0x20/0x25/0x40/0x1C response, status in 0x28)", 
                                address.c_str(), old_hint, old_name, preset_off_cmd[4], preset_off_cmd[5], preset_off_cmd[10], preset_off_cmd[11]);
                            if (debug_log_undefined_messages)
                            {
                                LOGD("Preset OFF packet: %s", bytes_to_hex(preset_off_cmd).c_str());
                            }
                        }
                        target->publish_data(0, std::move(preset_off_cmd));
                    }
                    
                    if (debug_log_messages)
                    {
                        const char* old_name = get_preset_name(old_hint);
                        if (old_hint > 0)
                        {
                            LOGD("Preset hint CLEARED for %s (AltMode=0) [was %s (AltMode=%d)]", 
                                address.c_str(), old_name, old_hint);
                        }
                        else
                        {
                            LOGD("Preset hint CLEARED for %s (AltMode=0) [was None (AltMode=0)]", 
                                address.c_str());
                        }
                    }
                    last_preset_sent_[address] = 0;
                }
            }

            if (request.swing_mode)
            {
                NonNasaWindDirection wind_dir = swingmode_to_wind_direction(request.swing_mode.value());
                req.wind_direction = wind_dir;
            }

            // Add to the queue with the current time
            NonNasaRequestQueueItem reqItem = NonNasaRequestQueueItem();
            reqItem.request = req;
            reqItem.time = millis();
            reqItem.time_sent = 0;
            reqItem.retry_count = 0;
            reqItem.resend_count = 0;

            // Safety check the length of the queue (in case something is spamming control
            // requests we don't want the queue to get too large).
            if (nonnasa_requests.size() < 10)
            {
                nonnasa_requests.push_back(reqItem);
            }
        }

        Mode nonnasa_mode_to_mode(NonNasaMode value)
        {
            switch (value)
            {
            case NonNasaMode::Auto:
            case NonNasaMode::Auto_Heat:
                return Mode::Auto;
            case NonNasaMode::Cool:
                return Mode::Cool;
            case NonNasaMode::Dry:
                return Mode::Dry;
            case NonNasaMode::Fan:
                return Mode::Fan;
            case NonNasaMode::Heat:
                return Mode::Heat;
            default:
                return Mode::Auto;
            }
        }

        // TODO
        WaterHeaterMode nonnasa_water_heater_mode_to_mode(int value)
        {
            switch (value)
            {
            default:
                return WaterHeaterMode::Unknown;
            }
        }

        FanMode nonnasa_fanspeed_to_fanmode(NonNasaFanspeed fanspeed)
        {
            switch (fanspeed)
            {
            case NonNasaFanspeed::Fresh:
            case NonNasaFanspeed::High:
                return FanMode::High;
            case NonNasaFanspeed::Medium:
                return FanMode::Mid;
            case NonNasaFanspeed::Low:
                return FanMode::Low;
            default:
            case NonNasaFanspeed::Auto:
                return FanMode::Auto;
            }
        }

        DecodeResult try_decode_non_nasa_packet(std::vector<uint8_t> &data)
        {
            return nonpacket_.decode(data);
        }

        void send_requests(MessageTarget *target)
        {
            const uint32_t now = millis();
            for (auto &item : nonnasa_requests)
            {
                if (item.time_sent == 0)
                {
                    item.time_sent = now;
                    target->publish_data(0, item.request.encode());
                }
            }
        }

        void send_register_controller(MessageTarget *target)
        {
            LOGD("Sending controller registration request...");

            // Registers our device as a "controller" with the outdoor unit. This will cause the
            // outdoor unit to poll us with a request_control message approximately every second,
            // which we can reply to with a control message if required.
            std::vector<uint8_t> data{
                0x32, // 00 start
                0xD0, // 01 src
                0xc8, // 02 dst
                0xD1, // 03 cmd (register_device)
                0xD2, // 04 device_type (controller)
                0,    // 05
                0,    // 06
                0,    // 07
                0,    // 08
                0,    // 09
                0,    // 10
                0,    // 11
                0,    // 12 crc
                0x34  // 13 end
            };
            data[12] = build_checksum(data);

            // Send now
            target->publish_data(0, std::move(data));
        }

        void process_non_nasa_packet(MessageTarget *target)
        {
            if (debug_log_undefined_messages)
            {
                LOG_PACKET_RECV("RECV", nonpacket_);
            }

            target->register_address(nonpacket_.src);

            // Check if we have a message from the indoor unit. If so, we can assume it is awake.
            if (!indoor_unit_awake && get_address_type(nonpacket_.src) == AddressType::Indoor)
            {
                indoor_unit_awake = true;
            }

            if (nonpacket_.cmd == NonNasaCommand::Cmd20)
            {
                // We may occasionally not receive a control_acknowledgement message when sending a control
                // packet, so as a backup approach check if the state of the device matches that of the
                // sent control packet. This also serves as a backup approach if for some reason a device
                // doesn't send control_acknowledgement messages at all.
                LOGD("Cmd20 received: src=%s, wind_direction=%d, target_temp=%d, power=%d, mode=%d, fanspeed=%d",
                     nonpacket_.src.c_str(),
                     (uint8_t)nonpacket_.command20.wind_direction,
                     nonpacket_.command20.target_temp,
                     nonpacket_.command20.power,
                     (uint8_t)nonpacket_.command20.mode,
                     (uint8_t)nonpacket_.command20.fanspeed);

                size_t before_size = nonnasa_requests.size();
                nonnasa_requests.remove_if([&](const NonNasaRequestQueueItem &item)
                                           { return item.time_sent > 0 &&
                                                    nonpacket_.src == item.request.dst &&
                                                    item.request.target_temp == nonpacket_.command20.target_temp &&
                                                    item.request.fanspeed == nonpacket_.command20.fanspeed &&
                                                    item.request.mode == nonpacket_.command20.mode &&
                                                    item.request.power == nonpacket_.command20.power &&
                                                    item.request.wind_direction == nonpacket_.command20.wind_direction; });
                size_t after_size = nonnasa_requests.size();
                if (before_size != after_size)
                {
                    LOGD("Cmd20: Removed %zu matching request(s) for %s (backup ack)",
                         before_size - after_size, nonpacket_.src.c_str());
                }

                // If a state update comes through after a control message has been sent, but before it
                // has been acknowledged, it should be ignored. This prevents the UI status bouncing
                // between states after a command has been issued.
                bool pending_control_message = false;
                for (auto &item : nonnasa_requests)
                {
                    if (item.time_sent > 0 && nonpacket_.src == item.request.dst)
                    {
                        pending_control_message = true;
                        break;
                    }
                }

                // Publish EVA (evaporator) temperatures - pipe_in/pipe_out are equivalent to eva_in/eva_out
                // These are sensor readings and should always be published, regardless of pending control messages
                // Compare to CmdC0 and Cmd8D handlers which explicitly do not check for pending control messages
                // Cast to int8_t first to preserve sign (uint8_t wraps negative values), then to float
                float pipe_in_temp = static_cast<float>(static_cast<int8_t>(nonpacket_.command20.pipe_in));
                float pipe_out_temp = static_cast<float>(static_cast<int8_t>(nonpacket_.command20.pipe_out));
                target->set_indoor_eva_in_temperature(nonpacket_.src, pipe_in_temp);
                target->set_indoor_eva_out_temperature(nonpacket_.src, pipe_out_temp);

                if (!pending_control_message)
                {
                    last_command20s_[nonpacket_.src] = nonpacket_.command20;
                    target->set_target_temperature(nonpacket_.src, nonpacket_.command20.target_temp);
                    // TODO
                    target->set_water_outlet_target(nonpacket_.src, false);
                    // TODO
                    target->set_target_water_temperature(nonpacket_.src, false);
                    target->set_room_temperature(nonpacket_.src, nonpacket_.command20.room_temp);
                    target->set_power(nonpacket_.src, nonpacket_.command20.power);
                    // TODO
                    target->set_water_heater_power(nonpacket_.src, false);
                    target->set_mode(nonpacket_.src, nonnasa_mode_to_mode(nonpacket_.command20.mode));
                    // TODO
                    target->set_water_heater_mode(nonpacket_.src, nonnasa_water_heater_mode_to_mode(-0));
                    target->set_fanmode(nonpacket_.src, nonnasa_fanspeed_to_fanmode(nonpacket_.command20.fanspeed));
                    // Cmd20 does NOT contain preset information - only Cmd28 does
                    // Don't clear preset here - use hint if available, otherwise leave unchanged
                    // This prevents Cmd20 from clearing the preset when switching presets
                    AltMode hint_value = 0;
                    if (get_valid_preset_hint(nonpacket_.src, hint_value))
                    {
                        // Use hint to maintain preset state (Cmd20 arrives frequently and shouldn't clear preset)
                        if (debug_log_messages)
                        {
                            const char* preset_name = get_preset_name(hint_value);
                            LOGD("Cmd20 from %s: Using hint to maintain preset state: %s (AltMode=%d) [Cmd20 doesn't contain preset info, using hint from last preset sent]", 
                                nonpacket_.src.c_str(), preset_name, hint_value);
                        }
                        target->set_altmode(nonpacket_.src, hint_value);
                    }
                    else if (debug_log_messages)
                    {
                        AltMode raw_hint = get_preset_hint(nonpacket_.src);
                        if (raw_hint == 0)
                        {
                            LOGD("Cmd20 from %s: No hint available - leaving preset unchanged (Cmd20 doesn't contain preset info, let Cmd28 handle it)", 
                                nonpacket_.src.c_str());
                        }
                        else
                        {
                            LOGD("Cmd20 from %s: Invalid hint=%d - leaving preset unchanged (Cmd20 doesn't contain preset info, let Cmd28 handle it)", 
                                nonpacket_.src.c_str(), raw_hint);
                        }
                    }
                    // If no hint, don't change preset - let Cmd28 handle it
                    // Cmd20 swing decode: converting wind_direction to vertical/horizontal booleans
                    target->set_swing_horizontal(nonpacket_.src,
                                                 (nonpacket_.command20.wind_direction == NonNasaWindDirection::Horizontal) ||
                                                     (nonpacket_.command20.wind_direction == NonNasaWindDirection::FourWay));
                    target->set_swing_vertical(nonpacket_.src,
                                               (nonpacket_.command20.wind_direction == NonNasaWindDirection::Vertical) ||
                                                   (nonpacket_.command20.wind_direction == NonNasaWindDirection::FourWay));
                }
            }
            else if (nonpacket_.cmd == NonNasaCommand::CmdC0)
            {
                // CmdC0 comes from the outdoor unit and contains outdoor temperature
                // The temperature is already in Celsius (after subtracting 55 from raw value)
                // Note: No pending control message check needed here since CmdC0 comes from the
                // outdoor unit (typically "c8"), while control messages are sent to indoor units.
                // Outdoor temperature updates are independent status data and should always be processed.
                // Cast to int8_t first to preserve sign (uint8_t wraps negative values), then to float
                float temp = static_cast<float>(static_cast<int8_t>(nonpacket_.commandC0.outdoor_unit_outdoor_temp_c));
                target->set_outdoor_temperature(nonpacket_.src, temp);
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd8D)
            {
                // Cmd8D comes from the outdoor unit and contains power/energy sensor data
                // Note: No pending control message check needed here since Cmd8D comes from the
                // outdoor unit (typically "c8"), while control messages are sent to indoor units.
                // Outdoor power/energy updates are independent status data and should always be processed.
                // Note: Following NASA protocol approach - publish raw current value, sensor filter will apply.
                target->set_outdoor_instantaneous_power(nonpacket_.src, nonpacket_.command8D.inverter_power_w);
                target->set_outdoor_current(nonpacket_.src, nonpacket_.command8D.inverter_current_a);
                target->set_outdoor_voltage(nonpacket_.src, nonpacket_.command8D.inverter_voltage_v);

                // Calculate cumulative energy by integrating power over time using trapezoidal rule
                // The helper function handles all edge cases: wraparound, first update, time delta validation
                CumulativeEnergyTracker &tracker = cumulative_energy_trackers_[nonpacket_.src];
                const uint32_t now = millis();
                update_cumulative_energy_tracker(tracker, nonpacket_.command8D.inverter_power_w, now);

                // Publish cumulative energy
                // Sensor has filter multiply: 0.001 and unit is kWh
                // NASA protocol publishes raw value in Wh, filter converts to kWh
                // So we publish in Wh (accumulated_energy_kwh * 1000), filter converts to kWh
                // Convert from double to float for API (API requires float)
                float cumulative_energy_wh = static_cast<float>(tracker.accumulated_energy_kwh * 1000.0);
                target->set_outdoor_cumulative_energy(nonpacket_.src, cumulative_energy_wh);
            }
            else if (nonpacket_.cmd == NonNasaCommand::CmdF0)
            {
                // CmdF0 comes from the outdoor unit and contains error code and status information
                // Note: No pending control message check needed here since CmdF0 comes from the
                // outdoor unit (typically "c8"), while control messages are sent to indoor units.
                // Outdoor error code updates are independent status data and should always be processed.
                int error_code = static_cast<int>(nonpacket_.commandF0.outdoor_unit_error_code);
                if (debug_log_messages && error_code != 0)
                {
                    LOGW("s:%s d:%s CmdF0 outdoor_unit_error_code %d", nonpacket_.src.c_str(), nonpacket_.dst.c_str(), error_code);
                }
                target->set_error_code(nonpacket_.src, error_code);
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd1C)
            {
                // Cmd1C comes from the indoor unit and contains feature status
                // Byte 4: Bit 0 = Beep, Bit 3 = Clean, etc.
                // Note: No pending control message check needed - feature status is independent
                uint8_t status_byte = nonpacket_.command1C.feature_status_byte;
                
                // Extract Clean status (Bit 3)
                bool clean_on = (status_byte & 0x08) != 0;
                if (debug_log_messages)
                    LOGD("Cmd1C from %s: Clean=%s (status_byte=0x%02x, bit3=%d)", 
                        nonpacket_.src.c_str(), clean_on ? "ON" : "OFF", status_byte, (status_byte >> 3) & 1);
                target->set_automatic_cleaning(nonpacket_.src, clean_on);
                
                // Extract Beep status (Bit 0)
                bool beep_on = (status_byte & 0x01) != 0;
                if (debug_log_messages)
                    LOGD("Cmd1C from %s: Beep=%s (status_byte=0x%02x, bit0=%d)", 
                        nonpacket_.src.c_str(), beep_on ? "ON" : "OFF", status_byte, status_byte & 1);
                target->set_beep(nonpacket_.src, beep_on);
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd21)
            {
                // Cmd21 comes from the indoor unit and contains Display status
                // Byte 7: Display ON/OFF status (tentative - needs confirmation)
                // Based on analysis: Byte 7 might indicate Display state
                // For now, extract and publish (needs confirmation of exact meaning)
                uint8_t display_status = nonpacket_.command21.display_status;
                // TODO: Confirm Byte 7 meaning - might be 0x01=ON, 0x02=OFF or bit-encoded
                // For now, assume non-zero means ON (needs verification)
                bool display_on = (display_status != 0);
                if (debug_log_messages)
                {
                    LOGD("Cmd21 from %s: Display=%s (byte7=0x%02x)", 
                        nonpacket_.src.c_str(), display_on ? "ON" : "OFF", display_status);
                    // Log if we see unexpected values for analysis
                    if (display_status != 0x00 && display_status != 0x01 && display_status != 0x02 && display_status != 0x06)
                    {
                        LOGD("Cmd21 from %s: Unusual Display status value: 0x%02x (expected 0x00, 0x01, 0x02, or 0x06)", 
                            nonpacket_.src.c_str(), display_status);
                    }
                }
                target->set_display(nonpacket_.src, display_on);
            }
            else if (nonpacket_.cmd == NonNasaCommand::CmdF5)
            {
                // CmdF5 is a preset control command that can be sent by us or the remote control
                // When we receive it (from remote or echoed back), we extract the preset type from Byte 4
                // and use it as a hint to resolve ambiguous 0x28 status responses
                uint8_t preset_byte4 = nonpacket_.commandF5.preset_byte4;
                AltMode preset_altmode = f5_byte4_to_altmode(preset_byte4);
                
                if (preset_altmode > 0 && is_valid_preset_hint(preset_altmode))
                {
                    // Set hint so we can resolve ambiguous 0x28 responses
                    // This works for both commands we send (echoed back) and remote control commands
                    // Note: We use dst (destination) because 0xF5 goes TO the indoor unit
                    //       Cmd28 uses src (source) because it comes FROM the indoor unit
                    //       These should be the same address (indoor unit)
                    AltMode old_hint = get_preset_hint(nonpacket_.dst);
                    last_preset_sent_[nonpacket_.dst] = preset_altmode;
                    
                    // Log at INFO level since this is a new feature that enables remote control detection
                    // This helps users verify it's working even without debug logging
                    const char* preset_name = get_preset_name(preset_altmode);
                    const char* old_name = get_preset_name(old_hint);
                    
                    // Determine if this is likely from remote control (src != "c8") or our own command (src == "c8")
                    bool likely_remote = (nonpacket_.src != "c8");
                    const char* source_type = likely_remote ? "remote control" : "our command";
                    
                    LOGI("CmdF5 from %s to %s: Detected preset command Byte4=0x%02x -> AltMode=%d (%s) [%s, hint updated from %s (AltMode=%d)]", 
                        nonpacket_.src.c_str(), nonpacket_.dst.c_str(), preset_byte4, preset_altmode, preset_name, 
                        source_type, old_name, old_hint);
                    
                    // Debug-level detail logging
                    if (debug_log_messages)
                    {
                        LOGD("CmdF5 hint stored for device %s: AltMode=%d (%s) - will be used to resolve ambiguous 0x28 responses", 
                            nonpacket_.dst.c_str(), preset_altmode, preset_name);
                    }
                }
                else
                {
                    // Log invalid preset commands at DEBUG level only
                    if (debug_log_messages)
                    {
                        LOGD("CmdF5 from %s to %s: Byte4=0x%02x -> AltMode=%d (not a valid preset, ignoring)", 
                            nonpacket_.src.c_str(), nonpacket_.dst.c_str(), preset_byte4, preset_altmode);
                    }
                }
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd28)
            {
                // Cmd28 comes from the indoor unit and contains preset/mode status
                // Byte 4: Current preset/mode status
                // Map preset_status to AltMode value
                uint8_t preset_status = nonpacket_.command28.preset_status;
                
                // Get last preset sent for this device to help disambiguate ambiguous values
                // Note: We use src (source) because 0x28 comes FROM the indoor unit
                //       CmdF5 uses dst (destination) because it goes TO the indoor unit
                //       These should be the same address (indoor unit)
                //       However, we check both src and dst as fallback for robustness
                AltMode last_preset_hint = get_preset_hint(nonpacket_.src);
                bool hint_exists = (last_preset_hint > 0);
                
                if (!hint_exists)
                {
                    // Fallback: try dst if src doesn't have hint (address mismatch case)
                    // This handles edge cases where addresses might differ
                    AltMode dst_hint = get_preset_hint(nonpacket_.dst);
                    if (dst_hint > 0)
                    {
                        last_preset_hint = dst_hint;
                        hint_exists = true;
                        if (debug_log_messages && nonpacket_.src != nonpacket_.dst)
                        {
                            LOGW("Cmd28 from %s: Using dst hint (AltMode=%d) - src hint not found (address mismatch: src=%s, dst=%s)", 
                                nonpacket_.src.c_str(), last_preset_hint, nonpacket_.src.c_str(), nonpacket_.dst.c_str());
                        }
                    }
                }
                
                // Warn if address mismatch when both hints exist (shouldn't happen, but helps debug)
                if (debug_log_messages && hint_exists && nonpacket_.src != nonpacket_.dst)
                {
                    AltMode dst_hint = get_preset_hint(nonpacket_.dst);
                    if (dst_hint > 0 && dst_hint != last_preset_hint)
                    {
                        // Both src and dst have hints - warn about mismatch
                        LOGW("Cmd28 from %s: Address mismatch detected - hint exists for both src=%s (AltMode=%d) and dst=%s (AltMode=%d), using src hint", 
                            nonpacket_.src.c_str(), nonpacket_.src.c_str(), last_preset_hint, nonpacket_.dst.c_str(), dst_hint);
                    }
                }
                
                if (debug_log_messages && (preset_status == 0x01 || preset_status == 0x02))
                {
                    const char* hint_name = get_preset_name(last_preset_hint);
                    LOGD("Cmd28 from %s: Received ambiguous preset_status=0x%02x [hint: %s (AltMode=%d), exists=%s]", 
                        nonpacket_.src.c_str(), preset_status, hint_name, last_preset_hint, hint_exists ? "YES" : "NO");
                }
                
                // Map preset_status to AltMode
                AltMode alt_mode = preset_status_to_altmode(preset_status, last_preset_hint);
                
                // If we got ambiguous status (0x01 or 0x02) but couldn't resolve it (alt_mode == 0),
                // and we have a valid hint, log a warning but don't clear the preset - keep using the hint
                // This prevents the UI from clearing when we temporarily can't resolve the status
                // Note: preset_status_to_altmode already validates the hint, so if alt_mode == 0 with
                // last_preset_hint > 0, the hint was invalid. This check is defensive programming.
                bool hint_used_to_resolve = false;
                if ((preset_status == 0x01 || preset_status == 0x02) && alt_mode == 0 && last_preset_hint > 0)
                {
                    if (is_valid_preset_hint(last_preset_hint))
                    {
                        if (debug_log_messages)
                        {
                            const char* hint_name = get_preset_name(last_preset_hint);
                            LOGW("Cmd28 from %s: Ambiguous preset_status=0x%02x with hint=%d (%s) but couldn't resolve - USING HINT to prevent UI clearing", 
                                nonpacket_.src.c_str(), preset_status, last_preset_hint, hint_name);
                        }
                        // Use the hint instead of returning None
                        alt_mode = last_preset_hint;
                        hint_used_to_resolve = true;
                    }
                    // If hint is invalid, don't use it - alt_mode stays 0 and we'll skip calling set_altmode()
                }
                
                // If we got ambiguous status without a valid hint, don't clear the preset - just ignore it
                // This prevents the UI from clearing when we receive unsolicited ambiguous status
                // Note: We check both no hint (== 0) and invalid hint (!is_valid) cases
                bool should_update = true;
                bool has_valid_hint = (last_preset_hint > 0) && is_valid_preset_hint(last_preset_hint);
                
                // Handle ambiguous status (0x01, 0x02) without valid hint
                if ((preset_status == 0x01 || preset_status == 0x02) && alt_mode == 0 && !has_valid_hint)
                {
                    if (debug_log_messages)
                    {
                        if (last_preset_hint == 0)
                        {
                            LOGD("Cmd28 from %s: Ambiguous preset_status=0x%02x without hint - ignoring to prevent clearing preset", 
                                nonpacket_.src.c_str(), preset_status);
                        }
                        else
                        {
                            LOGD("Cmd28 from %s: Ambiguous preset_status=0x%02x with invalid hint=%d - ignoring to prevent clearing preset", 
                                nonpacket_.src.c_str(), preset_status, last_preset_hint);
                        }
                    }
                    should_update = false;
                }
                
                // Handle 0x00 (None) status when we have a valid hint
                // 0x00 might be a transition state when switching presets, so ignore it if we're expecting a preset
                if (preset_status == 0x00 && has_valid_hint)
                {
                    if (debug_log_messages)
                    {
                        const char* hint_name = get_preset_name(last_preset_hint);
                        LOGD("Cmd28 from %s: Received 0x00 (None) but have valid hint=%d (%s) - ignoring to prevent clearing preset (might be transition state)", 
                            nonpacket_.src.c_str(), last_preset_hint, hint_name);
                    }
                    should_update = false;
                }
                
                if (debug_log_messages)
                {
                    const char* preset_name = get_preset_name(alt_mode);
                    if (preset_status == 0x01 || preset_status == 0x02)
                    {
                        const char* resolution_method = "unknown";
                        if (hint_used_to_resolve)
                        {
                            resolution_method = "hint used to resolve";
                        }
                        else if (has_valid_hint && alt_mode > 0)
                        {
                            resolution_method = "hint used in mapping";
                        }
                        else if (!has_valid_hint && alt_mode == 0)
                        {
                            resolution_method = "no hint, cannot resolve";
                        }
                        else if (has_valid_hint && alt_mode == 0)
                        {
                            resolution_method = "hint invalid, cannot resolve";
                        }
                        
                        LOGD("Cmd28 from %s: preset_status=0x%02x -> AltMode=%d (%s) [hint=%d (%s), resolution=%s]", 
                            nonpacket_.src.c_str(), preset_status, alt_mode, preset_name, last_preset_hint,
                            has_valid_hint ? "valid" : "invalid/none", resolution_method);
                    }
                    else
                    {
                        LOGD("Cmd28 from %s: preset_status=0x%02x -> AltMode=%d (%s)", 
                            nonpacket_.src.c_str(), preset_status, alt_mode, preset_name);
                    }
                    
                    // Log state transitions
                    AltMode old_hint = get_preset_hint(nonpacket_.src);
                    if (old_hint > 0 && old_hint != alt_mode)
                    {
                        const char* old_name = get_preset_name(old_hint);
                        LOGD("Cmd28 from %s: Preset state changed from %s (AltMode=%d) to %s (AltMode=%d)", 
                            nonpacket_.src.c_str(), old_name, old_hint, preset_name, alt_mode);
                    }
                }
                
                // Only update the preset if we resolved it or got explicit None (0x00 without hint)
                // Note: 0x00 with a valid hint is ignored as it might be a transition state
                if (should_update)
                {
                    target->set_altmode(nonpacket_.src, alt_mode);
                    
                    // Update last preset tracking if we successfully mapped to a preset
                    // This helps maintain state even if we didn't send the command
                    if (alt_mode > 0)
                    {
                        // Update hint when we get confirmed status (0x03 for Comfort, or resolved ambiguous)
                        AltMode old_hint = get_preset_hint(nonpacket_.src);
                        
                        if (old_hint != alt_mode)
                        {
                            last_preset_sent_[nonpacket_.src] = alt_mode;
                            if (debug_log_messages)
                            {
                                const char* preset_name = get_preset_name(alt_mode);
                                const char* old_name = get_preset_name(old_hint);
                                LOGD("Preset hint UPDATED for %s: %s (AltMode=%d) [was %s (AltMode=%d)] - from status confirmation", 
                                    nonpacket_.src.c_str(), preset_name, alt_mode, old_name, old_hint);
                            }
                        }
                    }
                    // Note: We don't clear hints on 0x00 because:
                    // 1. 0x00 might be a transition state when switching presets
                    // 2. Hints are only used to resolve ambiguous status (0x01/0x02)
                    // 3. Hints are explicitly cleared when user selects AltMode=0
                    // 4. Hints are overwritten when we get confirmed status (alt_mode > 0)
                }
                else
                {
                    // We're not updating (ambiguous status without hint) - preserve the hint
                    // Don't clear tracking here, as we want to keep the hint for future ambiguous status
                }
                
                // Handle unknown status values (outside of should_update check)
                if (preset_status > 0x03)
                {
                    // Unknown preset status value - log warning but preserve hint
                    // Unknown status might be:
                    // 1. A transient error
                    // 2. A future protocol extension
                    // 3. A device-specific quirk
                    // Preserving the hint allows future ambiguous status to still be resolved
                    if (debug_log_messages)
                    {
                        AltMode old_hint = get_preset_hint(nonpacket_.src);
                        if (old_hint > 0)
                        {
                            const char* old_name = get_preset_name(old_hint);
                            LOGW("Cmd28 from %s: Unknown preset_status=0x%02x (preserving hint: %s) - might be transient error or protocol extension", 
                                nonpacket_.src.c_str(), preset_status, old_name);
                        }
                        else
                        {
                            LOGW("Cmd28 from %s: Unknown preset_status=0x%02x (mapped to None, no hint to preserve)", 
                                nonpacket_.src.c_str(), preset_status);
                        }
                    }
                    // Don't clear hint - preserve it for future ambiguous status resolution
                    // Hint will be cleared when:
                    // 1. User explicitly selects None (AltMode=0)
                    // 2. We receive confirmed status (0x00 without hint, or 0x03 for Comfort)
                }
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd2F)
            {
                // Cmd2F comes from the indoor unit and contains usage statistics
                // Byte 9 and Byte 11: Electricity consumption raw values
                // Manual states display shows 0.1-99 kWh, but exact conversion formula is unknown
                // These are raw byte values (0-255) that represent electricity consumption statistics
                uint8_t usage_statistic_1 = nonpacket_.command2F.usage_byte_9;   // Byte 9: Raw statistic
                uint8_t usage_statistic_2 = nonpacket_.command2F.usage_byte_11;  // Byte 11: Raw statistic
                
                if (debug_log_messages)
                    LOGD("Cmd2F from %s: Usage statistic1 (Byte9)=0x%02x (%d), statistic2 (Byte11)=0x%02x (%d)", 
                        nonpacket_.src.c_str(), usage_statistic_1, usage_statistic_1, usage_statistic_2, usage_statistic_2);
                
                // Publish usage statistics as custom sensors
                // Using message numbers that won't conflict with NASA protocol
                target->set_custom_sensor(nonpacket_.src, 0x2F09, static_cast<float>(usage_statistic_1));
                target->set_custom_sensor(nonpacket_.src, 0x2F0B, static_cast<float>(usage_statistic_2));
                
                // Also publish to dedicated sensors if configured
                target->set_usage_statistic_1(nonpacket_.src, static_cast<float>(usage_statistic_1));
                target->set_usage_statistic_2(nonpacket_.src, static_cast<float>(usage_statistic_2));
            }
            else if (nonpacket_.cmd == NonNasaCommand::CmdC6)
            {
                // We have received a request_control message. This is a message outdoor units will
                // send to a registered controller, allowing us to reply with any control commands.
                // Control commands should be sent immediately (per SNET Pro behaviour).
                if (nonpacket_.src == "c8" && nonpacket_.dst == "d0" && nonpacket_.commandC6.control_status == true)
                {
                    if (controller_registered == false)
                    {
                        LOGD("Controller registered");
                        controller_registered = true;
                    }
                    if (indoor_unit_awake)
                    {
                        // We know the outdoor unit is awake due to this request_control message, so we only
                        // need to check that the indoor unit is awake.
                        send_requests(target);
                    }
                }
            }
            else if (nonpacket_.cmd == NonNasaCommand::Cmd54 && nonpacket_.dst == "d0")
            {
                // We have received a control_acknowledgement message. This message will come from an
                // indoor unit in reply to a control message from us, allowing us to confirm the control
                // message was successfully sent. The data portion contains the same data we sent (however
                // we can just assume it's for any sent packet, rather than comparing).
                nonnasa_requests.remove_if([&](const NonNasaRequestQueueItem &item)
                                           { return item.time_sent > 0 && nonpacket_.src == item.request.dst; });
            }
            else if (nonpacket_.src == "c8" && nonpacket_.dst == "ad" && (nonpacket_.commandRaw.data[0] & 1) == 1)
            {
                // We have received a broadcast registration request. It isn't necessary to register
                // more than once, however we can use this as a keepalive method. A 30ms delay is added
                // to allow other controllers to register. This mimics SNET Pro behaviour.
                // It's unknown why the first data byte must be odd.
                if (non_nasa_keepalive)
                {
                    const uint32_t now = millis();
                    // rate limit
                    // Wrap-safe elapsed time (unsigned subtraction works across millis() rollover)
                    const uint32_t elapsed_ms = now - last_keepalive_sent_ms_;

                    if (elapsed_ms >= KEEPALIVE_MIN_INTERVAL_MS)
                    {
                        pending_keepalive_ = true;
                        pending_keepalive_due_ms_ = now + KEEPALIVE_DELAY_MS;
                    }
                }
            }
        }

        void NonNasaProtocol::protocol_update(MessageTarget *target)
        {
            // non-blocking keepalive send (scheduled from broadcast request)
            if (non_nasa_keepalive && pending_keepalive_)
            {
                const uint32_t now = millis();
                if ((int32_t)(now - pending_keepalive_due_ms_) >= 0)
                {
                    send_register_controller(target);
                    last_keepalive_sent_ms_ = now;
                    pending_keepalive_ = false;
                }
            }
            else if (!non_nasa_keepalive)
            {
                pending_keepalive_ = false;
            }

            // If we're not currently registered, keep sending a registration request until it has
            // been confirmed by the outdoor unit.
            if (!controller_registered)
            {
                send_register_controller(target);
            }

            // If we have *any* messages in the queue for longer than 15s, assume failure and
            // remove from queue (the AC or UART connection is likely offline).
            const uint32_t now = millis();
            nonnasa_requests.remove_if([&](const NonNasaRequestQueueItem &item)
                                       { return now - item.time > 15000; });

            // If we have any *sent* messages in the queue that haven't received an ack in under 5s,
            // assume they failed and queue for resend on the next request_control message. Retry at
            // most 3 times.
            for (auto &item : nonnasa_requests)
            {
                if (item.time_sent > 0 && item.resend_count < 3 && now - item.time_sent > 4500)
                {
                    item.time_sent = 0; // Resend
                    item.resend_count++;
                }
            }

            // If we have any *unsent* messages in the queue for over 1000ms, it likely means the indoor
            // and/or outdoor unit has gone to sleep due to inactivity. Send a registration request to
            // wake the unit up.
            for (auto &item : nonnasa_requests)
            {
                if (item.time_sent == 0 && now - item.time > 1000 && item.resend_count == 0 && item.retry_count == 0)
                {
                    // Both the outdoor and the indoor unit must be awake before we can send a command
                    indoor_unit_awake = false;
                    item.retry_count++;
                    LOGD("Device is likely sleeping, waking...");
                    send_register_controller(target);
                    break;
                }
            }
        }
    } // namespace samsung_ac
} // namespace esphome
