// tfmini_s_can_read_main.cpp
//
// Standalone TFmini-S monitor through two LinkX EtherCAT modules.
//
// Wiring for this test:
//   EtherCAT slave 1: physical module 2, CAN0, CAN1, CAN2, CAN3
//   EtherCAT slave 2: physical module 1, CAN2, CAN3
//   Each CAN bus has TFmini-S sensors on standard IDs 0x01 and 0x02.
//
// The TFmini-S manual describes the default UART output as a 9-byte frame:
//   59 59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum
// With the serial-to-CAN module in transparent mode, those UART bytes are
// carried inside CAN/CAN-FD payloads. This tool treats each matching CAN
// payload as a byte stream and reassembles TFmini-S frames per sensor.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "ecat_manager.h"
#include "linkx4c_handler.h"

namespace
{
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint16_t kInvalidStrength = 65535U;
constexpr uint32_t kFirstTargetId = 0x01U;
constexpr uint32_t kLastTargetId = 0x02U;
constexpr size_t kModuleCount = 2;

struct ModuleWatch
{
    uint32_t slave_id;
    std::array<uint8_t, LINKX_CAN_CHANNEL_NUM> channels;
    size_t channel_count;
};

constexpr ModuleWatch kWatchedModules[kModuleCount] = {
    {1U, {0U, 1U, 2U, 3U}, 4U},
    {2U, {2U, 3U, 0U, 0U}, 2U},
};

ecat_master_t st_master {};
std::array<linkx_t, kModuleCount> st_linkx {};
std::atomic<bool> st_running {true};

struct Options
{
    std::string ifname = "enp4s0";
    std::string can_rate = "1m-5m";
    uint32_t print_ms = 200U;
    uint32_t offline_ms = 500U;
    float duration_s = 0.0f;
    bool print_frames = false;
    bool print_raw = false;
};

struct TfminiFrame
{
    uint16_t distance_cm = 0;
    uint16_t strength = 0;
    uint16_t temperature_raw = 0;
    float temperature_c = 0.0f;
    bool valid = false;
};

struct CanTiming
{
    const char *name;
    bool canfd;
    uint8_t n_pre;
    uint8_t n_seg1;
    uint8_t n_seg2;
    uint8_t n_sjw;
    uint8_t d_pre;
    uint8_t d_seg1;
    uint8_t d_seg2;
    uint8_t d_sjw;
};

constexpr CanTiming kCanTimings[] = {
    // 80 MHz CAN clock. Bitrate = 80M / prescaler / (1 + seg1 + seg2).
    {"1m-5m", true, 2, 31, 8, 8, 1, 12, 3, 3},
};

struct RxSnapshot
{
    bool valid = false;
    uint32_t can_id = 0;
    uint16_t params_raw = 0;
    uint64_t timestamp = 0;
    uint8_t dlen = 0;
    uint8_t data[LINKX_CAN_MAX_DATA_BYTES] = {};
};

struct SensorState
{
    uint32_t slave_id = 0;
    uint8_t channel = 0;
    uint32_t id = 0;
    uint8_t rx_index = 0;
    uint8_t rx_buffer[9] = {};
    TfminiFrame latest {};
    uint32_t frame_count = 0;
    uint32_t prev_print_frame_count = 0;
    float frame_hz = 0.0f;
    uint32_t checksum_errors = 0;
    uint64_t last_can_timestamp = 0;
    std::chrono::steady_clock::time_point last_frame_time {};
    std::chrono::steady_clock::time_point prev_print_time {};

    bool push_byte(uint8_t byte, uint64_t can_timestamp, TfminiFrame &out)
    {
        if (rx_index == 0U)
        {
            if (byte == 0x59U)
            {
                rx_buffer[0] = byte;
                rx_index = 1U;
            }
            return false;
        }

        if (rx_index == 1U && byte != 0x59U)
        {
            rx_index = 0U;
            if (byte == 0x59U)
            {
                rx_buffer[0] = byte;
                rx_index = 1U;
            }
            return false;
        }

        rx_buffer[rx_index++] = byte;
        if (rx_index < 9U)
            return false;

        rx_index = 0U;

        uint16_t checksum = 0;
        for (int i = 0; i < 8; ++i)
            checksum = static_cast<uint16_t>(checksum + rx_buffer[i]);

        if (static_cast<uint8_t>(checksum & 0xFFU) != rx_buffer[8])
        {
            ++checksum_errors;
            return false;
        }

        out.distance_cm = static_cast<uint16_t>((rx_buffer[3] << 8) | rx_buffer[2]);
        out.strength = static_cast<uint16_t>((rx_buffer[5] << 8) | rx_buffer[4]);
        out.temperature_raw = static_cast<uint16_t>((rx_buffer[7] << 8) | rx_buffer[6]);
        out.temperature_c = static_cast<float>(out.temperature_raw) / 8.0f - 256.0f;
        out.valid = (out.strength >= 100U && out.strength != kInvalidStrength);

        latest = out;
        ++frame_count;
        last_can_timestamp = can_timestamp;
        last_frame_time = std::chrono::steady_clock::now();
        return true;
    }
};

constexpr size_t kSensorCount =
    ((kLastTargetId - kFirstTargetId) + 1U) * (kWatchedModules[0].channel_count + kWatchedModules[1].channel_count);

std::array<SensorState, kSensorCount> st_sensors {};
std::array<std::array<RxSnapshot, LINKX_CAN_CHANNEL_NUM>, kModuleCount> st_last_rx {};

void on_signal(int)
{
    st_running.store(false);
}

bool has_arg(int argc, char **argv, const std::string &key)
{
    const std::string flag = "--" + key;
    for (int i = 1; i < argc; ++i)
    {
        if (argv[i] == flag)
            return true;
    }
    return false;
}

std::string cli_value(int argc, char **argv, const std::string &key, const std::string &fallback)
{
    const std::string flag = "--" + key;
    const std::string prefix = flag + "=";
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == flag && (i + 1) < argc)
            return argv[i + 1];
        if (arg.compare(0, prefix.size(), prefix) == 0)
            return arg.substr(prefix.size());
    }
    return fallback;
}

uint32_t parse_u32(const std::string &value, uint32_t fallback)
{
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value.c_str(), &end, 10);
    if (end == value.c_str())
        return fallback;
    return static_cast<uint32_t>(parsed);
}

float parse_float(const std::string &value, float fallback)
{
    char *end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    return (end == value.c_str()) ? fallback : parsed;
}

const CanTiming *find_can_timing(const std::string &name)
{
    for (const auto &timing : kCanTimings)
    {
        if (name == timing.name)
            return &timing;
    }
    return nullptr;
}

uint16_t pack_params(const can_pdo_param_t &params)
{
    uint16_t raw = 0;
    raw |= static_cast<uint16_t>(params.ext & 0x01u);
    raw |= static_cast<uint16_t>((params.rtr & 0x01u) << 1);
    raw |= static_cast<uint16_t>((params.canfd & 0x01u) << 2);
    raw |= static_cast<uint16_t>((params.brs & 0x01u) << 3);
    raw |= static_cast<uint16_t>(params.dlen << 8);
    return raw;
}

bool snapshot_equals_pdo(const RxSnapshot &snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    if (!snap.valid)
        return false;
    if (snap.can_id != pdo->can_id)
        return false;
    if (snap.params_raw != pack_params(pdo->params))
        return false;
    if (snap.timestamp != pdo->timestamp)
        return false;
    if (snap.dlen != dlen)
        return false;
    if (dlen == 0U)
        return true;
    return std::memcmp(snap.data, pdo->data_u32, dlen) == 0;
}

void snapshot_update(RxSnapshot &snap, const can_tx_pdo_t *pdo, uint8_t dlen)
{
    snap.valid = true;
    snap.can_id = pdo->can_id;
    snap.params_raw = pack_params(pdo->params);
    snap.timestamp = pdo->timestamp;
    snap.dlen = dlen;
    std::memset(snap.data, 0, sizeof(snap.data));
    if (dlen > 0U)
        std::memcpy(snap.data, pdo->data_u32, dlen);
}

bool quick_recv_isolated(linkx_t *linkx, size_t module_index, uint8_t ch, can_msg_t *out_msg)
{
    if (!linkx || !out_msg || ch >= LINKX_CAN_CHANNEL_NUM || module_index >= kModuleCount)
        return false;

    can_tx_pdo_t *rx_pdo = linkx_recv_can(linkx, ch);
    if (rx_pdo == nullptr)
        return false;

    uint8_t pdo_dlen = rx_pdo->params.dlen;
    if (pdo_dlen > LINKX_CAN_MAX_DATA_BYTES)
        pdo_dlen = LINKX_CAN_MAX_DATA_BYTES;

    RxSnapshot &snap = st_last_rx[module_index][ch];
    if (snapshot_equals_pdo(snap, rx_pdo, pdo_dlen))
        return false;

    out_msg->id = rx_pdo->can_id;
    out_msg->timestamp = rx_pdo->timestamp;
    out_msg->canfd = rx_pdo->params.canfd != 0;
    out_msg->brs = rx_pdo->params.brs != 0;
    out_msg->ext = rx_pdo->params.ext != 0;
    out_msg->rtr = rx_pdo->params.rtr != 0;

    uint8_t out_dlen = pdo_dlen;
    if (out_dlen > sizeof(out_msg->data))
        out_dlen = sizeof(out_msg->data);
    out_msg->dlen = out_dlen;

    std::memset(out_msg->data, 0, sizeof(out_msg->data));
    if (out_dlen > 0U)
        std::memcpy(out_msg->data, rx_pdo->data_u32, out_dlen);

    snapshot_update(snap, rx_pdo, pdo_dlen);
    linkx->can_stats[ch].rx_frames++;
    linkx->can_stats[ch].rx_bytes += pdo_dlen;
    return true;
}

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [--ifname enp86s0] [--duration 0]\n"
        << "        [--can-rate 1m-5m] [--print-ms 200] [--offline-ms 500]\n"
        << "        [--print-frames] [--print-raw]\n\n"
        << "Behavior:\n"
        << "  Configures two LinkX EtherCAT slaves and monitors:\n"
        << "    slave 1: physical module 2, CAN0/CAN1/CAN2/CAN3 id 0x01..0x02\n"
        << "    slave 2: physical module 1, CAN2/CAN3 id 0x01..0x02\n"
        << "  Parses TFmini-S default 9-byte frames from transparent CAN payload byte streams.\n"
        << "  Distance is reported in cm and m; temperature is Temp / 8 - 256 degC.\n\n"
        << "CAN rates:\n"
        << "  1m-5m\n"
        << "  All serial-to-CAN modules on the same bus must use the same CAN-FD rate.\n";
}

void init_sensor_table()
{
    size_t index = 0;
    for (const auto &module : kWatchedModules)
    {
        for (size_t i = 0; i < module.channel_count; ++i)
        {
            const uint8_t ch = module.channels[i];
            for (uint32_t id = kFirstTargetId; id <= kLastTargetId; ++id)
            {
                st_sensors[index].slave_id = module.slave_id;
                st_sensors[index].channel = ch;
                st_sensors[index].id = id;
                ++index;
            }
        }
    }
}

SensorState *find_sensor(uint32_t slave_id, uint8_t channel, uint32_t can_id)
{
    const uint32_t std_id = can_id & 0x7FFU;
    for (auto &sensor : st_sensors)
    {
        if (sensor.slave_id == slave_id && sensor.channel == channel && sensor.id == std_id)
            return &sensor;
    }
    return nullptr;
}

bool configure_linkx_can(linkx_t *linkx, const ModuleWatch &module, const Options &opt)
{
    const CanTiming *timing = find_can_timing(opt.can_rate);
    if (timing == nullptr)
    {
        std::cerr << "[TFMINI-S] unsupported --can-rate '" << opt.can_rate << "'\n";
        return false;
    }

    for (size_t i = 0; i < module.channel_count; ++i)
    {
        const uint8_t ch = module.channels[i];
        linkx_switch_can_channel(linkx, ch, true);
    }

    for (size_t i = 0; i < module.channel_count; ++i)
    {
        const uint8_t ch = module.channels[i];
        if (!linkx_set_can_baudrate(linkx,
                                    ch,
                                    timing->canfd ? 1U : 0U,
                                    timing->n_pre,
                                    timing->n_seg1,
                                    timing->n_seg2,
                                    timing->n_sjw,
                                    timing->d_pre,
                                    timing->d_seg1,
                                    timing->d_seg2,
                                    timing->d_sjw))
        {
            std::cerr << "[TFMINI-S] slave " << module.slave_id
                      << " CAN" << static_cast<int>(ch)
                      << " " << (timing->canfd ? "FDCAN" : "classic CAN")
                      << " " << timing->name << " config failed\n";
            return false;
        }
    }

    for (size_t i = 0; i < module.channel_count; ++i)
    {
        const uint8_t ch = module.channels[i];
        linkx_switch_can_channel(linkx, ch, true);
    }

    return true;
}

bool init_ethercat_linkx(const Options &opt)
{
    if (!ecat_master_init(&st_master, opt.ifname.c_str()))
    {
        std::cerr << "[TFMINI-S] ecat_master_init failed for " << opt.ifname << "\n";
        return false;
    }

    if (st_master.slave_count < static_cast<int>(kModuleCount))
    {
        std::cerr << "[TFMINI-S] need at least " << kModuleCount
                  << " EtherCAT slaves, found " << st_master.slave_count << "\n";
        return false;
    }

    for (size_t i = 0; i < kModuleCount; ++i)
    {
        linkx_init(&st_linkx[i], kWatchedModules[i].slave_id, &st_master.ctx);
        if (!configure_linkx_can(&st_linkx[i], kWatchedModules[i], opt))
            return false;
    }

    if (!ecat_master_bring_online(&st_master))
    {
        std::cerr << "[TFMINI-S] ecat_master_bring_online failed\n";
        return false;
    }

    return true;
}

void print_raw_msg(uint32_t slave_id, uint8_t ch, const can_msg_t &msg)
{
    std::cout << "[TFMINI-S][RAW] slave=" << slave_id
              << " CAN" << static_cast<int>(ch)
              << " id=0x" << std::hex << std::setw(3) << std::setfill('0') << (msg.id & 0x7FFU)
              << std::dec << std::setfill(' ')
              << " dlc=" << static_cast<int>(msg.dlen)
              << " fd=" << (msg.canfd ? 1 : 0)
              << " brs=" << (msg.brs ? 1 : 0)
              << " data=";
    for (uint8_t i = 0; i < msg.dlen; ++i)
    {
        if (i != 0)
            std::cout << ' ';
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(msg.data[i]);
    }
    std::cout << std::dec << std::setfill(' ') << "\n";
}

void print_frame(const SensorState &sensor, const TfminiFrame &frame)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[TFMINI-S][FRAME] slave=" << sensor.slave_id
              << " CAN" << static_cast<int>(sensor.channel)
              << " id=0x" << std::hex << std::setw(3) << std::setfill('0') << sensor.id
              << std::dec << std::setfill(' ')
              << " dist=" << frame.distance_cm << "cm"
              << " range=" << (static_cast<float>(frame.distance_cm) * 0.01f) << "m"
              << " strength=" << frame.strength
              << " temp=" << frame.temperature_c << "C"
              << " valid=" << (frame.valid ? 1 : 0)
              << " frames=" << sensor.frame_count
              << " checksum_err=" << sensor.checksum_errors
              << "\n";
}

void pump_can_receive(const Options &opt)
{
    ecat_master_sync(&st_master);

    for (size_t module_index = 0; module_index < kModuleCount; ++module_index)
        linkx_recv_pdos(&st_linkx[module_index]);

    can_msg_t msg {};
    for (size_t module_index = 0; module_index < kModuleCount; ++module_index)
    {
        linkx_t *linkx = &st_linkx[module_index];
        const ModuleWatch &module = kWatchedModules[module_index];
        for (size_t i = 0; i < module.channel_count; ++i)
        {
            const uint8_t ch = module.channels[i];
            while (quick_recv_isolated(linkx, module_index, ch, &msg))
            {
                SensorState *sensor = find_sensor(module.slave_id, ch, msg.id);
                if (sensor == nullptr)
                    continue;

                if (opt.print_raw)
                    print_raw_msg(module.slave_id, ch, msg);

                const uint8_t n = (msg.dlen > LINKX_CAN_MAX_DATA_BYTES) ? LINKX_CAN_MAX_DATA_BYTES : msg.dlen;
                for (uint8_t j = 0; j < n; ++j)
                {
                    TfminiFrame frame {};
                    if (sensor->push_byte(msg.data[j], msg.timestamp, frame) && opt.print_frames)
                        print_frame(*sensor, frame);
                }
            }
        }
    }
}

bool sensor_online(const SensorState &sensor, uint32_t offline_ms)
{
    if (sensor.frame_count == 0U)
        return false;

    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - sensor.last_frame_time).count();
    return age_ms <= static_cast<long long>(offline_ms);
}

void update_sensor_rate(SensorState &sensor, std::chrono::steady_clock::time_point now)
{
    if (sensor.prev_print_time.time_since_epoch().count() == 0)
    {
        sensor.prev_print_time = now;
        sensor.prev_print_frame_count = sensor.frame_count;
        sensor.frame_hz = 0.0f;
        return;
    }

    const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(now - sensor.prev_print_time).count();
    if (dt_us > 0)
    {
        const uint32_t df = sensor.frame_count - sensor.prev_print_frame_count;
        sensor.frame_hz = static_cast<float>(df) * 1000000.0f / static_cast<float>(dt_us);
    }

    sensor.prev_print_time = now;
    sensor.prev_print_frame_count = sensor.frame_count;
}

void print_table_separator()
{
    std::cout << "+-------+------+-------+------+--------+----------+----------+----------+--------+----------+------------+------------+\n";
}

void print_table_header(double t_s)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[TFMINI-S] t=" << t_s << "s\n";
    print_table_separator();
    std::cout << "| Slave | CAN  | ID    | Stat | Hz     | Dist(cm) | Range(m) | Strength | TempC  | Frames   | CsumErr    | Timestamp  |\n";
    print_table_separator();
}

void print_sensor_row(const SensorState &sensor, const char *status)
{
    std::cout << "| "
              << std::right << std::setw(5) << sensor.slave_id
              << " | "
              << std::left << std::setw(4) << ("CAN" + std::to_string(sensor.channel))
              << " | "
              << std::right << "0x" << std::hex << std::setw(3) << std::setfill('0') << sensor.id
              << std::dec << std::setfill(' ')
              << " | "
              << std::left << std::setw(4) << status
              << " | "
              << std::right << std::fixed << std::setprecision(1) << std::setw(6) << sensor.frame_hz
              << " | "
              << std::setw(8) << sensor.latest.distance_cm
              << " | "
              << std::setprecision(3) << std::setw(8) << (static_cast<float>(sensor.latest.distance_cm) * 0.01f)
              << " | "
              << std::setprecision(0) << std::setw(8) << sensor.latest.strength
              << " | "
              << std::setprecision(1) << std::setw(6) << sensor.latest.temperature_c
              << " | "
              << std::setprecision(0) << std::setw(8) << sensor.frame_count
              << " | "
              << std::setw(10) << sensor.checksum_errors
              << " | "
              << std::setw(10) << sensor.last_can_timestamp
              << " |\n";
}

void print_status(double t_s, const Options &opt)
{
    const auto now = std::chrono::steady_clock::now();

    print_table_header(t_s);

    for (auto &sensor : st_sensors)
    {
        update_sensor_rate(sensor, now);
        const bool online = sensor_online(sensor, opt.offline_ms);
        const char *status = online ? (sensor.latest.valid ? "OK " : "BAD") : "OFF";
        print_sensor_row(sensor, status);
    }

    print_table_separator();
}

}  // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    if (has_arg(argc, argv, "help") || has_arg(argc, argv, "h"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt;
    opt.ifname = cli_value(argc,
                           argv,
                           "ifname",
                           std::getenv("IFNAME") ? std::getenv("IFNAME") : "enp86s0");
    opt.can_rate = cli_value(argc, argv, "can-rate", "1m-5m");
    opt.print_ms = std::max<uint32_t>(20U, parse_u32(cli_value(argc, argv, "print-ms", "200"), 200U));
    opt.offline_ms = std::max<uint32_t>(opt.print_ms, parse_u32(cli_value(argc, argv, "offline-ms", "500"), 500U));
    opt.duration_s = std::max(0.0f, parse_float(cli_value(argc, argv, "duration", "0"), 0.0f));
    opt.print_frames = has_arg(argc, argv, "print-frames");
    opt.print_raw = has_arg(argc, argv, "print-raw");

    if (find_can_timing(opt.can_rate) == nullptr)
    {
        std::cerr << "[TFMINI-S] unsupported --can-rate '" << opt.can_rate
                  << "'. Use --help to list supported rates.\n";
        return 1;
    }

    init_sensor_table();

    std::cout << "===============================================\n"
              << "  R2 TFmini-S CAN Read Test\n"
              << "  IFNAME      : " << opt.ifname << "\n"
              << "  CAN MODE    : FDCAN (" << opt.can_rate << ") transparent payload\n"
              << "  WATCH       : slave1(physical module2) CAN0/CAN1/CAN2/CAN3 id 0x01..0x02\n"
              << "                slave2(physical module1) CAN2/CAN3 id 0x01..0x02\n"
              << "  PRINT_MS    : " << opt.print_ms << "\n"
              << "  OFFLINE_MS  : " << opt.offline_ms << "\n"
              << "===============================================\n";

    if (!init_ethercat_linkx(opt))
        return 2;

    const uint64_t print_period_ticks = std::max<uint64_t>(1U, opt.print_ms / kControlPeriodMs);
    const uint64_t duration_ticks =
        (opt.duration_s > 0.0f) ? static_cast<uint64_t>(opt.duration_s * 1000.0f) : 0U;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint64_t tick = 0;
    while (st_running.load() && st_master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(kControlPeriodMs);

        pump_can_receive(opt);

        if ((tick % print_period_ticks) == 0U)
            print_status(static_cast<double>(tick) * 0.001, opt);

        if (duration_ticks > 0U && tick >= duration_ticks)
            break;

        ++tick;
        std::this_thread::sleep_until(next_wakeup);
    }

    std::cout << "[TFMINI-S] done.\n";
    return 0;
}
