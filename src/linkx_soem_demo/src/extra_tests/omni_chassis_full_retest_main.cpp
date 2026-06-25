// Full omni chassis retest orchestrator.
//
// This program intentionally reuses the existing extra_tests executables rather
// than reimplementing motor or ROS control logic. It builds a timestamped test
// plan from omni_chassis_full_retest_SPEC.md, runs selected stages when
// --execute is provided, and records the commands, logs, and statuses under
// var_data/omni/.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace
{
struct Options
{
    std::string bin_dir = "install/linkx_soem_demo/lib/linkx_soem_demo";
    std::string output_root = "var_data/omni";
    std::string ifname = "enp4s0";
    std::string cmd_topic = "/chassis/cmd_vel";
    std::string imu_topic = "/IMU_data";
    std::string odom_topic = "/chassis/odom_twist";
    std::string bridge_node = "/r2_vehicle_bridge";
    std::string phases = "0,1,2,3";
    std::string profile = "smoke";
    std::string speed_list = "1.0";
    double distance_m = 5.0;
    int repeats = 1;
    int accel_repeats = 1;
    int max_rounds = 1;
    bool execute = false;
    bool yes = false;
    bool dry_run = false;
    bool apply_params = false;
    bool allow_lifted = false;
    bool sudo_direct = false;
};

struct Step
{
    std::string stage;
    std::string name;
    std::string command;
    std::string log_path;
    std::string skip_reason;
    int exit_code = -1;
};

const char *cli_get(int argc, char **argv, const char *key, const char *fallback)
{
    const std::string k1 = std::string("--") + key;
    const std::string k2 = k1 + "=";
    for (int i = 1; i < argc; ++i)
    {
        if (k1 == argv[i] && (i + 1) < argc)
            return argv[i + 1];
        const std::string arg = argv[i];
        if (arg.compare(0, k2.size(), k2) == 0)
            return argv[i] + k2.size();
    }
    return fallback;
}

bool cli_has(int argc, char **argv, const char *key)
{
    const std::string k = std::string("--") + key;
    for (int i = 1; i < argc; ++i)
        if (k == argv[i])
            return true;
    return false;
}

std::string timestamp_string()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm_now {};
    localtime_r(&now, &tm_now);
    char buf[32] {};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_now);
    return std::string(buf);
}

bool path_is_dir(const std::string &path)
{
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool ensure_dir(const std::string &path)
{
    if (path.empty())
        return false;
    if (path_is_dir(path))
        return true;
    return mkdir(path.c_str(), 0755) == 0 || path_is_dir(path);
}

bool ensure_dir_recursive(const std::string &path)
{
    if (path.empty())
        return false;
    if (path_is_dir(path))
        return true;

    std::string current;
    size_t start = 0;
    if (path[0] == '/')
    {
        current = "/";
        start = 1;
    }

    while (start <= path.size())
    {
        const size_t slash = path.find('/', start);
        const std::string part = path.substr(start, slash == std::string::npos ?
                                             std::string::npos : slash - start);
        if (!part.empty())
        {
            if (!current.empty() && current.back() != '/')
                current += "/";
            current += part;
            if (!ensure_dir(current))
                return false;
        }
        if (slash == std::string::npos)
            break;
        start = slash + 1;
    }
    return path_is_dir(path);
}

std::string shell_quote(const std::string &value)
{
    std::string quoted = "'";
    for (char c : value)
    {
        if (c == '\'')
            quoted += "'\\''";
        else
            quoted += c;
    }
    quoted += "'";
    return quoted;
}

std::vector<std::string> split_csv(const std::string &text)
{
    std::vector<std::string> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
            values.push_back(item);
    }
    return values;
}

bool phase_enabled(const Options &opt, const std::string &phase)
{
    const auto phases = split_csv(opt.phases);
    return std::find(phases.begin(), phases.end(), phase) != phases.end() ||
           std::find(phases.begin(), phases.end(), "all") != phases.end();
}

std::string exe(const Options &opt, const std::string &name)
{
    return opt.bin_dir + "/" + name;
}

std::string common_ros_args(const Options &opt)
{
    std::ostringstream ss;
    ss << " --cmd-topic " << shell_quote(opt.cmd_topic)
       << " --imu-topic " << shell_quote(opt.imu_topic)
       << " --odom-topic " << shell_quote(opt.odom_topic)
       << " --bridge-node " << shell_quote(opt.bridge_node);
    if (opt.yes)
        ss << " --yes";
    if (opt.dry_run)
        ss << " --dry-run";
    if (!opt.apply_params)
        ss << " --no-param-set";
    return ss.str();
}

void add_step(std::vector<Step> &steps,
              const std::string &run_dir,
              const std::string &stage,
              const std::string &name,
              const std::string &command,
              const std::string &skip_reason = "")
{
    Step step;
    step.stage = stage;
    step.name = name;
    step.command = command;
    step.skip_reason = skip_reason;
    const std::string safe_name = stage + "_" + name;
    step.log_path = run_dir + "/" + safe_name + ".log";
    steps.push_back(step);
}

void add_accel_response(std::vector<Step> &steps,
                        const Options &opt,
                        const std::string &run_dir,
                        const std::string &stage,
                        const std::string &name,
                        bool profiled)
{
    if (opt.dry_run)
    {
        add_step(steps, run_dir, stage, name, "", "dry-run skips direct EtherCAT accel_response");
        return;
    }
    if (!opt.allow_lifted)
    {
        add_step(steps, run_dir, stage, name, "",
                 "skipped: requires wheels lifted; rerun with --allow-lifted");
        return;
    }

    for (int i = 0; i < std::max(1, opt.accel_repeats); ++i)
    {
        std::ostringstream cmd;
        if (opt.sudo_direct)
            cmd << "sudo -E ";
        cmd << shell_quote(exe(opt, "omni_accel_response"))
            << " --ifname " << shell_quote(opt.ifname)
            << " --speeds 10,30,60"
            << " --duration 2.5 --settle 1.0 --reach-frac 0.95"
            << " --profile " << (profiled ? "1" : "0")
            << " --csv " << shell_quote(run_dir + "/" + stage + "_" + name + "_r" + std::to_string(i + 1) + ".csv")
            << " --summary " << shell_quote(run_dir + "/" + stage + "_" + name + "_r" + std::to_string(i + 1) + ".txt");
        add_step(steps, run_dir, stage, name + "_r" + std::to_string(i + 1), cmd.str());
    }
}

void add_straight_line(std::vector<Step> &steps,
                       const Options &opt,
                       const std::string &run_dir,
                       const std::string &stage,
                       const std::string &name,
                       const std::string &speed,
                       const std::string &extra_args)
{
    std::ostringstream cmd;
    cmd << "ROS_LOG_DIR=" << shell_quote(run_dir + "/ros_log") << " "
        << shell_quote(exe(opt, "omni_straight_line_autotune"))
        << " --output-dir " << shell_quote(run_dir)
        << " --distance " << opt.distance_m
        << " --speed " << shell_quote(speed)
        << " --directions fb"
        << " --repeats " << opt.repeats
        << common_ros_args(opt)
        << extra_args;
    add_step(steps, run_dir, stage, name + "_v" + speed, cmd.str());
}

void add_deadzone_tune(std::vector<Step> &steps,
                       const Options &opt,
                       const std::string &run_dir,
                       const std::string &stage)
{
    std::ostringstream cmd;
    cmd << "ROS_LOG_DIR=" << shell_quote(run_dir + "/ros_log") << " "
        << shell_quote(exe(opt, "chassis_zero_deadzone_stiction_tune"))
        << " --output-dir " << shell_quote(run_dir)
        << " --mode online-tune"
        << " --directions fb"
        << " --repeats " << opt.repeats
        << " --max-rounds " << opt.max_rounds
        << " --cmd-topic " << shell_quote(opt.cmd_topic)
        << " --odom-topic " << shell_quote(opt.odom_topic)
        << " --bridge-node " << shell_quote(opt.bridge_node);
    if (opt.yes)
        cmd << " --yes";
    if (opt.dry_run)
        cmd << " --dry-run";
    if (!opt.apply_params)
        cmd << " --no-param-set";
    add_step(steps, run_dir, stage, "deadzone_stiction_online", cmd.str());
}

std::vector<Step> build_plan(const Options &opt, const std::string &run_dir)
{
    std::vector<Step> steps;
    const std::vector<std::string> speeds = split_csv(opt.speed_list);

    if (phase_enabled(opt, "0"))
    {
        add_accel_response(steps, opt, run_dir, "phase0", "gear_feedback_gate", false);
        if (!speeds.empty())
        {
            std::string extra = " --search grid --max-rounds 1 --imu-reset none";
            if (opt.apply_params)
                extra += " --kp-list 0 --ki-list 0 --kd-list 0 --out-scale-list 0 --dead-zone-list 0";
            add_straight_line(steps, opt, run_dir, "phase0", "odom_self_check", speeds.front(), extra);
        }
    }

    if (phase_enabled(opt, "1"))
    {
        for (const auto &speed : speeds)
        {
            std::string extra = " --search grid --max-rounds 1 --imu-reset none";
            if (opt.apply_params)
                extra += " --kp-list 0 --ki-list 0 --kd-list 0 --out-scale-list 0 --dead-zone-list 0";
            add_straight_line(steps, opt, run_dir, "phase1", "open_loop_fb", speed, extra);
        }
    }

    if (phase_enabled(opt, "2"))
    {
        for (const auto &speed : speeds)
        {
            std::ostringstream extra;
            extra << " --search coarse2fine --max-rounds " << opt.max_rounds
                  << " --imu-reset each";
            add_straight_line(steps, opt, run_dir, "phase2", "imu_closed_loop_fb", speed, extra.str());
        }
    }

    if (phase_enabled(opt, "3"))
    {
        add_accel_response(steps, opt, run_dir, "phase3", "profiled_wheel_accel", true);
        add_deadzone_tune(steps, opt, run_dir, "phase3");
        for (const auto &speed : speeds)
        {
            std::ostringstream extra;
            extra << " --search grid --max-rounds 1 --imu-reset none"
                  << " --linear-accel 8.0 --linear-decel 12.0 --accel-settle 0.2";
            add_straight_line(steps, opt, run_dir, "phase3", "floor_accel_stop", speed, extra.str());
        }
    }

    return steps;
}

int decode_status(int status)
{
    if (status == -1)
        return 127;
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    if (WIFSIGNALED(status))
        return 128 + WTERMSIG(status);
    return status;
}

void write_manifest(const std::string &path, const std::vector<Step> &steps)
{
    std::ofstream out(path);
    out << "stage,name,exit_code,log_path,skip_reason,command\n";
    for (const auto &s : steps)
    {
        out << s.stage << ','
            << s.name << ','
            << s.exit_code << ','
            << s.log_path << ','
            << s.skip_reason << ','
            << '"' << s.command << '"' << '\n';
    }
}

void write_report(const std::string &path,
                  const Options &opt,
                  const std::vector<Step> &steps,
                  const std::string &run_dir)
{
    std::ofstream out(path);
    out << "# omni_chassis_full_retest\n\n";
    out << "- run_dir: " << run_dir << "\n";
    out << "- profile: " << opt.profile << "\n";
    out << "- phases: " << opt.phases << "\n";
    out << "- speed_list: " << opt.speed_list << "\n";
    out << "- distance_m: " << opt.distance_m << "\n";
    out << "- repeats: " << opt.repeats << "\n";
    out << "- accel_repeats: " << opt.accel_repeats << "\n";
    out << "- execute: " << opt.execute << "\n";
    out << "- apply_params: " << opt.apply_params << "\n";
    out << "- allow_lifted: " << opt.allow_lifted << "\n";
    out << "- dry_run: " << opt.dry_run << "\n\n";
    out << "## Steps\n\n";
    out << "| stage | name | exit | log | notes |\n";
    out << "|---|---|---:|---|---|\n";
    for (const auto &s : steps)
    {
        out << "| " << s.stage
            << " | " << s.name
            << " | " << s.exit_code
            << " | " << s.log_path
            << " | " << s.skip_reason
            << " |\n";
    }
}

Options parse_options(int argc, char **argv)
{
    Options opt;
    if (const char *env_ifname = std::getenv("IFNAME"))
        opt.ifname = env_ifname;

    opt.bin_dir = cli_get(argc, argv, "bin-dir", opt.bin_dir.c_str());
    opt.output_root = cli_get(argc, argv, "output-root", opt.output_root.c_str());
    opt.ifname = cli_get(argc, argv, "ifname", opt.ifname.c_str());
    opt.cmd_topic = cli_get(argc, argv, "cmd-topic", opt.cmd_topic.c_str());
    opt.imu_topic = cli_get(argc, argv, "imu-topic", opt.imu_topic.c_str());
    opt.odom_topic = cli_get(argc, argv, "odom-topic", opt.odom_topic.c_str());
    opt.bridge_node = cli_get(argc, argv, "bridge-node", opt.bridge_node.c_str());
    opt.phases = cli_get(argc, argv, "phases", opt.phases.c_str());
    opt.profile = cli_get(argc, argv, "profile", opt.profile.c_str());

    if (opt.profile == "spec")
    {
        opt.speed_list = "1.0,1.5,2.0";
        opt.repeats = 20;
        opt.accel_repeats = 10;
        opt.max_rounds = 40;
    }

    opt.speed_list = cli_get(argc, argv, "speeds", opt.speed_list.c_str());
    opt.distance_m = std::stod(cli_get(argc, argv, "distance", std::to_string(opt.distance_m).c_str()));
    opt.repeats = std::max(1, std::atoi(cli_get(argc, argv, "repeats", std::to_string(opt.repeats).c_str())));
    opt.accel_repeats = std::max(0, std::atoi(cli_get(argc, argv, "accel-repeats", std::to_string(opt.accel_repeats).c_str())));
    opt.max_rounds = std::max(1, std::atoi(cli_get(argc, argv, "max-rounds", std::to_string(opt.max_rounds).c_str())));
    opt.execute = cli_has(argc, argv, "execute");
    opt.yes = cli_has(argc, argv, "yes");
    opt.dry_run = cli_has(argc, argv, "dry-run");
    opt.apply_params = cli_has(argc, argv, "apply-params");
    opt.allow_lifted = cli_has(argc, argv, "allow-lifted");
    opt.sudo_direct = cli_has(argc, argv, "sudo-direct");
    return opt;
}

void print_usage(const char *argv0)
{
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --execute                  run the plan; otherwise only writes plan/report\n"
        << "  --profile smoke|spec       smoke default: 1 repeat at 1.0 m/s; spec: 20 repeats at 1.0,1.5,2.0\n"
        << "  --phases 0,1,2,3|all       selected retest phases\n"
        << "  --allow-lifted             enable direct EtherCAT wheel tests that require suspended wheels\n"
        << "  --apply-params             allow child tuners to write /r2_vehicle_bridge parameters\n"
        << "  --yes                      pass safety confirmation through to child ROS tests\n"
        << "  --dry-run                  child ROS tests simulate; direct EtherCAT tests are skipped\n"
        << "  --speeds 1.0,1.5,2.0       override speed list in m/s\n"
        << "  --repeats N --max-rounds N --accel-repeats N\n"
        << "  --ifname IFACE --bin-dir DIR --output-root DIR\n";
}

} // namespace

int main(int argc, char **argv)
{
    if (cli_has(argc, argv, "help"))
    {
        print_usage(argv[0]);
        return 0;
    }

    Options opt = parse_options(argc, argv);
    const std::string stamp = timestamp_string();
    const std::string run_dir = opt.output_root + "/omni_chassis_full_retest_" + stamp;
    if (!ensure_dir_recursive(run_dir))
    {
        std::cerr << "[omni_chassis_full_retest] cannot create " << run_dir << "\n";
        return 2;
    }
    if (!ensure_dir_recursive(run_dir + "/ros_log"))
    {
        std::cerr << "[omni_chassis_full_retest] cannot create " << run_dir << "/ros_log\n";
        return 2;
    }

    std::vector<Step> steps = build_plan(opt, run_dir);
    std::cout << "[omni_chassis_full_retest] run_dir=" << run_dir << "\n"
              << "[omni_chassis_full_retest] steps=" << steps.size()
              << " execute=" << opt.execute << "\n";

    int final_rc = 0;
    if (opt.execute)
    {
        for (auto &step : steps)
        {
            if (!step.skip_reason.empty())
            {
                std::cout << "[SKIP] " << step.stage << "/" << step.name
                          << ": " << step.skip_reason << "\n";
                step.exit_code = 0;
                continue;
            }

            const std::string wrapped = step.command + " > " + shell_quote(step.log_path) + " 2>&1";
            std::cout << "[RUN] " << step.stage << "/" << step.name
                      << " log=" << step.log_path << "\n";
            const auto start = std::chrono::steady_clock::now();
            step.exit_code = decode_status(std::system(wrapped.c_str()));
            const auto end = std::chrono::steady_clock::now();
            const double elapsed_s =
                std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
            std::cout << "[DONE] " << step.stage << "/" << step.name
                      << " exit=" << step.exit_code
                      << " elapsed_s=" << std::fixed << std::setprecision(1) << elapsed_s << "\n";
            if (step.exit_code != 0 && final_rc == 0)
                final_rc = step.exit_code;
        }
    }

    const std::string manifest = run_dir + "/manifest.csv";
    const std::string report = run_dir + "/report.md";
    write_manifest(manifest, steps);
    write_report(report, opt, steps, run_dir);
    std::cout << "[omni_chassis_full_retest] manifest=" << manifest << "\n"
              << "[omni_chassis_full_retest] report=" << report << "\n";
    return final_rc;
}
