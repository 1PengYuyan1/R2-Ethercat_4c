#include "linkx_soem_demo/remote/device/dvc_logF710.h"

#include <cmath>

namespace
{
constexpr uint32_t kStartLongPressMs = 500;
constexpr uint32_t kBackLongPressMs = 100;

bool Is_Button_Pressed(const std::vector<int> &buttons, size_t idx)
{
    return (idx < buttons.size()) && (buttons[idx] != 0);
}

bool Is_DInput_Layout(const std::vector<float> &axes, const std::vector<int> &buttons)
{
    // Logitech Cordless RumblePad 2 常见: 6 axes + 12 buttons
    return (axes.size() == 6U) && (buttons.size() >= 10U);
}

bool Is_XInput_Layout(const std::vector<float> &axes, const std::vector<int> &buttons)
{
    // Logitech F710/Xbox 常见: 8 axes + >= 11 buttons
    return (axes.size() >= 8U) && (buttons.size() >= 11U);
}

bool Resolve_Dpad_From_Axes(const std::vector<float> &axes, size_t x_idx, size_t y_idx, uint16_t *key_code)
{
    if (axes.size() <= y_idx || key_code == nullptr) return false;

    const float x = axes[x_idx];
    const float y = axes[y_idx];
    if (y > 0.5f)
    {
        *key_code = LogF710_Key_Up;
        return true;
    }
    if (y < -0.5f)
    {
        *key_code = LogF710_Key_Down;
        return true;
    }
    if (x > 0.5f)
    {
        *key_code = LogF710_Key_Left;
        return true;
    }
    if (x < -0.5f)
    {
        *key_code = LogF710_Key_Right;
        return true;
    }
    return false;
}
}

void Class_LogF710::Set_Control_Params(float max_v, float max_w, float deadzone)
{
    robot_velocity_max_ = max_v;
    robot_rotation_max_ = max_w;
    deadzone_ = deadzone;
}

float Class_LogF710::Apply_Deadzone(float raw_val) const
{
    if (std::abs(raw_val) < deadzone_) return 0.0f;
    return raw_val;
}

Struct_LogF710_Command Class_LogF710::Resolve_Chassis(const std::vector<float> &axes) const
{
    Struct_LogF710_Command cmd;

    // 默认映射：axes[1] 前后, axes[0] 左右
    // 旋转轴在不同布局下固定不同索引，避免串轴：
    // DInput(RumblePad2): axes[2] (Z)；XInput/F710: axes[3] (Rx)。
    if (axes.size() > 2)
    {
        float vx_axis = axes[1];
        float vy_axis = axes[0];
        float omega_axis = axes[2];
        if (axes.size() > 3U)
        {
            const bool dinput_like = (axes.size() == 6U);
            if (dinput_like)
            {
                // RumblePad2/旧驱动下，右摇杆X可能落在 axes[2] 或 axes[3]。
                // 优先用 axes[2]，其静止且 axes[3] 活跃时回退到 axes[3]。
                omega_axis = axes[2];
                if ((std::abs(omega_axis) < deadzone_) && (std::abs(axes[3]) >= deadzone_))
                {
                    omega_axis = axes[3];
                }
            }
            else
            {
                omega_axis = axes[3];
            }
        }

        // DInput 模式下，F710 MODE 可能导致“左摇杆与十字键互换”
        // 当主平移轴近零、而 axes[4]/axes[5] 明显变化时，回退使用 4/5 作为平移轴。
        if (axes.size() == 6U)
        {
            const bool primary_xy_idle = (std::abs(axes[0]) < deadzone_) && (std::abs(axes[1]) < deadzone_);
            const bool alt_xy_active = (std::abs(axes[4]) >= deadzone_) || (std::abs(axes[5]) >= deadzone_);
            if (primary_xy_idle && alt_xy_active)
            {
                vy_axis = axes[4];
                vx_axis = axes[5];
            }
        }

        cmd.vx = Apply_Deadzone(vx_axis) * robot_velocity_max_;
        cmd.vy = Apply_Deadzone(vy_axis) * robot_velocity_max_;
        cmd.omega = Apply_Deadzone(omega_axis) * robot_rotation_max_;
    }

    return cmd;
}

uint16_t Class_LogF710::Resolve_Button_Code(const std::vector<float> &axes, const std::vector<int> &buttons) const
{
    const bool dinput_like = Is_DInput_Layout(axes, buttons);
    const bool xinput_like = Is_XInput_Layout(axes, buttons);

    // 与单片机端 Resolve_Key_Status 一致：返回单个键值码，不做位组合
    if (dinput_like)
    {
        if (Is_Button_Pressed(buttons, 8U)) return LogF710_Key_Back;
        if (Is_Button_Pressed(buttons, 9U)) return LogF710_Key_Start;
    }
    else if (xinput_like)
    {
        if (Is_Button_Pressed(buttons, 6U)) return LogF710_Key_Back;
        if (Is_Button_Pressed(buttons, 7U)) return LogF710_Key_Start;
    }
    else
    {
        // 未知布局时优先保持历史 DInput 约定，避免误触 BACK
        if (Is_Button_Pressed(buttons, 8U)) return LogF710_Key_Back;
        if (Is_Button_Pressed(buttons, 9U)) return LogF710_Key_Start;
    }

    if (Is_Button_Pressed(buttons, 2U)) return LogF710_Key_X;
    if (Is_Button_Pressed(buttons, 3U)) return LogF710_Key_Y;
    if (Is_Button_Pressed(buttons, 0U)) return LogF710_Key_A;
    if (Is_Button_Pressed(buttons, 1U)) return LogF710_Key_B;
    if (Is_Button_Pressed(buttons, 4U)) return LogF710_Key_LB;
    if (Is_Button_Pressed(buttons, 5U)) return LogF710_Key_RB;
    if (dinput_like)
    {
        if (Is_Button_Pressed(buttons, 6U)) return LogF710_Key_LT;
        if (Is_Button_Pressed(buttons, 7U)) return LogF710_Key_RT;
    }

    // 十字键按布局解算，未知布局时回退 DInput。
    uint16_t dpad_key = LogF710_Key_IDLE;
    if (xinput_like)
    {
        if (Resolve_Dpad_From_Axes(axes, 6U, 7U, &dpad_key)) return dpad_key;
    }
    else
    {
        if (Resolve_Dpad_From_Axes(axes, 4U, 5U, &dpad_key)) return dpad_key;
    }

    return LogF710_Key_IDLE;
}

void Class_LogF710::Update_Control_Enable(uint16_t key_code, uint32_t dt_ms)
{
    if (key_code == LogF710_Key_Start)
    {
        start_press_ms_ += dt_ms;
        if (start_press_ms_ >= kStartLongPressMs)
        {
            is_ros_control_enabled_ = true;
        }
    }
    else
    {
        start_press_ms_ = 0;
    }

    if (key_code == LogF710_Key_Back)
    {
        back_press_ms_ += dt_ms;
        if (back_press_ms_ >= kBackLongPressMs)
        {
            is_ros_control_enabled_ = false;
        }
    }
    else
    {
        back_press_ms_ = 0;
    }
}

bool Class_LogF710::Check_Key_Rising_Edge(uint16_t key_code, uint16_t *rising_key)
{
    const bool is_rising = (key_code != LogF710_Key_IDLE) && (key_code != last_button_code_);
    if (is_rising && rising_key)
    {
        *rising_key = key_code;
    }

    last_button_code_ = key_code;
    return is_rising;
}
