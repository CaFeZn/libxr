#include "hpm_watchdog.hpp"

using namespace LibXR;

namespace
{
#if LIBXR_HPM_EWDG_SUPPORTED
constexpr uint32_t kMillisecondsPerSecond = 1000u;
constexpr uint32_t kEwdgOsc32kHz = 32768u;
constexpr uint32_t kEwdgOsc24mHz = 24000000u;
constexpr uint32_t kEwdgBusClockSourceIndex = 0u;
constexpr uint32_t kEwdgExternalClockSourceIndex = 1u;

#if defined(EWDG_SOC_OVERTIME_REG_WIDTH) && (EWDG_SOC_OVERTIME_REG_WIDTH == 16)
constexpr uint64_t kEwdgSocTimeoutTickMax = 0xFFFFULL;
#else
constexpr uint64_t kEwdgSocTimeoutTickMax = 0xFFFFFFFFULL;
#endif

#if defined(EWDG_OT_RST_VAL_OT_RST_VAL_MASK) && defined(EWDG_OT_RST_VAL_OT_RST_VAL_SHIFT)
constexpr uint64_t kEwdgRegisterTimeoutTickMax =
    EWDG_OT_RST_VAL_OT_RST_VAL_MASK >> EWDG_OT_RST_VAL_OT_RST_VAL_SHIFT;
#else
constexpr uint64_t kEwdgRegisterTimeoutTickMax = kEwdgSocTimeoutTickMax;
#endif

constexpr uint64_t kEwdgTimeoutTickMax =
    kEwdgSocTimeoutTickMax < kEwdgRegisterTimeoutTickMax ? kEwdgSocTimeoutTickMax
                                                         : kEwdgRegisterTimeoutTickMax;

#if defined(EWDG_SOC_CLK_DIV_VAL_MAX)
constexpr uint32_t kEwdgSocClockDivPowerMax = EWDG_SOC_CLK_DIV_VAL_MAX;
#else
constexpr uint32_t kEwdgSocClockDivPowerMax =
    EWDG_CTRL0_DIV_VALUE_MASK >> EWDG_CTRL0_DIV_VALUE_SHIFT;
#endif

#if defined(EWDG_CTRL0_DIV_VALUE_MASK) && defined(EWDG_CTRL0_DIV_VALUE_SHIFT)
constexpr uint32_t kEwdgRegisterClockDivPowerMax =
    EWDG_CTRL0_DIV_VALUE_MASK >> EWDG_CTRL0_DIV_VALUE_SHIFT;
#else
constexpr uint32_t kEwdgRegisterClockDivPowerMax = kEwdgSocClockDivPowerMax;
#endif

constexpr uint32_t kEwdgClockDivPowerMax =
    kEwdgSocClockDivPowerMax < kEwdgRegisterClockDivPowerMax
        ? kEwdgSocClockDivPowerMax
        : kEwdgRegisterClockDivPowerMax;

uint64_t DivCeil(uint64_t value, uint64_t divisor)
{
  return divisor == 0u ? 0u : ((value + divisor - 1u) / divisor);
}

uint64_t ConvertTimeoutMsToTicks(uint32_t counter_clock_hz, uint32_t timeout_ms)
{
  if (counter_clock_hz == 0u)
  {
    return 0u;
  }

  const uint64_t timeout_clock_cycles =
      static_cast<uint64_t>(timeout_ms) * counter_clock_hz;
  return DivCeil(timeout_clock_cycles, kMillisecondsPerSecond);
}
#endif
}  // namespace

HPMWatchdog::HPMWatchdog(LibXRHpmEwdgType* ewdg, clock_name_t clock, uint32_t timeout_ms,
                         uint32_t feed_ms, clk_src_t clock_source, bool auto_start)
    : ewdg_(ewdg),
      clock_(clock),
      clock_source_(clock_source),
      current_config_{timeout_ms, feed_ms}
{
#if LIBXR_HPM_EWDG_SUPPORTED
  ASSERT(ewdg_ != nullptr);

  const ErrorCode config_ans = SetConfig(current_config_);
  ASSERT(config_ans == ErrorCode::OK);

  if (auto_start && config_ans == ErrorCode::OK)
  {
    const ErrorCode start_ans = Start();
    ASSERT(start_ans == ErrorCode::OK);
  }
#else
  (void)ewdg_;
  (void)clock_;
  (void)clock_source_;
  (void)auto_start;
#endif
}

ErrorCode HPMWatchdog::ConvertStatus(hpm_stat_t status)
{
#if LIBXR_HPM_EWDG_SUPPORTED
  switch (status)
  {
    case status_success:
      return ErrorCode::OK;
    case status_invalid_argument:
      return ErrorCode::ARG_ERR;
    case status_ewdg_tick_out_of_range:
    case status_ewdg_div_out_of_range:
      return ErrorCode::OUT_OF_RANGE;
    case status_ewdg_feature_unsupported:
      return ErrorCode::NOT_SUPPORT;
    default:
      return ErrorCode::FAILED;
  }
#else
  (void)status;
  return ErrorCode::NOT_SUPPORT;
#endif
}

ErrorCode HPMWatchdog::SetConfig(const Configuration& config)
{
  if (config.timeout_ms == 0u || config.feed_ms == 0u ||
      config.feed_ms > config.timeout_ms)
  {
    return ErrorCode::ARG_ERR;
  }

#if LIBXR_HPM_EWDG_SUPPORTED
  if (ewdg_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  ErrorCode ans = EnsureClockReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  uint32_t timeout_ticks = 0u;
  uint32_t clock_div_power = 0u;
  ans = ResolveTimeoutSetting(config.timeout_ms, counter_clock_hz_, &timeout_ticks,
                              &clock_div_power);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  current_config_ = config;
  return ApplyConfiguration(started_);
#else
  (void)config;
  return ErrorCode::NOT_SUPPORT;
#endif
}

ErrorCode HPMWatchdog::Feed()
{
#if LIBXR_HPM_EWDG_SUPPORTED
  if (ewdg_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }
  if (!started_)
  {
    return ErrorCode::INIT_ERR;
  }

  return ConvertStatus(ewdg_refresh(ewdg_));
#else
  return ErrorCode::NOT_SUPPORT;
#endif
}

ErrorCode HPMWatchdog::Start()
{
#if LIBXR_HPM_EWDG_SUPPORTED
  if (ewdg_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  ErrorCode ans = ApplyConfiguration(true);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  ans = ConvertStatus(ewdg_refresh(ewdg_));
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  auto_feed_ = true;
  started_ = true;
  return ErrorCode::OK;
#else
  return ErrorCode::NOT_SUPPORT;
#endif
}

ErrorCode HPMWatchdog::Stop()
{
#if LIBXR_HPM_EWDG_SUPPORTED
  if (ewdg_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  ewdg_disable(ewdg_);
  auto_feed_ = false;
  started_ = false;
  return ErrorCode::OK;
#else
  return ErrorCode::NOT_SUPPORT;
#endif
}

#if LIBXR_HPM_EWDG_SUPPORTED
ErrorCode HPMWatchdog::EnsureClockReady()
{
  clock_add_to_group(clock_, 0);
  return ResolveCounterClockFrequency(ResolveClockSource(), &counter_clock_hz_);
}

ErrorCode HPMWatchdog::ResolveTimeoutSetting(uint32_t timeout_ms,
                                             uint32_t counter_clock_hz,
                                             uint32_t* timeout_ticks,
                                             uint32_t* clock_div_power) const
{
  if (timeout_ticks == nullptr || clock_div_power == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  *timeout_ticks = 0u;
  *clock_div_power = 0u;

  uint64_t ticks = ConvertTimeoutMsToTicks(counter_clock_hz, timeout_ms);

  for (uint32_t div_power = 0u; div_power <= kEwdgClockDivPowerMax; ++div_power)
  {
    if (ticks <= kEwdgTimeoutTickMax)
    {
      *timeout_ticks = static_cast<uint32_t>(ticks);
      *clock_div_power = div_power;
      return ErrorCode::OK;
    }

    ticks = DivCeil(ticks, 2u);
  }

  return ErrorCode::OUT_OF_RANGE;
}

ErrorCode HPMWatchdog::ApplyConfiguration(bool enable_watchdog)
{
  ErrorCode ans = EnsureClockReady();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  ewdg_config_t config{};
  ewdg_get_default_config(ewdg_, &config);
  config.enable_watchdog = enable_watchdog;
  config.cnt_src_freq = counter_clock_hz_;

  uint32_t timeout_ticks = 0u;
  uint32_t clock_div_power = 0u;
  ans = ResolveTimeoutSetting(current_config_.timeout_ms, counter_clock_hz_,
                              &timeout_ticks, &clock_div_power);
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  config.ctrl_config.cnt_clk_sel = ResolveEwdgClockSelect();
  config.ctrl_config.use_lowlevel_timeout = true;
  config.ctrl_config.timeout_interrupt_val = 0u;
  config.ctrl_config.timeout_reset_val = timeout_ticks;
  config.ctrl_config.clock_div_by_power_of_2 = clock_div_power;
  config.ctrl_config.enable_window_mode = false;
  config.ctrl_config.enable_refresh_period = false;
  config.ctrl_config.enable_refresh_lock = false;
  config.ctrl_config.enable_config_lock = false;

  config.int_rst_config.enable_timeout_interrupt = false;
  config.int_rst_config.enable_timeout_reset = true;
  config.int_rst_config.enable_ctrl_parity_fail_reset = true;
  config.int_rst_config.enable_ctrl_unlock_fail_reset = true;
  config.int_rst_config.enable_ctrl_update_violation_reset = true;
  config.int_rst_config.enable_refresh_unlock_fail_reset = true;
  config.int_rst_config.enable_refresh_violation_reset = true;

  ans = ConvertStatus(ewdg_init(ewdg_, &config));
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  timeout_ms_ = current_config_.timeout_ms;
  auto_feed_interval_ms = current_config_.feed_ms;
  initialized_ = true;
  return ErrorCode::OK;
}

clk_src_t HPMWatchdog::ResolveClockSource() const
{
  if (clock_source_ != kAutoClockSource)
  {
    return clock_source_;
  }

  if (clock_ == clock_pwdg)
  {
    return clk_pwdg_src_osc32k;
  }

  return clk_wdg_src_osc32k;
}

ewdg_cnt_clk_sel_t HPMWatchdog::ResolveEwdgClockSelect() const
{
  const clk_src_t resolved_source = ResolveClockSource();
  const uint32_t source_group = GET_CLK_SRC_GROUP(resolved_source);
  const uint32_t source_index = GET_CLK_SRC_INDEX(resolved_source);

  if ((source_group == CLK_SRC_GROUP_EWDG || source_group == CLK_SRC_GROUP_PEWDG) &&
      source_index == kEwdgBusClockSourceIndex)
  {
    return ewdg_cnt_clk_src_bus_clk;
  }

  return ewdg_cnt_clk_src_ext_osc_clk;
}

ErrorCode HPMWatchdog::ResolveCounterClockFrequency(clk_src_t source,
                                                    uint32_t* frequency_hz) const
{
  if (frequency_hz == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  *frequency_hz = 0u;

  const uint32_t source_group = GET_CLK_SRC_GROUP(source);
  const uint32_t source_index = GET_CLK_SRC_INDEX(source);
  const bool is_pwdg_clock = (clock_ == clock_pwdg);

  if ((is_pwdg_clock && source_group != CLK_SRC_GROUP_PEWDG) ||
      (!is_pwdg_clock && source_group != CLK_SRC_GROUP_EWDG))
  {
    return ErrorCode::ARG_ERR;
  }

  if (source_group == CLK_SRC_GROUP_EWDG)
  {
    if (source_index == kEwdgExternalClockSourceIndex)
    {
      *frequency_hz = kEwdgOsc32kHz;
      return ErrorCode::OK;
    }

    if (source_index == kEwdgBusClockSourceIndex)
    {
      ewdg_switch_clock_source(ewdg_, ewdg_cnt_clk_src_bus_clk);
      *frequency_hz = clock_get_frequency(clock_);
      return *frequency_hz == 0u ? ErrorCode::INIT_ERR : ErrorCode::OK;
    }

    return ErrorCode::ARG_ERR;
  }

  if (source_group == CLK_SRC_GROUP_PEWDG)
  {
    if (source_index == kEwdgExternalClockSourceIndex)
    {
      *frequency_hz = kEwdgOsc32kHz;
      return ErrorCode::OK;
    }

    if (source_index == kEwdgBusClockSourceIndex)
    {
      *frequency_hz = kEwdgOsc24mHz;
      return ErrorCode::OK;
    }

    return ErrorCode::ARG_ERR;
  }

  return ErrorCode::ARG_ERR;
}
#endif
