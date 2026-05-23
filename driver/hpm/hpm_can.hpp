/**
 * @file hpm_can.hpp
 * @brief HPM classic CAN IP 适配头文件 / Adapter header for HPM classic CAN IP.
 *
 * @details
 * 本文件按底层外设 IP 归档，只对应 classic `CAN_Type` IP。类名仍按 LibXR 抽象命名，
 * 因此这里的 `HPMCAN` 表示“classic CAN IP + LibXR::CAN”。若目标 SoC 暴露的是
 * `MCAN_Type`，同名 `HPMCAN` 实现位于 `hpm_mcan.*`。
 * This file is grouped by hardware IP and only targets the classic `CAN_Type` IP.
 * Class names still follow LibXR abstractions, so `HPMCAN` here means
 * "classic CAN IP + LibXR::CAN". When the target exposes `MCAN_Type`, the `HPMCAN`
 * implementation lives in `hpm_mcan.*`.
 */
#pragma once

#include <atomic>

#include "can.hpp"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_soc.h"
#include "lockfree_pool.hpp"

#if defined(MCAN_SOC_MAX_COUNT) && (MCAN_SOC_MAX_COUNT > 0)
#include "hpm_mcan.hpp"
#else

#if defined(CAN_SOC_MAX_COUNT) && (CAN_SOC_MAX_COUNT > 0)
#include "hpm_can_drv.h"
#define LIBXR_HPM_CAN_SUPPORTED 1
using LibXRHpmCanType = CAN_Type;
#else
#define LIBXR_HPM_CAN_SUPPORTED 0
using LibXRHpmCanType = void;
#endif

namespace LibXR
{

/**
 * @class HPMCAN
 * @brief HPM classic CAN 适配器 / HPM classic CAN adapter.
 *
 * @details
 * 该类在 classic `CAN_Type` IP 上实现 `LibXR::CAN`。当前文件只提供 `HPMCAN`；
 * 若后续要补 classic CAN IP 上的 `LibXR::FDCAN` 适配器，仍应继续放在 `hpm_can.*`，
 * 因为文件归属按 IP，而不是按是否启用 FD。
 * This class implements `LibXR::CAN` on top of the classic `CAN_Type` IP.
 * This file currently provides only `HPMCAN`. If a classic-CAN-IP based
 * `LibXR::FDCAN` adapter is added later, it should still live in `hpm_can.*`
 * because file ownership follows the IP, not whether FD mode is enabled.
 */
class HPMCAN : public CAN
{
 public:
  static constexpr uint32_t kInvalidIrq = 0xFFFFFFFFu;
  static constexpr uint32_t kDefaultTxPoolSize = 8;

#if LIBXR_HPM_CAN_SUPPORTED
  static constexpr uint8_t kMaxInstances = CAN_SOC_MAX_COUNT;
  static constexpr uint8_t kRxInterruptMask =
      CAN_EVENT_RECEIVE | CAN_EVENT_RX_BUF_OVERRUN | CAN_EVENT_RX_BUF_FULL;
  static constexpr uint8_t kTxInterruptMask =
      CAN_EVENT_TX_SECONDARY_BUF | CAN_EVENT_TX_PRIMARY_BUF;
  static constexpr uint8_t kErrorInterruptMask = CAN_EVENT_ERROR | CAN_EVENT_ABORT;
  static constexpr uint8_t kInterruptMask =
      kRxInterruptMask | kTxInterruptMask | kErrorInterruptMask;
  static constexpr uint8_t kCanErrorInterruptMask =
      CAN_ERROR_PASSIVE_INT_ENABLE | CAN_ERROR_ARBITRATION_LOST_INT_ENABLE |
      CAN_ERROR_BUS_ERROR_INT_ENABLE;
#else
  static constexpr uint8_t kMaxInstances = 1;
#endif

  HPMCAN(LibXRHpmCanType* can, clock_name_t clock, uint8_t index = 0,
         uint32_t irq = kInvalidIrq, bool auto_enable_irq = true,
         uint32_t tx_pool_size = kDefaultTxPoolSize);
  ~HPMCAN() override;

  ErrorCode SetConfig(const CAN::Configuration& cfg) override;
  uint32_t GetClockFreq() const override;
  ErrorCode AddMessage(const ClassicPack& pack) override;
  ErrorCode GetErrorState(CAN::ErrorState& state) const override;

  void ProcessRx(bool in_isr = false);
  void ProcessInterrupt(bool in_isr = true);
  static void OnInterrupt(uint8_t index);

 private:
  static ErrorCode ConvertStatus(hpm_stat_t status);
  static ErrorCode ValidateConfig(const CAN::Configuration& cfg);
  void EmitErrorFrame(CAN::ErrorID error_id, bool in_isr);
  void Shutdown();
  static bool HasLowLevelTiming(const CAN::BitTiming& timing);
  static uint16_t SamplePointToPermille(float sample_point);
  void TxService();

#if LIBXR_HPM_CAN_SUPPORTED
  static can_node_mode_t ConvertMode(const CAN::Mode& mode);
  static void ApplyLowLevelTiming(const CAN::BitTiming& src, can_bit_timing_param_t& dst);
  static void BuildTxFrame(const ClassicPack& pack, can_transmit_buf_t& frame);
  static bool BuildRxPack(const can_receive_buf_t& frame, ClassicPack& pack);
  static CAN::ErrorID ConvertProtocolError(uint8_t error_kind);
  void ProcessRxBuffer(bool in_isr);
  void ProcessError(bool in_isr);
#endif

  LibXRHpmCanType* can_;
  clock_name_t clock_;
  uint8_t index_;
  uint32_t irq_;
  bool auto_enable_irq_;
  bool configured_ = false;

  LockFreePool<ClassicPack> tx_pool_;
  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

#if LIBXR_HPM_CAN_SUPPORTED
  static HPMCAN* instance_map_[kMaxInstances];
#endif
};

}  // namespace LibXR

extern "C" void libxr_hpm_can_process_interrupt(uint8_t index);

#endif
