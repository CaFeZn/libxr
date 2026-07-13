#pragma once

#include "double_buffer.hpp"
#include "libxr_rw.hpp"

namespace LibXR::HPMUARTDetail
{
inline void FailAndClearPendingWrites(WritePort& port, DoubleBuffer& buffer,
                                      ErrorCode result, bool in_isr)
{
  WriteInfoBlock pending{};
  while (port.queue_info_->Pop(pending) == ErrorCode::OK)
  {
    port.Finish(in_isr, result, pending);
  }
  port.Reset();
  buffer.Reset();
}
}  // namespace LibXR::HPMUARTDetail
