#include "communication/host_uart.hpp"
#include "isr_lock.h"
#include <cstring>

namespace host
{
Uart* Uart::owner_ = nullptr;
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

bool Uart::start()
{
    if (owner_ || !handle_.hdmarx || !handle_.hdmatx ||
        handle_.hdmarx->Init.Mode != DMA_CIRCULAR || handle_.hdmatx->Init.Mode != DMA_NORMAL)
        return false;
    owner_=this;
    // Custom sections are NOLOAD: initialize DMA storage explicitly before first use.
    std::memset(rx_.data(),0,rx_.size());std::memset(tx_.data(),0,tx_.size());
    __DMB(); // Publish initialization before enabling DMA on noncached AXI RAM.
    if (HAL_UART_RegisterRxEventCallback(&handle_,receive)!=HAL_OK ||
        HAL_UART_RegisterCallback(&handle_,HAL_UART_TX_COMPLETE_CB_ID,transmitted)!=HAL_OK ||
        HAL_UART_RegisterCallback(&handle_,HAL_UART_ERROR_CB_ID,error)!=HAL_OK)
        return false;
    started_=HAL_UARTEx_ReceiveToIdle_DMA(&handle_,rx_.data(),rx_.size())==HAL_OK;
    return started_;
}

void Uart::receive(UART_HandleTypeDef* h,std::uint16_t)
{
    if(owner_ && h==&owner_->handle_ && !owner_->rx_error_.load())owner_->receiveRange();
}

void Uart::receiveRange()
{
    // Read live DMA position, not an older pending HT/TC/IDLE event's Size.
    // HT/TC stay enabled: continuous traffic is serviced at least every half-buffer.
    const auto remaining=__HAL_DMA_GET_COUNTER(handle_.hdmarx);
    __DMB(); // Observe DMA-written bytes after sampling its completed position.
    const auto position=static_cast<std::uint16_t>((rx_.size()-remaining)%rx_.size());
    auto write=write_.load(std::memory_order_relaxed);
    const auto read=read_.load(std::memory_order_acquire);
    const auto count=(position+rx_.size()-dma_position_)%rx_.size();
    if(count>ring_.size()-(write-read))
    {
        rx_drops_.fetch_add(count,std::memory_order_relaxed);
        discontinuity_.store(true,std::memory_order_release);
        dma_position_=position;
        return;
    }
    for(std::size_t n=0;n<count;++n)
    {
        ring_[write%ring_.size()]=rx_.data()[dma_position_];
        ++write;dma_position_=(dma_position_+1)%rx_.size();
    }
    write_.store(write,std::memory_order_release);
}

bool Uart::pop(std::uint8_t& byte) noexcept
{
    const auto read=read_.load(std::memory_order_relaxed);
    if(read==write_.load(std::memory_order_acquire))return false;
    byte=ring_[read%ring_.size()];read_.store(read+1,std::memory_order_release);return true;
}

void Uart::transmitted(UART_HandleTypeDef* h)
{
    if(owner_ && h==&owner_->handle_)owner_->tx_busy_.store(false,std::memory_order_release);
}

void Uart::error(UART_HandleTypeDef* h)
{
    if(!owner_ || h!=&owner_->handle_)return;
    auto& self=*owner_;
    self.uart_errors_.fetch_add(1,std::memory_order_relaxed);
    if(h->ErrorCode & (HAL_UART_ERROR_PE|HAL_UART_ERROR_FE|HAL_UART_ERROR_NE|HAL_UART_ERROR_ORE))
        self.rx_error_.store(true,std::memory_order_release);
    if(h->ErrorCode & HAL_UART_ERROR_DMA)
    {
        if(h->hdmarx && h->hdmarx->ErrorCode!=HAL_DMA_ERROR_NONE)self.rx_error_.store(true);
        if(h->hdmatx && h->hdmatx->ErrorCode!=HAL_DMA_ERROR_NONE)self.tx_error_.store(true);
    }
    self.discontinuity_.store(true,std::memory_order_release);
}

void Uart::service()
{
    // Abort/restart only the already configured transport, in task context.
    if(rx_error_.load())
    {
        if(HAL_UART_AbortReceive(&handle_)==HAL_OK)
        {
            {ISRGuard guard;read_.store(write_.load());dma_position_=0;}
            if(HAL_UARTEx_ReceiveToIdle_DMA(&handle_,rx_.data(),rx_.size())==HAL_OK)
            {
                rx_error_.store(false);discontinuity_.store(true);
            }
        }
    }
    if(tx_error_.load() && HAL_UART_AbortTransmit(&handle_)==HAL_OK)
    {
        tx_error_.store(false);tx_busy_.store(false);tx_errors_.fetch_add(1);
    }
    if(discontinuity_.load())
    {
        ISRGuard guard;
        read_.store(write_.load()); // Discard ambiguous pre/post-gap bytes as one boundary.
    }
}

bool Uart::send(const Frame& frame)
{
    if(tx_busy_.load(std::memory_order_acquire) || tx_error_.load())return false;
    std::memcpy(tx_.data(),frame.data(),frame.size());
    __DMB(); // Publish the full immutable frame before DMA can read it.
    tx_busy_.store(true,std::memory_order_release);
    const auto status=HAL_UART_Transmit_DMA(&handle_,tx_.data(),frame.size());
    if(status==HAL_OK)return true;
    tx_busy_.store(false,std::memory_order_release);
    if(status!=HAL_BUSY)tx_errors_.fetch_add(1,std::memory_order_relaxed);
    return false;
}
} // namespace host
