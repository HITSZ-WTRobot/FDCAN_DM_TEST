#pragma once
#include "communication/host_wire.hpp"
#include "dma_buffer.hpp"
#include "usart.h"
#include <array>
#include <atomic>

namespace host
{
// One owner per UART. DMA storage must live in noncached, DMA-accessible AXI RAM.
// RX ISR copies completed ranges into an SPSC byte ring; parsing is task-only.
// TX uses one immutable DMA buffer until completion or confirmed abort.
class Uart final
{
public:
    Uart(UART_HandleTypeDef& handle, memory::DMABuffer<512>& rx,
         memory::DMABuffer<64>& tx) : handle_(handle), rx_(rx), tx_(tx) {}
    bool start();
    void service();
    bool pop(std::uint8_t& byte) noexcept;
    bool send(const Frame& frame);
    bool takeDiscontinuity() noexcept { return discontinuity_.exchange(false); }
    std::uint32_t rxDrops() const noexcept { return rx_drops_.load(); }
    std::uint32_t uartErrors() const noexcept { return uart_errors_.load(); }
    std::uint32_t txErrors() const noexcept { return tx_errors_.load(); }
    bool healthy() const noexcept {return started_ && !rx_error_.load();}
private:
    static Uart* owner_;
    static void receive(UART_HandleTypeDef*, std::uint16_t);
    static void transmitted(UART_HandleTypeDef*);
    static void error(UART_HandleTypeDef*);
    void receiveRange();
    UART_HandleTypeDef& handle_;
    memory::DMABuffer<512>& rx_;
    memory::DMABuffer<64>& tx_;
    std::array<std::uint8_t,2048> ring_{};
    std::atomic<std::uint32_t> write_{0}, read_{0};
    std::atomic<bool> tx_busy_{false}, rx_error_{false}, tx_error_{false}, discontinuity_{false};
    std::atomic<std::uint32_t> rx_drops_{0}, uart_errors_{0}, tx_errors_{0};
    std::uint16_t dma_position_{0}; // Only RX events, or task with IRQ excluded.
    bool started_{false};
};
} // namespace host
