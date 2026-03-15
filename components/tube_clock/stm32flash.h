/*
  stm32flash - Open Source ST STM32 flash program for Arduino
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#pragma once

#ifdef USE_TUBE_CLOCK_DFU

#include <cstdint>
#include <memory>
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tube_clock {

/* flags */
constexpr auto STREAM_OPT_BYTE = (1 << 0);      /* byte (not frame) oriented */
constexpr auto STREAM_OPT_GVR_ETX = (1 << 1);   /* cmd GVR returns protection status */
constexpr auto STREAM_OPT_CMD_INIT = (1 << 2);  /* use INIT cmd to autodetect speed */
constexpr auto STREAM_OPT_RETRY = (1 << 3);     /* allowed read() retry after timeout */
constexpr auto STREAM_OPT_I2C = (1 << 4);       /* i2c */
constexpr auto STREAM_OPT_STRETCH_W = (1 << 5); /* warning for no-stretching commands */

constexpr auto STREAM_SERIAL = (STREAM_OPT_BYTE | STREAM_OPT_GVR_ETX | STREAM_OPT_CMD_INIT | STREAM_OPT_RETRY);
constexpr auto STREAM_I2C = (STREAM_OPT_I2C | STREAM_OPT_STRETCH_W);

constexpr auto STM32_MAX_RX_FRAME = 256;           /* cmd read memory */
constexpr auto STM32_MAX_TX_FRAME = (1 + 256 + 1); /* cmd write memory */

constexpr auto STM32_MAX_PAGES = 0x0000ffff;
constexpr auto STM32_MASS_ERASE = 0x00100000; /* > 2 x max_pages */

using stm32_err_t = enum Stm32Err {
  STM32_ERR_OK = 0,
  STM32_ERR_UNKNOWN, /* Generic error */
  STM32_ERR_NACK,
  STM32_ERR_NO_CMD,     /* Command not available in bootloader */
  STM32_ERR_PENDING,    /* No byte available yet; caller should retry next loop() */
};

using flags_t = enum Flags {
  F_NO_ME = 1 << 0, /* Mass-Erase not supported */
  F_OBLL = 1 << 1,  /* OBL_LAUNCH required */
};

using stm32_cmd_t = struct Stm32Cmd {
  uint8_t get;
  uint8_t gvr;
  uint8_t gid;
  uint8_t rm;
  uint8_t go;
  uint8_t wm;
  uint8_t er; /* this may be extended erase */
  uint8_t wp;
  uint8_t uw;
  uint8_t rp;
  uint8_t ur;
  uint8_t crc;
};

using stm32_dev_t = struct Stm32Dev {  // NOLINT
  const uint16_t id;
  const char *name;
  const uint32_t ram_start, ram_end;
  const uint32_t fl_start, fl_end;
  const uint16_t fl_pps;  // pages per sector
  const uint32_t *fl_ps;  // page size
  const uint32_t opt_start, opt_end;
  const uint32_t mem_start, mem_end;
  const uint32_t flags;
};

using stm32_t = struct Stm32 {
  uart::UARTDevice *stream;
  uint8_t flags;
  struct VarlenCmd *cmd_get_reply;
  uint8_t bl_version;
  uint8_t version;
  uint8_t option1, option2;
  uint16_t pid;
  stm32_cmd_t *cmd;
  const stm32_dev_t *dev;
};

/*
 * Specify the length of reply for command GET
 * This is helpful for frame-oriented protocols, e.g. i2c, to avoid time
 * consuming try-fail-timeout-retry operation.
 * On byte-oriented protocols, i.e. UART, this information would be skipped
 * after read the first byte, so not needed.
 */
struct VarlenCmd {
  uint8_t version;
  uint8_t length;
};

using stm32_unique_ptr = std::unique_ptr<stm32_t, void (*)(stm32_t *)>;

stm32_unique_ptr stm32_init(uart::UARTDevice *stream, uint8_t flags, char init);
stm32_err_t stm32_read_memory(const stm32_unique_ptr &stm, uint32_t address, uint8_t *data, unsigned int len);
stm32_err_t stm32_write_memory(const stm32_unique_ptr &stm, uint32_t address, const uint8_t *data, unsigned int len);
stm32_err_t stm32_wunprot_memory(const stm32_unique_ptr &stm);
stm32_err_t stm32_wprot_memory(const stm32_unique_ptr &stm);
stm32_err_t stm32_erase_memory(const stm32_unique_ptr &stm, uint32_t spage, uint32_t pages);
stm32_err_t stm32_go(const stm32_unique_ptr &stm, uint32_t address);
stm32_err_t stm32_reset_device(const stm32_unique_ptr &stm);
stm32_err_t stm32_readprot_memory(const stm32_unique_ptr &stm);
stm32_err_t stm32_runprot_memory(const stm32_unique_ptr &stm);
stm32_err_t stm32_crc_memory(const stm32_unique_ptr &stm, uint32_t address, uint32_t length, uint32_t *crc);
stm32_err_t stm32_crc_wrapper(const stm32_unique_ptr &stm, uint32_t address, uint32_t length, uint32_t *crc);
uint32_t stm32_sw_crc(uint32_t crc, uint8_t *buf, unsigned int len);

// ---------------------------------------------------------------------------
// Non-blocking protocol helpers (for use with Stm32Flasher)
// ---------------------------------------------------------------------------

// Non-blocking ACK poll. Returns PENDING when no byte is available yet.
// BUSY (0x76) is also treated as PENDING (device still processing).
stm32_err_t stm32_poll_ack(const stm32_unique_ptr &stm);

// Send the erase command for a single flash page without waiting for the
// final erase-complete ACK. Returns OK once the command and page payload have
// been written; the caller must then poll stm32_poll_ack() with a ≤5s deadline.
stm32_err_t stm32_erase_page_send(const stm32_unique_ptr &stm, uint32_t page_num);

// Send write command + address + data payload without waiting for the final
// write-complete ACK. Returns OK once all bytes have been flushed; the caller
// must then poll stm32_poll_ack() with a ≤1s deadline.
stm32_err_t stm32_write_memory_send(const stm32_unique_ptr &stm, uint32_t address, const uint8_t *data,
                                    unsigned int len);

// ---------------------------------------------------------------------------
// Stm32Flasher — non-blocking state-machine flasher
// ---------------------------------------------------------------------------

/**
 * @brief Non-blocking STM32 UART bootloader flasher.
 *
 * Encapsulates all STM32 bootloader protocol state machine logic.
 * The caller drives it by invoking tick() once per loop() iteration and
 * supplying firmware chunks when tick() returns NEEDS_DATA.
 *
 * No ESP-IDF HTTP or UART reconfiguration dependencies — only uart::UARTDevice.
 * This makes Stm32Flasher independently reusable from any ESPHome component.
 *
 * Typical usage:
 *   flasher.begin(stream, firmware_size, settings_page);
 *   // in loop():
 *   auto r = flasher.tick();
 *   if (r == Stm32Flasher::Result::NEEDS_DATA)
 *     flasher.provide_chunk(buf, n);
 *   if (r == Stm32Flasher::Result::DONE || r == Stm32Flasher::Result::FAILED)
 *     cleanup();
 */
class Stm32Flasher {
 public:
  enum class Result : uint8_t {
    PENDING,     ///< Still in progress; call tick() again next loop()
    NEEDS_DATA,  ///< Call provide_chunk() then continue calling tick()
    DONE,        ///< Flash completed successfully
    FAILED,      ///< Unrecoverable error; call reset() to clean up
  };

  /**
   * Begin a new flash session.
   * @param stream       UART device already configured for the STM32 bootloader
   * @param total_bytes  Firmware size in bytes (determines pages to erase)
   * @param settings_page  First page index to never erase. Pages [settings_page,∞)
   *                       are preserved. Pass STM32_MAX_PAGES+1 to erase everything.
   */
  void begin(uart::UARTDevice *stream, size_t total_bytes, uint32_t settings_page = STM32_MAX_PAGES + 1);

  /**
   * Advance one step of the state machine. Call from loop().
   * @return PENDING, NEEDS_DATA, DONE, or FAILED.
   */
  Result tick();

  /**
   * Provide the next firmware chunk. Only valid immediately after tick() returns NEEDS_DATA.
   * @param data  Pointer to firmware bytes
   * @param len   Number of valid bytes (may be < 256 for the last chunk)
   */
  void provide_chunk(const uint8_t *data, unsigned int len);

  /// Write-phase progress in [0.0, 1.0]. Valid once writing begins.
  float progress() const { return this->progress_; }

  /// Reset to IDLE and free all STM32 resources. Safe to call at any time.
  void reset();

 protected:
  enum class FlashState : uint8_t {
    IDLE,
    INIT_ATTEMPT,     ///< Attempt stm32_init(); retry up to 5× with 4ms gap
    ERASE_PAGE_SEND,  ///< Send one-page erase command; if all pages done → WRITE_NEED_DATA
    ERASE_PAGE_WAIT,  ///< Poll for page-erase ACK (≤5s deadline)
    WRITE_NEED_DATA,  ///< Blocked; waiting for caller to supply next chunk
    WRITE_SEND,       ///< Chunk available; call stm32_write_memory_send()
    WRITE_WAIT,       ///< Poll for write ACK (≤1s deadline)
    FINALIZE,         ///< Send GO command → DONE
    DONE,
    FAILED,
  };

  uart::UARTDevice *stream_{nullptr};
  stm32_unique_ptr stm_{nullptr, nullptr};

  FlashState state_{FlashState::IDLE};
  uint32_t deadline_ms_{0};

  // Erase tracking
  uint32_t erase_page_{0};
  uint32_t erase_pages_total_{0};
  uint32_t settings_page_{0};

  // Write tracking
  size_t total_bytes_{0};
  uint32_t write_addr_{0};
  size_t bytes_remaining_{0};
  float progress_{0.0f};

  // Chunk buffer (filled by caller via provide_chunk())
  uint8_t chunk_buf_[256]{};
  unsigned int chunk_len_{0};

  // Init retry state
  uint8_t init_attempts_{0};
  uint32_t init_retry_ms_{0};
};

}  // namespace tube_clock
}  // namespace esphome

#endif  // USE_TUBE_CLOCK_DFU
