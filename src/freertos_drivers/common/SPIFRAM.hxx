/** \copyright
 * Copyright (c) 2026, Rick Lull
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file SPIFRAM.hxx
 *
 * Driver for SPI-attached FRAM (Ferroelectric RAM) devices. Unlike NOR flash,
 * FRAM requires no erase cycle before writing and has no page-size
 * limitations. Writes complete within the SPI clock cycle — no busy-polling
 * is needed.
 *
 * @author Rick Lull and Claude
 * @date 4 March 2026
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SPIFRAM_HXX_
#define _FREERTOS_DRIVERS_COMMON_SPIFRAM_HXX_

#include <inttypes.h>
#include <spi/spidev.h>
#include <sys/types.h>

class OSMutex;

/// Configuration for an SPI FRAM device. Create a const instance of this
/// and pass a pointer to the SPIFRAM constructor.
///
/// Example:
/// @code
///   static const SPIFRAMConfig framCfg = {
///       .speedHz_ = 20000000,
///       .addrBytes_ = 2,
///   };
/// @endcode
struct SPIFRAMConfig
{
    /// SPI clock frequency in Hz. Most SPI FRAMs support up to 20–40 MHz.
    uint32_t speedHz_ {20000000};

    /// Number of address bytes: 2 for devices up to 512 Kbit (64 KB),
    /// 3 for devices larger than 512 Kbit.
    uint8_t addrBytes_ {3};

    /// SPI bus mode (CPOL/CPHA). Most SPI FRAMs use SPI_MODE_0.
    uint8_t spiMode_ {SPI_MODE_0};

    /// Command to read device identification bytes (RDID).
    /// Not all FRAM parts implement this command.
    uint8_t idCommand_ {0x9F};

    /// Command for sequential read (READ).
    uint8_t readCommand_ {0x03};

    /// Write Enable Latch command (WREN). Must be sent before every write.
    /// The WEL bit is automatically cleared at the end of each write.
    uint8_t writeEnableCommand_ {0x06};

    /// Command for sequential write (WRITE).
    uint8_t writeCommand_ {0x02};
};

/// Driver for SPI-attached FRAM (Ferroelectric RAM) devices.
///
/// Key differences from NOR flash:
///   - No erase-before-write required.
///   - No page-size constraint — arbitrary byte ranges may be written.
///   - No busy-polling after write — FRAM writes complete within the clock
///     cycle and the WIP (Write In Progress) status bit is never set.
///   - A Write Enable (WREN) command must precede every write; the Write
///     Enable Latch is automatically cleared after each write.
///
/// Typical setup in HwInit.cxx:
/// @code
///   static const SPIFRAMConfig framCfg = { .speedHz_ = 20000000,
///                                          .addrBytes_ = 2 };
///   OSMutex framMutex;
///   SPIFRAM spiFram(&framCfg, &framMutex);
///   // In hw_postinit:
///   spiFram.init("/dev/spi1.fram");
/// @endcode
class SPIFRAM
{
public:
    /// Constructor.
    /// @param cfg static configuration for this FRAM device.
    /// @param lock mutex held for the duration of each multi-step operation
    ///             (WREN + WRITE). Prevents interleaving with other users of
    ///             the same SPI bus. May be nullptr if the caller guarantees
    ///             single-threaded access.
    SPIFRAM(const SPIFRAMConfig *cfg, OSMutex *lock)
        : cfg_(cfg)
        , lock_(lock)
    {
    }

    /// @return the configuration.
    const SPIFRAMConfig &cfg()
    {
        return *cfg_;
    }

    /// Opens the SPI device. Call once in hw_postinit() or appl_main()
    /// before performing any read/write operations.
    /// @param dev_name path to the SPI device (e.g. "/dev/spi1.fram").
    void init(const char *dev_name);

    /// Reads bytes from the FRAM.
    /// @param addr start address within the FRAM (0 = first byte).
    /// @param buf  destination buffer; must be at least @p len bytes.
    /// @param len  number of bytes to read.
    void read(uint32_t addr, void *buf, size_t len);

    /// Writes bytes to the FRAM.
    /// No erase is required and there are no page-boundary constraints.
    /// The write is complete when this function returns.
    /// @param addr start address within the FRAM.
    /// @param buf  source data.
    /// @param len  number of bytes to write.
    void write(uint32_t addr, const void *buf, size_t len);

    /// Reads the 3-byte device identification bytes from the FRAM.
    /// Not all FRAM devices implement the RDID command.
    /// @param id_out receives the three identification bytes.
    void get_id(char id_out[3]);

private:
    /// Configuration.
    const SPIFRAMConfig *cfg_;

    /// Optional mutex protecting multi-step operations.
    OSMutex *lock_;

    /// File descriptor for the opened SPI device.
    int spiFd_ {-1};
};

#endif // _FREERTOS_DRIVERS_COMMON_SPIFRAM_HXX_
