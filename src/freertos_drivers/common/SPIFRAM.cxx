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
 * \file SPIFRAM.cxx
 *
 * Driver implementation for SPI-attached FRAM devices.
 *
 * @author Rick Lull and Claude
 * @date 4 March 2026
 */

#include "freertos_drivers/common/SPIFRAM.hxx"

#include <fcntl.h>
#include <spi/spidev.h>
#include <stropts.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "os/OS.hxx"
#include "utils/logging.h"

/// RAII guard that locks an OSMutex on construction and unlocks on
/// destruction. Handles nullptr safely (no-op when no mutex is needed).
class LockIfPresent
{
public:
    LockIfPresent(OSMutex *mu)
        : mu_(mu)
    {
        if (mu_)
        {
            mu_->lock();
        }
    }

    ~LockIfPresent()
    {
        if (mu_)
        {
            mu_->unlock();
        }
    }

private:
    OSMutex *mu_;
};

/// Intentional compile-time trap. Writing LockIfPresent(mu) — without a
/// named variable — creates a temporary that is destroyed immediately,
/// releasing the lock before the critical section runs. This macro turns
/// that mistake into a compile error. Always write: LockIfPresent l(mu);
#define LockIfPresent(l) int error_omitted_lock_guard_variable[-1]

void SPIFRAM::init(const char *dev_name)
{
    spiFd_ = ::open(dev_name, O_RDWR);
    HASSERT(spiFd_ >= 0);

    uint8_t spi_bpw = 8;
    int ret;
    ret = ::ioctl(spiFd_, SPI_IOC_WR_MODE, &cfg_->spiMode_);
    HASSERT(ret == 0);
    ret = ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
    HASSERT(ret == 0);
    ret = ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &cfg_->speedHz_);
    HASSERT(ret == 0);
}

void SPIFRAM::get_id(char id_out[3])
{
    LockIfPresent l(lock_);
    struct spi_ioc_transfer xfer[2] = {0, 0};
    xfer[0].tx_buf = (uintptr_t)&cfg_->idCommand_;
    xfer[0].len = 1;
    xfer[1].rx_buf = (uintptr_t)id_out;
    xfer[1].len = 3;
    xfer[1].cs_change = true;
    ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer);
}

void SPIFRAM::read(uint32_t addr, void *buf, size_t len)
{
    LockIfPresent l(lock_);

    // Build the read-command + address header.
    uint8_t rdreq[4];
    unsigned hlen = 0;
    rdreq[hlen++] = cfg_->readCommand_;
    if (cfg_->addrBytes_ == 3)
    {
        rdreq[hlen++] = (addr >> 16) & 0xFF;
    }
    rdreq[hlen++] = (addr >> 8) & 0xFF;
    rdreq[hlen++] = addr & 0xFF;

    struct spi_ioc_transfer xfer[2] = {0, 0};
    // Segment 0: command + address (CS stays asserted after this segment).
    xfer[0].tx_buf = (uintptr_t)rdreq;
    xfer[0].len = hlen;
    // Segment 1: receive payload (CS deasserted at end via cs_change).
    xfer[1].rx_buf = (uintptr_t)buf;
    xfer[1].len = len;
    xfer[1].cs_change = true;
    ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer);
}

void SPIFRAM::write(uint32_t addr, const void *buf, size_t len)
{
    LockIfPresent l(lock_);

    // Build the write-command + address header.
    uint8_t wreq[4];
    unsigned hlen = 0;
    wreq[hlen++] = cfg_->writeCommand_;
    if (cfg_->addrBytes_ == 3)
    {
        wreq[hlen++] = (addr >> 16) & 0xFF;
    }
    wreq[hlen++] = (addr >> 8) & 0xFF;
    wreq[hlen++] = addr & 0xFF;

    struct spi_ioc_transfer xfer[3] = {0, 0, 0};
    // Segment 0: WREN — CS is toggled (asserted then deasserted) by cs_change
    // so that the Write Enable Latch is set before the data write begins.
    xfer[0].tx_buf = (uintptr_t)&cfg_->writeEnableCommand_;
    xfer[0].len = 1;
    xfer[0].cs_change = true;
    // Segment 1: write command + address (CS stays asserted after this).
    xfer[1].tx_buf = (uintptr_t)wreq;
    xfer[1].len = hlen;
    // Segment 2: payload data (CS deasserted at end via cs_change).
    // FRAM has no page-boundary constraint, so the full buffer is sent here.
    xfer[2].tx_buf = (uintptr_t)buf;
    xfer[2].len = len;
    xfer[2].cs_change = true;
    ::ioctl(spiFd_, SPI_IOC_MESSAGE(3), xfer);

    // No busy-polling: FRAM writes complete within the clock cycle.
    // The Write Enable Latch is automatically cleared by the device.
}
