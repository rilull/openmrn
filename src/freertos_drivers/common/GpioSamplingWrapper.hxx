/** \copyright
 * Copyright (c) 2026, Balazs Racz
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
 * \file GpioSamplingWrapper.hxx
 *
 * Wraps a static, dynamically-switchable hardware GPIO structure (such as an
 * @ref GpioHwPin declared via GPIO_XPIN) into an implementation of the
 * os-independent @ref Gpio class whose set_direction() actually reconfigures
 * the pin between input and output at runtime.
 *
 * This is the runtime adaptor that @ref ConfiguredSamplingIO uses so that the
 * driver can stay entirely hardware-agnostic (it only ever sees @ref Gpio*),
 * while all target-specific detail (which port, pull configuration, HAL
 * specifics for e.g. STM32L4 vs STM32F3) lives in the board-level pin
 * declaration.
 *
 * @author Balazs Racz
 * @date 24 Jul 2026
 */

#ifndef _FREERTOS_DRIVERS_COMMON_GPIOSAMPLINGWRAPPER_HXX_
#define _FREERTOS_DRIVERS_COMMON_GPIOSAMPLINGWRAPPER_HXX_

#include "os/Gpio.hxx"

/// Creates an implementation of an os-independent @ref Gpio object from a
/// static hardware GPIO structure that supports dynamic input/output
/// switching. Unlike @ref GpioWrapper, this adaptor permits changing the pin
/// direction: set_direction() maps to the pin's set_output() / set_input()
/// static methods, which reconfigure the hardware while preserving the
/// original output-drive (e.g. open-drain) and pull configuration
/// respectively.
///
/// @tparam PIN a static pin structure exposing set(bool), get(),
/// set_output(), set_input() and is_output(). @ref GpioHwPin (declared through
/// GPIO_XPIN with the GpioHwPin policy) satisfies this contract.
///
/// Usage:
/// @code
/// GPIO_XPIN(IO1, GpioHwPin, A, 5, Output(), PullNone());
/// ...
/// const Gpio *const kPins[] = {
///     GpioSamplingWrapper<IO1_Pin>::instance(),
/// };
/// @endcode
template <class PIN> class GpioSamplingWrapper : public Gpio
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the data section.
    constexpr GpioSamplingWrapper()
    {
    }

    void write(Value new_state) const override
    {
        PIN::set(new_state);
    }

    void set() const override
    {
        PIN::set(true);
    }

    void clr() const override
    {
        PIN::set(false);
    }

    Value read() const override
    {
        return PIN::get() ? VHIGH : VLOW;
    }

    void set_direction(Direction dir) const override
    {
        if (dir == Direction::DOUTPUT)
        {
            // Re-enables the output driver. On STM32 the ODR retains its last
            // value across the mode switch, so any level written via write()
            // while the pin was an input takes effect without a glitch.
            PIN::set_output();
        }
        else
        {
            // Switches to a high-impedance input, preserving the pull
            // configuration from the original pin declaration.
            PIN::set_input();
        }
    }

    Direction direction() const override
    {
        return PIN::is_output() ? Direction::DOUTPUT : Direction::DINPUT;
    }

    /// @return the static Gpio object instance controlling this pin.
    static constexpr const Gpio *instance()
    {
        return &instance_;
    }

    /// Singleton instance for this pin.
    static const GpioSamplingWrapper instance_;
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <class PIN>
const GpioSamplingWrapper<PIN> GpioSamplingWrapper<PIN>::instance_;

#endif // _FREERTOS_DRIVERS_COMMON_GPIOSAMPLINGWRAPPER_HXX_
