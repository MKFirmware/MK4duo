/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Fast I/O interfaces for STM32
 * These use GPIO register access for fast port manipulation.
 */

// ------------------------
// Defines
// ------------------------
#define GPIO2PORT(PIN)  ((PIN)/16)
#define GPIO2BIT(PIN)   (_BV((PIN) % 16))

#define OUTPUT_LOW  0x4
#define OUTPUT_HIGH 0x5

// Set the mode and extended function of a pin
static void gpio_peripheral(uint32_t pin, uint32_t mode, int pullup) {

  GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];

  // Enable GPIO clock
  uint32_t rcc_pos = ((uint32_t)regs - AHB1PERIPH_BASE) / 0x400;
  RCC->AHB1ENR |= 1 << rcc_pos;
  RCC->AHB1ENR;

  // Configure GPIO
  const uint32_t  mode_bits = mode & 0xf,
                  func = (mode >> 4) & 0xf,
                  od = mode >> 8,
                  pup = pullup ? (pullup > 0 ? 1 : 2) : 0,
                  pos = pin % 16,
                  af_reg = pos / 8,
                  af_shift = (pos % 8) * 4,
                  af_msk = 0x0f << af_shift,
                  m_shift = pos * 2,
                  m_msk = 0x03 << m_shift;

  regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
  regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
  regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
  regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);
  regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (0x02 << m_shift);

}

// Read a pin
FORCE_INLINE static bool READ(const uint8_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      return pcf8574.digitalRead(pin - PIN_START_FOR_PCF8574);
    }
    else
  #endif
  {
    GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];
    return !!(regs->IDR & GPIO2BIT(pin));
  }
}

// Write to a pin
FORCE_INLINE static void WRITE(const uint8_t pin, const bool flag) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, flag);
    }
    else
  #endif
  {
    GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];
    if (flag)
      regs->BSRR = GPIO2BIT(pin);
    else
      regs->BSRR = GPIO2BIT(pin) << 16;
  }
}

// Toogle pin
FORCE_INLINE static void TOGGLE(const uint8_t pin) {
  GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];
  regs->ODR ^= GPIO2BIT(pin);
}

// Set pin as input
FORCE_INLINE static void SET_INPUT(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, INPUT);
    }
    else
  #endif
    gpio_peripheral(pin, INPUT, 0);
}

// Set pin as input with pullup
FORCE_INLINE static void SET_INPUT_PULLUP(const pin_t pin) {
  gpio_peripheral(pin, INPUT, 1);
}

// Set pin as input analog
FORCE_INLINE static void SET_INPUT_ANALOG(const pin_t pin) {
  gpio_peripheral(pin, 0x3, 0);
}

// Set pin as output
FORCE_INLINE static void SET_OUTPUT(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
    }
    else
  #endif
    gpio_peripheral(pin, OUTPUT, 0);
}
FORCE_INLINE static void SET_OUTPUT_LOW(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, LOW);
    }
    else
  #endif
  {
    GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];
    regs->BSRR = GPIO2BIT(pin) << 16;
    gpio_peripheral(pin, OUTPUT, 0);
  }
}
FORCE_INLINE static void SET_OUTPUT_HIGH(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, HIGH);
    }
    else
  #endif
  {
    GPIO_TypeDef *regs = GPIOPort[GPIO2PORT(pin)];
    regs->BSRR = GPIO2BIT(pin);
    gpio_peripheral(pin, OUTPUT, 0);
  }
}

// Shorthand
FORCE_INLINE static void OUT_WRITE(const pin_t pin, const uint8_t flag) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, flag);
    }
    else
  #endif
  {
    flag ? SET_OUTPUT_HIGH(pin) : SET_OUTPUT_LOW(pin);
  }
}

FORCE_INLINE static bool USEABLE_HARDWARE_PWM(const pin_t pin) {
  return digitalPinHasPWM(pin);
}
