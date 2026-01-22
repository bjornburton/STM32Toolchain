# STM32F401CCU6 Bare-Metal Linux Toolchain Notes

These notes document the toolchain, build inputs, and debug workflow assembled for  
**bare-metal STM32F401CCU6** development (no HAL, no LL, no RTOS).

---

## Target Hardware

- **MCU:** STM32F401CCU6 (Cortex-M4F)
- **Flash:** 256 KiB at `0x0800_0000`
- **SRAM:** 64 KiB at `0x2000_0000 – 0x2000_FFFF`
  - Top of stack: `0x2001_0000`
- **Debug interface:** SWD
- **Probe:** ST-LINK V3-MINIE
- **Board:** STM32 Mini F401/HW-848. See https://stm32-base.org/boards/

For V3-MINIE access, file `/etc/udev/rules.d/49-stlinkv3.rules` should contain:
```text
# STMicroelectronics ST-LINK V1/V2/V3
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", MODE="0666", GROUP="plugdev"
```

---

## Repository Layout

Example working directory for LED blinker:

```text
~/ST32/
  CMSIS_5/
  cmsis-device-f4/
  blinky/
    blinky.c
    Makefile
    stm32f401.ld
```

---

## Toolchain

### Compiler and Binutils

- **Compiler / Link driver:** `arm-none-eabi-gcc`
- **Assembler:** `arm-none-eabi-as` (invoked by GCC)
- **Linker:** `arm-none-eabi-ld` (invoked by GCC)
- **Size tool:** `arm-none-eabi-size`

### Typical Compiler Flags

```text
-mcpu=cortex-m4
-mthumb
-mfpu=fpv4-sp-d16
-mfloat-abi=hard
-ffreestanding
-fdata-sections
-ffunction-sections
-Wall -Wextra
```

### Build Modes

#### Release

```text
-O2
-DNDEBUG
```

#### Debug

```text
-Og
-g3
```

---


## CMSIS and Startup Files

https://github.com/ARM-software/CMSIS_5.git

https://github.com/STMicroelectronics/cmsis-device-f4.git


### Include Paths

```text
-I ~/ST32/CMSIS_5/CMSIS/Core/Include
-I ~/ST32/cmsis-device-f4/Include
```

### Mandatory Device Selection Macro

```text
-DSTM32F401xC
```

This selects the correct device header content (interrupt enumerations, peripheral definitions, and FPU presence macros).

### Startup and System Files

From STM32CubeF4:

- **Startup (GCC variant):**
  ```text
  cmsis-device-f4/Source/Templates/gcc/startup_stm32f401xc.s
  ```

- **System clock support:**
  ```text
  cmsis-device-f4/Source/Templates/system_stm32f4xx.c
  ```

---

## No HAL, No LL

This environment uses:

- CMSIS-Core (core headers, intrinsics such as `__WFI()`)
- CMSIS-Device (STM32F4xx register definitions and startup templates)

It does not use:

- STM32 HAL
- STM32 LL drivers
- CubeMX code generation
- Middleware frameworks
- RTOS

All peripheral configuration is performed by direct register access (RCC, GPIO, TIM, NVIC, etc.).

---

## Linker Script Notes

### Memory Map
The linker script `stm32f401.ld` is borrowed from Nucleo:
https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM32F401RE-Nucleo/Templates/STM32CubeIDE/STM32F401CEUX_FLASH.ld

But adjusted to suit STM32F401CCU6 RAM size:

```ld
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
  ROM   (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
}
```

### Practical Importance

A mismatched RAM length caused an invalid MSP for me (for example, `0x20018000`), preventing reliable execution after reset. Correcting the linker script produced the expected debug state:

```text
pc   = 0x0800xxxx  (Flash)
msp  = 0x20010000  (top of 64 KiB SRAM)
xPSR = 0x01000000  (Thumb state)
```

---

## OpenOCD

Version 0.11.0-1 is known not to work with ST-LINK V3-MINIE.

### Version and Scripts

- OpenOCD: `0.12.0+dev` (https://github.com/openocd-org/openocd.git)
- Scripts installed under: `/usr/local/share/openocd/scripts`

### Interface and Target Scripts

- **Interface:** `interface/stlink.cfg`
- **Target:** `target/stm32f4x.cfg`

### Suppressing Transport Warning

Explicitly select SWD:

```text
-c "transport select swd"
```

### Start OpenOCD GDB Server

```bash
openocd \
  -s /usr/local/share/openocd/scripts \
  -f interface/stlink.cfg \
  -c "transport select swd" \
  -f target/stm32f4x.cfg \
  -c "adapter speed 1000"
```

Notes:

- `adapter speed 1000` kHz is conservative and stable during bring-up.
- Higher speeds may work after validating wiring and target behavior.

### Program Firmware (ELF Preferred)

```bash
openocd \
  -s /usr/local/share/openocd/scripts \
  -f interface/stlink.cfg \
  -c "transport select swd" \
  -f target/stm32f4x.cfg \
  -c "program blinky.elf verify reset exit"
```

Using the ELF avoids address and section ambiguity present with raw BIN files.

---

## GDB

### GDB Choice

`gdb-multiarch` was used successfully.

### Launch and Connect

Terminal 1: run OpenOCD (server) as above.

Terminal 2:

```bash
gdb-multiarch blinky.elf
```

Inside GDB:

```text
target extended-remote :3333
monitor reset halt
break main
continue
```

---

## Low-Level Peripheral Debugging

When CMSIS symbols are unavailable in GDB, registers can be accessed by address.

Example: enable GPIOA–GPIOE clocks via RCC AHB1ENR:

```text
monitor reset halt
set *(unsigned int *)0x40023830 = 0x0000001F
```

### Key Base Addresses (STM32F4 AHB1)

| Peripheral | Base Address |
|-----------|--------------|
| GPIOA | `0x40020000` |
| GPIOB | `0x40020400` |
| GPIOC | `0x40020800` |
| GPIOD | `0x40020C00` |
| GPIOE | `0x40021000` |
| RCC   | `0x40023800` |

### Common Offsets

| Register | Offset |
|---------|--------|
| MODER | `0x00` |
| ODR | `0x14` |
| BSRR | `0x18` |
| AHB1ENR (RCC) | `0x30` |

Thus:

```text
RCC->AHB1ENR = 0x40023830
```

---

## Binary Size Inspection

```bash
arm-none-eabi-size --format=berkeley blinky.elf
```

Output fields:

| Field | Meaning |
|------|--------|
| text | Code + read-only data (Flash) |
| data | Initialized data copied into RAM |
| bss  | Zero-initialized RAM |
| dec  | Total bytes (decimal) |
| hex  | Total bytes (hex) |

---

## Application Pattern in `blinky.c`

The project includes:

- System clock initialization (HSE → PLL → SYSCLK)
- GPIO configuration for LED control
- TIM2 configured for periodic update interrupts
- `TIM2_IRQHandler()` toggling the LED
- `__WFI()` in the main loop for sleep between interrupts

This demonstrates:

- Register-level RCC, GPIO, and TIM configuration
- NVIC interrupt enabling
- Low-power idle using the Cortex-M `WFI` instruction

---

## Design Philosophy

This setup emphasizes:

- Register-level transparency for datasheet correlation
- Deterministic behavior
- Minimal dependencies
- Longevity beyond vendor tooling changes
- Debuggability using OpenOCD + GDB without an IDE

**C → Registers → Silicon**  
No hidden layers, no generated code, no framework lock-in.
