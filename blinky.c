#include "stm32f4xx.h"
/*
 * CMSIS device header for STM32F4.
 *
 * Why this matters:
 *   - Provides the memory-mapped peripheral base addresses (RCC, GPIOC, TIM2, …).
 *   - Provides register layouts (RCC->CR, GPIOC->MODER, TIM2->PSC, …).
 *   - Provides bitfield masks/positions (RCC_CR_HSEON, RCC_PLLCFGR_PLLM_Pos, …).
 *
 * Practical consequence:
 *   Every statement like “RCC->CR |= …” is a direct write to a documented register in the
 *   STM32F401 reference manual (RM0368 family) via CMSIS names.
 */

/* System clock: 84 MHz from 25 MHz HSE */
static void clock_init(void)
{
    RCC->CR |= RCC_CR_HSEON;
    /*
     * RCC_CR.HSEON: request enabling the external high-speed oscillator (HSE).
     *
     * Why this is done:
     *   - The PLL input source is typically either HSI (internal RC) or HSE (external crystal/clock).
     *   - If you want a deterministic, accurate 84 MHz core clock derived from a 25 MHz source,
     *     the PLL must be sourced from HSE (set later in RCC->PLLCFGR).
     *
     * Hardware effect:
     *   - Starts the HSE oscillator circuitry.
     *   - HSE takes time to stabilize; hence the “ready” poll below.
     */

    while ((RCC->CR & RCC_CR_HSERDY) == 0);
    /*
     * RCC_CR.HSERDY: indicates HSE is stable and usable.
     *
     * Why this is done:
     *   - Using an unstabilized oscillator as PLL input can produce invalid PLL locking behavior.
     *   - Downstream clocks (CPU, buses, Flash interface timing) depend on the PLL being valid.
     *
     * Architectural note:
     *   This is a blocking poll; in bring-up code that is acceptable and deterministic.
     */

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    /*
     * RCC_APB1ENR.PWREN: enables the APB1 clock to the PWR peripheral.
     *
     * Why this is done:
     *   - PWR registers are not accessible unless their peripheral clock is enabled.
     *   - Voltage scaling (VOS) is controlled by the PWR block and must be set before running
     *     at higher frequencies to meet device timing requirements.
     */

    PWR->CR |= PWR_CR_VOS;
    /*
     * PWR_CR.VOS: voltage regulator scaling configuration.
     *
     * Why this is done:
     *   - Higher system frequencies require a higher internal regulator performance level.
     *   - If the regulator scaling is too low, operation at 84 MHz may violate timing margins,
     *     causing unpredictable behavior.
     *
     * Device-specific note:
     *   The intended VOS setting depends on the STM32F401 variant and reference-manual guidance
     *   for maximum frequency vs voltage range. This line is asserting the “performance” setting
     *   needed for 84 MHz operation.
     */

    FLASH->ACR =
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN |
        FLASH_ACR_LATENCY_2WS;
    /*
     * FLASH_ACR: Flash access control register.
     *
     * Why this is done:
     *   - At higher CPU clocks, Flash reads must be pipelined with wait states.
     *   - Instruction and data caches reduce stall cycles by reusing recently fetched lines.
     *   - Prefetch improves throughput for sequential accesses.
     *
     * Individual fields:
     *   - ICEN: enable instruction cache (reduces instruction fetch stalls).
     *   - DCEN: enable data cache (reduces literal/constant table and data fetch stalls).
     *   - PRFTEN: enable prefetch (helps keep pipeline fed).
     *   - LATENCY_2WS: set Flash wait states to a value compatible with the target HCLK.
     *
     * Failure mode if omitted:
     *   If Flash latency is too low for the chosen HCLK, the CPU can fetch incorrect instructions/data
     *   or fault due to timing violations.
     */

    RCC->PLLCFGR =
        (25  << RCC_PLLCFGR_PLLM_Pos) |
        (336 << RCC_PLLCFGR_PLLN_Pos) |
        (1   << RCC_PLLCFGR_PLLP_Pos) |   /* PLLP = 4 */
        RCC_PLLCFGR_PLLSRC_HSE;
    /*
     * RCC_PLLCFGR: PLL configuration register.
     *
     * Why this is done:
     *   - Defines the multiplication/division chain that converts the HSE source into system clocks.
     *   - For STM32F4, PLL parameters are constrained so that:
     *       VCO input  = HSE / PLLM        must be in an allowed MHz range,
     *       VCO output = VCO input * PLLN  must be in an allowed MHz range,
     *       PLLP output (SYSCLK) = VCO output / PLLP.
     *
     * Values chosen here (intent):
     *   - PLLM = 25  : 25 MHz / 25 = 1 MHz VCO input.
     *   - PLLN = 336 : 1 MHz * 336 = 336 MHz VCO output.
     *   - PLLP = 4   : 336 MHz / 4 = 84 MHz SYSCLK.
     *   - PLLSRC_HSE : choose HSE as PLL source.
     *
     * Note on PLLP encoding:
     *   PLLP is encoded; the comment “PLLP = 4” indicates the author’s intended divider.
     *   The shift uses RCC_PLLCFGR_PLLP_Pos; the literal “1” corresponds to the encoding value
     *   that selects divide-by-4 in the STM32F4 PLLP field definition.
     *
     * Failure mode if incorrect:
     *   - If PLL input/output ranges are violated, PLL may not lock (PLLRDY never asserts).
     *   - If SYSCLK is not what the rest of the code assumes, timer prescalers become wrong.
     */

    RCC->CR |= RCC_CR_PLLON;
    /*
     * RCC_CR.PLLON: enable the main PLL.
     *
     * Why this is done:
     *   - Powers and starts the PLL locking process using the previously programmed PLLCFGR.
     */

    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    /*
     * RCC_CR.PLLRDY: PLL lock indicator.
     *
     * Why this is done:
     *   - Switching system clock source to the PLL before lock is unsafe.
     *   - System clock must be stable before bus prescalers and peripherals rely on it.
     */

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    /*
     * RCC_CFGR.SW: system clock switch.
     *
     * Why this is done:
     *   - Selects PLL output as the SYSCLK source (instead of HSI/HSE).
     *
     * System-level effect:
     *   - Once switched, HCLK and derived clocks are driven from PLL (subject to prescalers).
     */

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    /*
     * RCC_CFGR.SWS: system clock switch status.
     *
     * Why this is done:
     *   - Confirms the hardware mux has actually switched SYSCLK to PLL.
     *   - This avoids proceeding while the CPU might still be running on a different source.
     */
}

/* PC13 as output */
static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    /*
     * RCC_AHB1ENR.GPIOCEN: enable AHB1 clock for GPIOC.
     *
     * Why this is done:
     *   - GPIO peripheral registers are clock-gated.
     *   - Without this, writes to GPIOC configuration registers are ignored.
     *
     * Practical debugging note:
     *   “GPIO doesn’t work” symptoms often reduce to “clock not enabled.”
     */

    GPIOC->MODER &= ~(3U << (13 * 2));
    /*
     * GPIOC_MODER: two mode bits per pin.
     *
     * Why this is done:
     *   - Clears PC13 mode field to a known baseline before setting it.
     *   - Avoids accidentally leaving the pin in alternate-function or analog mode.
     *
     * Bitfield:
     *   PC13 uses bits [27:26] = 2*(13) .. 2*(13)+1.
     */

    GPIOC->MODER |=  (1U << (13 * 2));    /* Output */
    /*
     * Sets PC13 mode to 01 (general purpose output).
     *
     * Why this is done:
     *   - Connects the pin to the GPIO output driver instead of input sampling or AF muxing.
     */

    GPIOC->OTYPER &= ~(1U << 13);         /* Push-pull */
    /*
     * GPIOC_OTYPER: output type for each pin.
     *
     * Why this is done:
     *   - Push-pull can actively drive both high and low.
     *   - If the LED is wired active-low (common), the “low” drive must be strong and defined.
     *
     * Alternative (open-drain) would require an external pull-up to define the high state.
     */

    GPIOC->OSPEEDR |= (1U << (13 * 2));   /* Medium speed */
    /*
     * GPIOC_OSPEEDR: output slew-rate control (two bits per pin).
     *
     * Why this is done:
     *   - Controls edge speed (EMI vs transition time).
     *   - Medium speed is typically adequate for an LED and avoids needlessly fast edges.
     *
     * Note:
     *   This line ORs-in a single bit, which sets OSPEEDR[27:26] partially. If the other bit is
     *   left at reset-state 0, the resulting 2-bit value becomes 01 (often “medium” depending on
     *   STM32F4 encoding). The exact mapping is defined in the GPIO chapter.
     */

    GPIOC->PUPDR &= ~(3U << (13 * 2));    /* No pull */
    /*
     * GPIOC_PUPDR: pull-up/pull-down control (two bits per pin).
     *
     * Why this is done:
     *   - Internal pulls are unnecessary when driving an LED with an external resistor network.
     *   - Internal pulls can create unintended bias currents or alter LED brightness subtly.
     */
}

/* TIM2 @ 2 Hz update interrupt */
static void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    /*
     * RCC_APB1ENR.TIM2EN: enable APB1 clock for TIM2.
     *
     * Why this is done:
     *   - TIM2 registers are clock-gated like other peripherals.
     *   - Without this, prescaler/ARR writes do not take effect and the timer will not run.
     */

    TIM2->PSC = 8399;     /* 84 MHz / 8400 = 10 kHz */
    /*
     * TIM2_PSC: prescaler.
     *
     * Why this is done:
     *   - Reduces the timer input clock to a convenient “tick” rate.
     *   - With a presumed timer clock of 84 MHz, PSC=8399 yields:
     *       f_tick = 84,000,000 / (8399 + 1) = 10,000 Hz
     *
     * Reference-manual concept:
     *   The prescaler divides the counter clock; the counter increments at the divided rate.
     */

    TIM2->ARR = 4999;     /* 10 kHz / 5000 = 2 Hz */
    /*
     * TIM2_ARR: auto-reload register (period).
     *
     * Why this is done:
     *   - Sets the counter period for the update event.
     *   - With a 10 kHz tick and ARR=4999:
     *       update rate = 10,000 / (4999 + 1) = 2 Hz
     *
     * Hardware effect:
     *   - When CNT reaches ARR, an update event occurs and UIF is set in TIM2->SR.
     */

    TIM2->DIER |= TIM_DIER_UIE;   /* Update interrupt enable */
    /*
     * TIM2_DIER.UIE: update interrupt enable.
     *
     * Why this is done:
     *   - Converts the internal update event into an interrupt request to the NVIC.
     *   - Without UIE, UIF may still set, but no IRQ is generated.
     */

    TIM2->CR1  |= TIM_CR1_CEN;    /* Counter enable */
    /*
     * TIM2_CR1.CEN: counter enable.
     *
     * Why this is done:
     *   - Starts the timer counting.
     *   - Without this, the prescaler/ARR configuration is inert.
     */

    NVIC_EnableIRQ(TIM2_IRQn);
    /*
     * NVIC: Nested Vectored Interrupt Controller (Cortex-M4 core peripheral).
     *
     * Why this is done:
     *   - Even if TIM2 asserts an interrupt request, the CPU will not vector to TIM2_IRQHandler
     *     unless the corresponding NVIC enable bit is set.
     *
     * Architectural effect:
     *   - Enables the delivery of TIM2 IRQ to the core exception mechanism.
     */
}

/* TIM2 interrupt handler */
void TIM2_IRQHandler(void)
{
    /*
     * This function name must match the vector table entry for TIM2.
     *
     * Why this is done:
     *   - On an interrupt, the core fetches the handler address from the vector table and branches
     *     to it.
     *   - Startup code supplies the vector table; the symbol name must match that table’s entry.
     */

    if (TIM2->SR & TIM_SR_UIF) {
        /*
         * TIM2_SR.UIF: update interrupt flag.
         *
         * Why this is done:
         *   - Confirms that the interrupt was caused by an update event.
         *   - Defensive coding: some timers can have multiple interrupt sources.
         */

        TIM2->SR &= ~TIM_SR_UIF;          /* Clear flag */
        /*
         * Clears UIF so the interrupt does not immediately retrigger.
         *
         * Why this is done:
         *   - Many STM32 timer flags are “write 0 to clear” in SR.
         *   - If you do not clear UIF, the pending condition remains true, and the CPU may re-enter
         *     the handler continuously (interrupt storm).
         */

        GPIOC->ODR ^= (1U << 13);        /* Toggle LED */
        /*
         * GPIOC_ODR: output data register.
         *
         * Why this is done:
         *   - Toggles the output latch for PC13 on each TIM2 update interrupt.
         *   - With a 2 Hz interrupt rate, this produces a 1 Hz blink pattern (on/off each half-cycle)
         *     if the LED is active-low and the initial state is high.
         *
         * Electrical meaning (active-low LED typical):
         *   - PC13 = 0 → LED current flows → LED ON.
         *   - PC13 = 1 → no current → LED OFF.
         *
         * Atomicity note:
         *   ODR read-modify-write is not atomic with respect to concurrent writers (e.g., other ISRs).
         *   For a single-writer demo, it is acceptable. For multi-writer designs, BSRR is preferred.
         */
    }
}

int main(void)
{
    clock_init();
    /*
     * Establishes the assumed clock tree used by later code.
     *
     * Why this is done first:
     *   - tim2_init() prescaler calculations assume a specific timer clock frequency.
     *   - Flash latency and voltage scaling must be correct before sustaining higher HCLK.
     */

    gpio_init();
    /*
     * Configures the LED pin as output.
     *
     * Why before enabling interrupts:
     *   - If the timer IRQ fires before GPIO is configured, the ISR toggles an uninitialized pin.
     */

    tim2_init();
    /*
     * Enables TIM2 update interrupts and starts the counter.
     *
     * Why this enables blinking:
     *   - The timer periodically sets UIF.
     *   - UIE + NVIC enable causes the core to vector to TIM2_IRQHandler().
     *   - The ISR toggles the GPIO output state.
     */

    while (1) {
        __WFI();   /* Sleep until interrupt */
        /*
         * __WFI(): “Wait For Interrupt” (Cortex-M instruction).
         *
         * Why this is done:
         *   - Prevents the core from burning cycles in a busy-wait loop.
         *   - Places the core in sleep until an enabled interrupt becomes pending.
         *
         * Core behavior:
         *   - Execution stops; the pipeline is quiesced.
         *   - On TIM2 interrupt, the core wakes, services ISR, then returns here and executes WFI again.
         *
         * System behavior:
         *   - Peripheral clocks (TIM2, GPIO) continue running while the core sleeps,
         *     as long as they are not explicitly gated in a low-power mode configuration.
         */
    }
}


