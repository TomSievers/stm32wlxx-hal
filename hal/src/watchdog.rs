//! Hardware watchdog.

use crate::{
    pac::{self, WWDG, RCC},
    rcc::pclk1_hz,
    embedded_hal::watchdog::{WatchdogEnable, Watchdog}
};

const WWDG_DIV : u32 = 4096;
const WWDG_MAX_PRESCALER : u32 = 128;
const WWDG_MAX_RELOAD : u32 = 0x3F;

/// Driver for window watchdog (WWDG) peripheral;
pub struct WindowWatchdog {
    wwdg : WWDG,
    reload : u8
}

impl WindowWatchdog {
    /// Create a new WindowWatchdog, enables the clock for the WWDG.
    /// 
    /// # Arguments
    /// 
    /// * `wwdg` - WWDG Peripheral needed for the driver.
    /// * `rcc` - RCC Peripheral needed to enable the WWDG.
    #[inline]
    pub fn new(wwdg : WWDG, rcc : &RCC) -> Self {
        rcc.apb1enr1.modify(|_, w| w.wwdgen().enabled());
        WindowWatchdog { wwdg, reload : 0 }
    }

    #[inline]
    /// Steal the WindowWatchdog from whatever is using it.
    /// 
    /// This does NOT initialize the WindowWatchdog (unlike [`new`]).
    /// 
    /// # Safety
    /// 
    /// 1. Ensure that the code stealing the WWDG peripheral has exclusive access.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for enabling the WWDG peripheral clock before use.
    pub unsafe fn steal() -> Self {
        let dp = pac::Peripherals::steal();
        WindowWatchdog { wwdg: dp.WWDG, reload: 0 }
    }

    /// Set the window for this watchdog to open after the given microseconds. 
    /// If the given target microseconds results in less than 1 clock cycle 
    /// a panic will be created by a debug_assert.
    /// 
    /// If the APB1 clock (PCLK1) is changed after this window has been set the resulting 
    /// real-time duration of the window will be incorrect. 
    /// 
    /// To ensure proper operation do NOT change the APB1 clock (PCLK1)
    /// after the window watchdog was configured.
    /// 
    /// # Arguments
    /// 
    /// * `us` - The number of microseconds until the window opens.
    /// * `rcc` - RCC Peripheral needed to determine the time.
    pub fn set_window_us(&self, us : u32, rcc : &RCC) {
        let us_per_cycle = self.clock_period_us(rcc);

        let cycles = us / us_per_cycle;

        debug_assert!(cycles > 0);

        self.wwdg.cfr.modify(|_, w| w.w().bits(0x40 & cycles as u8));
    }

    /// Configure the watchdog to stop while the cpu is halted during debugging.
    /// 
    /// # Arguments
    /// 
    /// * `dbg` - Instance of DBGMCU from PAC needed to configure.
    /// * `stop` - Boolean to configure if the watchdog should be stopped while the cpu is halted.
    #[cfg(not(feature = "stm32wl5x_cm0p"))]
    #[inline]
    pub fn stop_on_debug(&self, dbg : &pac::DBGMCU, stop : bool) {
        dbg.apb1fzr1.modify(|_, w| w.dbg_wwdg_stop().bit(stop));
    }

    /// Get the currently configured interval of the watchdog in microseconds.
    /// 
    /// # Arguments
    /// 
    /// * `rcc` - Instance of RCC from PAC needed to get the PCLK1 speed.
    pub fn interval_us(&self, rcc : &RCC) -> u32 {
        let us_per_cycle = self.clock_period_us(rcc);
        self.reload as u32 * us_per_cycle
    }

    /// Enable or disable the early wakeup interrupt for the window watchdog.
    /// 
    /// # Arguments
    /// 
    /// * `enabled` - Boolean for if the EWI should be enabled.
    #[inline]
    pub fn enable_ewi(&self, enabled : bool) {
        self.wwdg.cfr.modify(|_, w|w.ewi().bit(enabled));
    }

    /// Unmask the WWDG interrupt.
    /// 
    /// # Safety
    /// 
    /// This may break mask based critical sections.
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[inline]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::WWDG);
    }

    /// Mask the WWDG interrupt.
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[inline]
    pub fn mask_irq() {
        pac::NVIC::mask(pac::Interrupt::WWDG);
    }

    /// Get access to the underlying register block.
    /// 
    /// # Safety 
    /// 
    /// This function is not memory unsafe per se, 
    /// but does not guarantee anything about assumptions of 
    /// invariants made in this implementation.
    /// 
    /// Changing specific options can lead to un-expected behavior and nothing is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut WWDG {
        &mut self.wwdg
    }

    #[inline]
    fn clock_period_us(&self, rcc : &RCC) -> u32 {
        let pclk = pclk1_hz(rcc);

        let div = 1u32 << self.wwdg.cfr.read().wdgtb().bits();

        1_000_000 / ((pclk / WWDG_DIV) / div)
    }

    fn setup(&mut self, target_us : u32, rcc : &RCC) {
        let us_per_cycle = self.clock_period_us(rcc);

        let mut reload = target_us / us_per_cycle;

        let mut psc = 0;
        loop {
            if psc >= WWDG_MAX_PRESCALER as u8 {
                // ceil the value to the maximum allow reload value
                if reload > WWDG_MAX_RELOAD {
                    reload = WWDG_MAX_RELOAD;
                }
                break;
            }
            if reload <= WWDG_MAX_RELOAD {
                break;
            }
            psc += 1;
            // When the precaler value increased, the reload value has to be halfed
            // so that the timeout stays the same.
            reload /= 2;
        }

        self.reload = reload as u8;

        self.feed();
    }
}

impl Watchdog for WindowWatchdog {
    #[inline]
    fn feed(&mut self) {
        self.wwdg.cr.write(|w| w.t().bits(0x40 & self.reload))
    }
}

impl WatchdogEnable for WindowWatchdog {
    type Time = u32;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time> {
        let dp = unsafe{pac::Peripherals::steal()};
        self.setup(period.into(), &dp.RCC);

        self.wwdg.cr.modify(|_, w|w.wdga().enabled());
    }
}