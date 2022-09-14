//! Hardware watchdog.

use crate::{
    pac::{IWDG, RCC, iwdg::pr::PR_A, Peripherals},
    rcc::lsi_hz,
    embedded_hal::watchdog::{Watchdog, WatchdogEnable}
};

#[cfg(not(feature = "stm32wl5x_cm0p"))]
use crate::pac::DBGMCU;

const MAX_PRESCALER: PR_A = PR_A::DivideBy256;
const MAX_RELOAD: u32 = 0x0FFF;

fn into_division_value(psc : PR_A) -> u32 {
    1 << (2 + psc as u8) 
}

#[derive(Debug)]
/// Independent Watchdog driver.
pub struct IndependentWatchDog {
    iwdg : IWDG
}

#[cfg(feature = "defmt")]
impl defmt::Format for IndependentWatchDog {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "IndependentWatchDog {{ iwdg: IWDG }}");
    }
}

impl IndependentWatchDog {
    /// Create a new `IndependentWatchdog` without starting it.
    /// 
    /// Call the start function to start the watchdog with a given period.
    /// 
    /// # Arguments
    /// 
    /// * `iwdg` - Instance of IWDG from PAC.
    pub fn new(iwdg : IWDG) -> Self {
        IndependentWatchDog { 
            iwdg
        }
    }

    #[cfg(not(feature = "stm32wl5x_cm0p"))]
    /// Configure the watchdog to stop while the cpu is halted during debugging.
    /// 
    /// # Arguments
    /// 
    /// * `dbg` - Instance of DBGMCU from PAC to configure.
    /// * `stop` - Boolean to configure if the watchdog should be stopped while the cpu is halted.
    pub fn stop_on_debug(&mut self, dbg: &DBGMCU, stop : bool) {
        dbg.apb1fzr1.modify(|_, w|w.dbg_iwdg_stop().bit(stop));
        #[cfg(any(feature = "stm32wl5x_cm0p", feature = "stm32wl5x_cm4"))]
        dbg.c2apb1fzr1.modify(|_, w|w.dbg_iwdg_stop().bit(stop));
    }

    /// Get the currently configured interval of the watchdog in milliseconds.
    /// 
    /// # Arguments
    /// 
    /// * `rcc` - Instance of RCC from PAC needed to get the LSI speed.
    pub fn interval_ms(&self, rcc : &RCC) -> u32 {
        let us_per_cycle = self.us_per_cycle(rcc);

        let rl = self.iwdg.rlr.read().rl().bits() as u32;

        // Convert microseconds to milliseconds after complete value has been calculated.
        (rl *  us_per_cycle) / 1000
    }

    fn us_per_cycle(&self, rcc : &RCC) -> u32{
        // Multiply frequency by 1000 so no float calculations would be needed with maximum divisor.
        let lsi_freq = lsi_hz(rcc) as u32 * 1000;

        let wdg_div = into_division_value(self.iwdg.pr.read().pr().variant());

        // Calculate microseconds per clock cycle, normal formula would be 1_000_000 / <freq_in_hz> 
        // but because frequency is multiplied by 1000 the lhs also needs to be multiplied by 1000.
        1_000_000_000 / (lsi_freq / wdg_div)
    }

    /// Get access to the underlying register block.
    /// 
    /// # Safety 
    /// 
    /// This function is not memory unsafe per se, but does not guarantee anything about assumptions of invariants made in this implementation.
    /// 
    /// Changing specific options can lead to un-expected behavior and nothing is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut IWDG {
        &mut self.iwdg
    }

    /// Setup the watchdog to try to hit the target in milliseconds.
    fn setup(&self, rcc : &RCC, millis : u32) {
        let lsi_freq = lsi_hz(rcc) as u32;

        let mut reload = millis * lsi_freq / into_division_value(PR_A::DivideBy4);

        // Reload is potentially too high to be stored in the register.
        // The goal of this loop is to find the maximum possible reload value,
        // which can be stored in the register, while still guaranteeing the wanted timeout.
        //
        // This is achived by increasing the prescaler value.
        let mut psc = 0;
        loop {
            if psc >= MAX_PRESCALER as u8 {
                // ceil the value to the maximum allow reload value
                if reload > MAX_RELOAD {
                    reload = MAX_RELOAD;
                }
                break;
            }
            if reload <= MAX_RELOAD {
                break;
            }
            psc += 1;
            // When the precaler value increased, the reload value has to be halfed
            // so that the timeout stays the same.
            reload /= 2;
        }

        self.lock_free(|wdg|{
            wdg.pr.modify(|_, w| w.pr().bits(psc));
            wdg.rlr.modify(|_, w| w.rl().bits(reload as u16));
        });
        
    }

    fn lock_free<R, F : Fn(&IWDG) -> R>(&self, f : F) -> R {
        self.iwdg.kr.write(|w| w.key().enable());

        let res = f(&self.iwdg);

        self.iwdg.kr.write(|w| w.key().reset());

        res
    }
}

impl WatchdogEnable for IndependentWatchDog {
    type Time = u32;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time> {
        let us : Self::Time = period.into();

        let dp = unsafe{Peripherals::steal()};

        self.setup(&dp.RCC, us);

        self.iwdg.kr.write(|w|w.key().start());
    }
}

impl Watchdog for IndependentWatchDog {
    fn feed(&mut self) {
        self.iwdg.kr.write(|w|w.key().reset());
    }
}