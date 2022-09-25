#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;


#[defmt_test::tests]
mod tests {
    use defmt::*;
    use nucleo_wl55jc_bsp::hal::{
        watchdog::WindowWatchdog,
        embedded_hal::watchdog::WatchdogEnable,
        rcc,
        pac::{
            self,
            interrupt
        }
    };

    const MSI_FREQ : u32 = 8_000_000;
    const PCLK1_FREQ : u32 = 1_000_000;

    struct TestArgs {
        wwdg: pac::WWDG,
        rcc: pac::RCC,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_hsi(
                &mut dp.FLASH,
                &mut dp.PWR,
                &mut dp.RCC,
                cs,
            )
        });

        assert_eq!(rcc::sysclk_hz(&dp.RCC), MSI_FREQ);
        assert_eq!(rcc::pclk1_hz(&dp.RCC), PCLK1_FREQ);

        TestArgs {
            rcc: dp.RCC,
            wwdg: dp.WWDG,
        }
    }

    #[test]
    fn accurate_interval(ta: &mut TestArgs) {
        let wdg = WindowWatchdog::new(ta.wwdg, &ta.rcc);

        wdg.start(5120);

        assert_eq!(wdg.interval_us(&ta.rcc));
    }

    #[test]
    fn below_minimum_interval(ta: &mut TestArgs) {
        let wdg = WindowWatchdog::new(ta.wwdg, &ta.rcc);

        wdg.start(5120);

        assert_eq!(wdg.interval_us(&ta.rcc));
    }
}