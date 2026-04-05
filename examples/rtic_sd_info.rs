//! Demonstrates SD card initialization on the Teensy 4.1.
//!
//! Insert a FAT32-formatted SD card, flash this example, and connect to the
//! USB serial port to see card info and a root directory listing.
//!
//! This example requires a Teensy 4.1 (which has the built-in micro-SD slot).

#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::{board, hal};

    use imxrt_usdhc::{embedded_sdmmc, Usdhc};
    use imxrt_log as logging;
    use rtic_monotonics::systick::*;

    /// Dummy time source for FAT timestamps.
    struct FakeTime;

    impl embedded_sdmmc::TimeSource for FakeTime {
        fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
            embedded_sdmmc::Timestamp {
                year_since_1970: 56, // 2026
                zero_indexed_month: 2,
                zero_indexed_day: 29,
                hours: 0,
                minutes: 0,
                seconds: 0,
            }
        }
    }

    #[local]
    struct Local {
        poller: logging::Poller,
        led: board::Led,
        sd: Option<Usdhc>,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            mut pins,
            mut gpio2,
            usdhc1,
            ..
        } = board::t41(cx.device);
        let led = board::led(&mut gpio2, pins.p13);

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        // Configure SD card pins for USDHC1 (p42–p47 on Teensy 4.1).
        hal::iomuxc::usdhc::prepare(&mut pins.p45); // CMD
        hal::iomuxc::usdhc::prepare(&mut pins.p44); // CLK
        hal::iomuxc::usdhc::prepare(&mut pins.p43); // DATA0
        hal::iomuxc::usdhc::prepare(&mut pins.p42); // DATA1
        hal::iomuxc::usdhc::prepare(&mut pins.p47); // DATA2
        hal::iomuxc::usdhc::prepare(&mut pins.p46); // DATA3

        // Initialize SD card during init (blocking, before USB is up).
        // The result is passed to the sd_info task for logging once
        // the USB host has had time to connect.
        let sd = match Usdhc::new(usdhc1, board::USDHC1_FREQUENCY) {
            Ok(sd) => Some(sd),
            Err(e) => {
                // Can't log yet — USB isn't enumerated. The sd_info task
                // will detect None and report the failure.
                let _ = e;
                None
            }
        };

        sd_info::spawn().unwrap();
        led.set();

        (Shared {}, Local { poller, led, sd })
    }

    /// Wait for the USB host to connect, then log SD card info.
    #[task(local = [sd])]
    async fn sd_info(cx: sd_info::Context) {
        // Give the host time to enumerate the USB device and open the serial port.
        Systick::delay(2000.millis()).await;

        let Some(sd) = cx.local.sd.take() else {
            log::error!("SD card not initialized (no card or init failed)");
            return;
        };

        log::info!(
            "SD card: {:?}, {} MB ({} blocks)",
            sd.card_type(),
            sd.capacity_mb(),
            sd.block_count(),
        );

        let volume_mgr = embedded_sdmmc::VolumeManager::new(sd, FakeTime);

        match volume_mgr.open_raw_volume(embedded_sdmmc::VolumeIdx(0)) {
            Ok(volume) => {
                match volume_mgr.open_root_dir(volume) {
                    Ok(root_dir) => {
                        log::info!("Root directory:");
                        let _ = volume_mgr.iterate_dir(root_dir, |entry| {
                            log::info!("  {}", entry.name);
                        });
                        let _ = volume_mgr.close_dir(root_dir);
                    }
                    Err(e) => log::error!("open root dir: {:?}", e),
                }
                let _ = volume_mgr.close_volume(volume);
            }
            Err(e) => log::error!("open volume: {:?}", e),
        }

        log::info!("Done.");
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Systick::delay(500.millis()).await;
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}
