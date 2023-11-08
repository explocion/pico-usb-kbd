#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::OnceCell;

use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::{self, pac};
use hal::usb::UsbBus;
use rp_pico as bsp;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

use rtic::app;
use rtic_monotonics::rp2040::*;
use rtic_monotonics::*;

#[app(device = pac, peripherals = true, dispatchers = [UART1_IRQ])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        kbd_hid: HIDClass<'static, UsbBus>,
    }

    #[local]
    struct Local {
        kbd_dev: UsbDevice<'static, UsbBus>,
    }

    #[init(local = [usb_allocator: OnceCell<UsbBusAllocator<UsbBus>> = OnceCell::new()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = cx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(cx.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            bsp::XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        defmt::info!("clock configured!");

        let sio = hal::sio::Sio::new(cx.device.SIO);
        let pins = rp_pico::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let _led = pins
            .led
            .into_push_pull_output_in_state(hal::gpio::PinState::High);

        let usb_bus = UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        );

        let allocator = cx
            .local
            .usb_allocator
            .get_or_init(|| UsbBusAllocator::new(usb_bus));

        let token = create_rp2040_monotonic_token!();
        Timer::start(cx.device.TIMER, &resets, token);
        kbd::spawn().unwrap();

        (
            Shared {
                kbd_hid: HIDClass::new(&allocator, KeyboardReport::desc(), 10),
            },
            Local {
                kbd_dev: UsbDeviceBuilder::new(&allocator, UsbVidPid(0xc0de, 0xcafe))
                    .manufacturer("University of Michigan")
                    .product("KeyOracle")
                    .serial_number("Grid")
                    .build(),
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, shared = [kbd_hid], local = [kbd_dev], priority = 1)]
    fn usb(mut cx: usb::Context) {
        cx.shared
            .kbd_hid
            .lock(|hid| cx.local.kbd_dev.poll(&mut [hid]));
    }

    #[task(shared = [kbd_hid], priority = 2)]
    async fn kbd(mut cx: kbd::Context) {
        loop {
            match cx.shared.kbd_hid.lock(|hid| {
                hid.push_input(&KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [4, 0, 0, 0, 0, 0],
                })
            }) {
                Ok(_) => (),
                Err(UsbError::WouldBlock) => defmt::warn!("host not detected!"),
                Err(e) => defmt::error!("{:?}", e),
            }
            Timer::delay(10.millis()).await;
            match cx.shared.kbd_hid.lock(|hid| {
                hid.push_input(&KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [0, 0, 0, 0, 0, 0],
                })
            }) {
                Ok(_) => (),
                Err(UsbError::WouldBlock) => defmt::warn!("host not detected!"),
                Err(e) => defmt::error!("{:?}", e),
            }
            Timer::delay(200.millis()).await;
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }
}
