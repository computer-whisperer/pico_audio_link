#![no_std]
#![no_main]

#![feature(impl_trait_in_assoc_type)]

extern crate alloc;


use core::cmp::PartialEq;
use core::future::Future;
use cortex_m::prelude::_embedded_hal_blocking_serial_Write;
use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use {defmt_rtt as _, panic_probe as _};
use defmt::*;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::digital::{OutputPin, PinState};
use uom::si::f32::*;


use embedded_alloc::LlffHeap as Heap;
use static_cell::StaticCell;


use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_rp::{gpio, pwm, spi, peripherals, i2c, pio, uart, pac, dma, bind_interrupts};
use embassy_rp::gpio::{Output, Pin};
use fixed::FixedU32;
use uom::num::Zero;
use uom::si::pressure::{kilopascal, pascal};
use uom::si::electric_potential::volt;
use uom::si::electrical_resistance::{kiloohm, ohm};
use uom::si::thermodynamic_temperature::{degree_celsius, kelvin};
use micromath::F32;
use uom::si::electric_charge::milliampere_hour;
use uom::si::electric_charge::Units::coulomb;

type ProjectMutex = NoopRawMutex;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<peripherals::I2C1>;
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, gpio::Output<'static>, PioSpi<'static, peripherals::PIO0, 0, peripherals::DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();


#[cortex_m_rt::entry]
fn main() -> ! {
    // Clear spinlocks
    pac::SIO.spinlock(31).write_value(1);

    /*
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 10240;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }*/

    //pressure_control_test();

    /*
    let mut config = embassy_rp::config::Config::default();
    match config.clocks.xosc {
        Some(ref mut xosc) => {
            match xosc.sys_pll {
                None => {}
                Some(ref mut pll) => {
                    pll.post_div1 = 3;
                }
            }
        }
        None => {}
    }*/

    let p = embassy_rp::init(Default::default());

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(core0_main(spawner, p))));
}

const WIFI_NETWORK: &str = "KalogonProdTest";
const WIFI_PASSWORD: &str = "fwgnjEDjp7jd4pm9";

#[embassy_executor::task]
async fn core0_main(spawner: Spawner, p: embassy_rp::Peripherals) {


    let mut delay = Delay;
    let mut delay2 = Delay;


    Timer::after_millis(1000).await;

    let fw = include_bytes!("../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");

    let pwr = gpio::Output::new(p.PIN_23, gpio::Level::Low);
    let cs = gpio::Output::new(p.PIN_25, gpio::Level::High);
    let mut pio = pio::Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        FixedU32::from_bits(0x0300),
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = embassy_net::Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guaranteed to be random.

    // Init network stack
    static RESOURCES: StaticCell<StackResources<10>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::<10>::new()),
        seed,
    );

    unwrap!(spawner.spawn(net_task(runner)));

    loop {
        match control
            .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
            .await
        {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("DHCP is now up!");

    defmt::trace!("Hello World!");

    let mut scl1 = p.PIN_19;
    let mut sda1 = p.PIN_18;

    Timer::after_millis(100).await;

    defmt::trace!("Checkpoint 3");


    defmt::trace!("Checkpoint 8");
    Timer::after_millis(100).await;


    let tcp_state = TcpClientState::<1, 3000, 3000>::new();
    let tcp_client = TcpClient::new(stack, &tcp_state);

    loop {
        Timer::after_millis(1010).await;
    }
}