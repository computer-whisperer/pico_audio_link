#![no_std]
#![no_main]

#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use core::cmp::PartialEq;
use core::future::Future;
use core::str::FromStr;
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


use embassy_net::{IpEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4, IpAddress, IpListenEndpoint};
use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::udp::{PacketMetadata, UdpMetadata, UdpSocket};
use embassy_rp::{gpio, pwm, spi, peripherals, i2c, pio, uart, pac, usb, dma, bind_interrupts};
use embassy_rp::gpio::{Output, Pin};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::{speaker, FeedbackRefresh, SampleWidth};
use embassy_usb::class::uac1::speaker::Speaker;
use embassy_usb::UsbDevice;
use fixed::FixedU32;
use heapless::Vec;
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
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
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



const WIFI_NETWORK: &str = "PicoAudioLink0";
const WIFI_PASSWORD: &str = "W86i<u9vL=S|";

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
    let addr_device = IpAddress::v4(192, 168, 0, 1);
    let addr_dongle = IpAddress::v4(192, 168, 0, 0);

    let config = embassy_net::Config::ipv4_static(StaticConfigV4{
        address: Ipv4Cidr::from_str("192.168.0.1/24").unwrap(),
        gateway: None,
        dns_servers: Vec::new()
    });


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


    let mut rx_meta = [PacketMetadata::EMPTY; 24];
    let mut rx_buff = [0u8; 2048];
    let mut tx_meta = [PacketMetadata::EMPTY; 24];
    let mut tx_buff = [0u8; 2048];
    let mut udp_socket = UdpSocket::new(stack,
                                    &mut rx_meta,
                                    &mut rx_buff,
                                    &mut tx_meta,
                                    &mut tx_buff
    );
    let our_endpoint = IpListenEndpoint{ addr: Some(addr_device.clone()), port:1000 };
    udp_socket.bind(our_endpoint).unwrap();


    let remote_endpoint = UdpMetadata::from( IpEndpoint::new( addr_dongle, 1000 ));
    const SAMPLES_PER_PACKET: u32 = 500;
    let sample_rate = 48000u64;
    let packet_interval_ns = (1000_000_000*SAMPLES_PER_PACKET as u64) / sample_rate;
    let sample_interval_ns = (1000_000_000) / sample_rate;
    let mut last_packet_instant = None;
    loop {
        if let Some(last_packet_instant) = last_packet_instant {
            Timer::at(last_packet_instant + Duration::from_nanos(packet_interval_ns)).await;
        }
        let current_time = Instant::now();
        last_packet_instant = Some(current_time);
        let current_timestamp_ns = current_time.as_micros()*1000;
        let mut samples_raw = [0u8; SAMPLES_PER_PACKET as usize*2];
        for i in 0..SAMPLES_PER_PACKET as usize {
            let current_timestamp_ns = current_timestamp_ns + (i as u64*sample_interval_ns);
            let sample = if current_timestamp_ns%500_000 < 200_000 {20000u16} else {0u16};
            samples_raw[i*2] = ((sample)&0xFF) as u8;
            samples_raw[i*2 + 1] = ((sample>>8)&0xFF) as u8;
        }
        udp_socket.send_to(&samples_raw[..], remote_endpoint.clone()).await.unwrap();
    }
}