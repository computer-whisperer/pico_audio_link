#![no_std]
#![no_main]

#![feature(impl_trait_in_assoc_type)]

extern crate alloc;
mod microphone;
mod class_codes;
mod terminal_type;

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


use embassy_net::{IpAddress, IpEndpoint, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join3, join};
use embassy_futures::select::{select, select3, Either, Either3, Select};
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::udp::{PacketMetadata, RecvError, UdpMetadata, UdpSocket};
use embassy_rp::{gpio, pwm, spi, peripherals, i2c, pio, uart, pac, usb, dma, bind_interrupts};
use embassy_rp::gpio::{Output, Pin};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::signal::Signal;
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::{speaker, FeedbackRefresh, SampleWidth};
use embassy_usb::class::uac1::speaker::Speaker;
use embassy_usb::driver::EndpointError;
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


type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

const WIFI_NETWORK: &str = "PicoAudioLink0";
const WIFI_PASSWORD: &str = "W86i<u9vL=S|";

#[embassy_executor::task]
async fn core0_main(spawner: Spawner, p: embassy_rp::Peripherals) {

    // usb stack init
    let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Christian");
        config.product = Some("USB-example");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let (mut usb_audio_stream, usb_audio_control) = {
        static STATE: StaticCell<microphone::State> = StaticCell::new();
        let state = STATE.init(microphone::State::new());
        microphone::Microphone::new(
            &mut builder,
            state,
            48,
            SampleWidth::Width3Byte,
            &[48000],
            &[microphone::Channel::OnlyChannel],
        )
    };

    let (mut speaker_stream, b, c) = {
        static STATE: StaticCell<speaker::State> = StaticCell::new();
        let state = STATE.init(speaker::State::new());
        speaker::Speaker::new(
            &mut builder,
            state,
            48,
            SampleWidth::Width2Byte,
            &[48000],
            &[uac1::Channel::LeftFront],
            FeedbackRefresh::Period32Frames
        )
    };

    let usb = builder.build();

    unwrap!(spawner.spawn(usb_task(usb)));

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
        address: Ipv4Cidr::from_str("192.168.0.0/24").unwrap(),
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

       control
            .start_ap_wpa2(WIFI_NETWORK, WIFI_PASSWORD, 3)
            .await;

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for Network...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("Network is now up!");

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
    let our_endpoint = IpListenEndpoint{ addr: Some(addr_dongle), port:1000 };
    let remote_endpoint = UdpMetadata::from( IpEndpoint::new( addr_device, 1000 ));
    udp_socket.bind(our_endpoint).unwrap();

    const TRANSFER_PACKET_SIZE: usize = 768;
    const SAMPLE_LEN_BYTES: usize = 3;

    let mic_data_signal: Signal<ProjectMutex, ([u8; TRANSFER_PACKET_SIZE], usize, Instant)> = Signal::new();
    let speaker_data_signal: Signal<ProjectMutex, ([u8; TRANSFER_PACKET_SIZE], usize)> = Signal::new();

    let mic_async = async {
        const SAMPLES_PER_PACKET: u32 = 16;
        let mut latest_buffer = [0u8; TRANSFER_PACKET_SIZE];
        let sample_rate = 48000u64;
        let packet_interval_ns = (1000_000_000*SAMPLES_PER_PACKET as u64) / sample_rate;
        let sample_interval_ns = (1000_000_000) / sample_rate;
        let mut last_buffer_start = Instant::now();
        let mut good_samples = 0usize;

        let mut last_packet_instant = Instant::now();
        loop {
            let a = Timer::at(last_packet_instant + Duration::from_nanos(packet_interval_ns));
            let b = mic_data_signal.wait();
            match select(a, b).await {
                Either::First(_) => {
                    let current_time = Instant::now();
                    last_packet_instant = current_time;
                    let offset = (((current_time - last_buffer_start).as_micros()*1000)/sample_interval_ns) as usize;
                    let mut samples_raw = [0u8; SAMPLES_PER_PACKET as usize*3];
                    for i in 0..SAMPLES_PER_PACKET as usize {
                        if (i + offset) < good_samples {
                            /*for j in 0..SAMPLE_LEN_BYTES {
                                samples_raw[i*4 + j + 1] =
                                    latest_buffer[(i + offset)*SAMPLE_LEN_BYTES + j]
                            };*/

                            let sample = ((latest_buffer[(i + offset)*SAMPLE_LEN_BYTES + 0] as u32) << 16) |
                                         ((latest_buffer[(i + offset)*SAMPLE_LEN_BYTES + 1] as u32) << 8)  |
                                         ((latest_buffer[(i + offset)*SAMPLE_LEN_BYTES + 2] as u32) << 0);
                            let sample = sample | ((sample & 0x400000) <<1);
                            //let sample = sample << 8;
                            //defmt::info!("{:06X}", sample);
                            
                            //
                            samples_raw[i*3 + 0] = ((sample >> 0)&0xFF) as u8;
                            samples_raw[i*3 + 1] = ((sample >> 8)&0xFF) as u8;
                            samples_raw[i*3 + 2] = ((sample >> 16)&0xFF) as u8;
                            //samples_raw[i*4 + 3] = ((sample >> 24)&0xFF) as u8;
                            //defmt::info!("{:02X}, {:02X}, {:02X}", latest_buffer[(i+offset)*3 + 0], latest_buffer[(i+offset)*3 + 1], latest_buffer[(i+offset)*3 + 2]);
                        }
                        
                    }
                    usb_audio_stream.write_packet(&samples_raw).await.unwrap();
                },
                Either::Second((a, b, c)) => {
                    latest_buffer.copy_from_slice(&a);
                    good_samples = b;
                    last_buffer_start = c;
                }
            }
        }
    };

    let spkr_async = async {
        let mut sample_buffer = [0u8; TRANSFER_PACKET_SIZE];
        let mut sample_buffer_idx = 0;
        let mut data = [0u8; 48];
        loop {
            match speaker_stream.read_packet(&mut data).await {
                Ok(a) => {
                    let end = sample_buffer_idx + a;
                    if end > sample_buffer.len() {
                        speaker_data_signal.signal((
                            sample_buffer.clone(),
                            sample_buffer_idx
                            ));
                        sample_buffer_idx = 0;
                    }
                    sample_buffer[sample_buffer_idx..sample_buffer_idx+a].copy_from_slice(&data[sample_buffer_idx..sample_buffer_idx+a]);
                }
                Err(err) => {
                   defmt::error!("usb speaker read error: {:?}", err) 
                }
            }
        }
    };

    let udp_async = async {

        let mut udp_recv_buf = [0u8; TRANSFER_PACKET_SIZE];
        loop {
            let a = speaker_data_signal.wait();
            let b = udp_socket.recv_from(&mut udp_recv_buf);
            match select(a, b).await {
                Either::First((data, good_samples)) => {
                    udp_socket.send_to(&data[0..good_samples], remote_endpoint).await.unwrap()
                }
                Either::Second(x) => {
                    match x {
                        Ok((num_bytes, _meta)) => {
                            mic_data_signal.signal((
                                udp_recv_buf.clone(),
                                num_bytes/3,
                                Instant::now()
                                ));
                        }
                        Err(x) => {
                            defmt::error!("UDP recv error: {:?}", x);
                        }
                    }
                }
            }
        }
    };

    join(mic_async, udp_async).await;
}