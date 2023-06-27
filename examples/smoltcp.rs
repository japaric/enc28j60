//! ENC28J60 + smoltcp demo
//!
//! Demonstrates how to use an ENC28J60 with smoltcp by running a simple demo that
//! toggles and returns the current LED state.
//!
//! You can test this program with the following:
//!
//! - `ping 192.168.1.2`. The device will respond to every request (response time should be ~10ms).
//! - `curl 192.168.1.2`. The device will respond with a HTTP response with the current
//! LED state in the body.
//! - Visiting `https://192.168.1.2/`. Every refresh will toggle the LED and the page will
//! reflect the current state.
//!
#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;
use cortex_m_rt::entry;
use enc28j60::{smoltcp_phy::Phy, Enc28j60};
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address},
};
use stm32f1xx_hal::{
    device,
    prelude::*,
    serial::{self, Serial},
    spi::Spi,
    timer::Timer,
};

const SRC_MAC: [u8; 6] = [0x20, 0x18, 0x03, 0x01, 0x00, 0x00];

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    cp.DWT.enable_cycle_counter();

    // LED
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // turn the LED off during initialization
    let _ = led.set_high();

    // Serial
    let mut serial = {
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let rx = gpiob.pb7;
        let serial = Serial::new(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            serial::Config::default().baudrate(115_200.bps()),
            &clocks,
        );

        serial.split().0
    };
    writeln!(serial, "serial start").unwrap();

    // SPI
    let spi = {
        let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

        Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            &mut afio.mapr,
            enc28j60::MODE,
            1.MHz(),
            clocks,
        )
    };
    writeln!(serial, "spi initialized").unwrap();

    // ENC28J60
    let enc28j60 = {
        let mut ncs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let _ = ncs.set_high();
        let mut reset = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let _ = reset.set_high();
        let mut delay = Timer::delay(Timer::syst(cp.SYST, &clocks));

        Enc28j60::new(
            spi,
            ncs,
            enc28j60::Unconnected,
            reset,
            &mut delay,
            7168,
            SRC_MAC,
        )
        .ok()
        .unwrap()
    };
    writeln!(serial, "enc26j60 initialized").unwrap();

    // PHY Wrapper
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut eth = Phy::new(enc28j60, &mut rx_buf, &mut tx_buf);
    writeln!(serial, "eth initialized").unwrap();

    // Ethernet interface
    let local_addr = Ipv4Address::new(192, 168, 1, 2);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    // let mut neighbor_storage = [None; 16];
    // let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let config = Config::new(HardwareAddress::Ethernet(EthernetAddress(SRC_MAC)));
    let mut iface = Interface::new(config, &mut eth, Instant::from_micros(0));
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs.push(ip_addr).unwrap();
    });
    // .ethernet_addr(ethernet_addr)
    // .ip_addrs(&mut ip_addrs[..])
    // .neighbor_cache(neighbor_cache)
    // .finalize();
    writeln!(serial, "iface initialized").unwrap();

    // Sockets
    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let mut sockets_storage: [_; 2] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);
    writeln!(serial, "sockets initialized").unwrap();

    // LED on after initialization
    let _ = led.set_low();

    let mut count: u64 = 0;
    loop {
        if iface.poll(Instant::from_millis(0), &mut eth, &mut sockets) {
            let socket = sockets.get_mut::<TcpSocket>(server_handle);
            if !socket.is_open() {
                socket.listen(80).unwrap();
            }

            if socket.can_send() {
                let _ = led.toggle();

                writeln!(serial, "tcp:80 send").unwrap();
                write!(
                            socket,
                            "HTTP/1.1 200 OK\r\n\r\nHello!\nLED is currently {} and has been toggled {} times.\n",
                            match led.is_set_low() {
                                true => "on",
                                false => "off",
                            },
                            count
                        )
                        .unwrap();

                writeln!(serial, "tcp:80 close").unwrap();
                socket.close();

                count += 1;
            }
        }
    }
}
