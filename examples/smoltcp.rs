//! ENC28J60 demo: pong server + UDP echo server
//!
//! This program:
//!
//! - Responds to ARP requests
//! - Responds to ICMP echo requests, thus you can `ping` the device
//! - Responds to *all* UDP datagrams by sending them back
//!
//! You can test this program by running the following commands:
//!
//! - `ping 192.168.1.33`. The device should respond and toggle the state of the LED on every `ping`
//! request.
//! - `nc -u 192.168.1.33 1337` and sending any string. The device should respond back by sending
//! back the received string; the LED will toggle each time a UDP datagram is sent.
//!
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate embedded_hal;
extern crate enc28j60;
extern crate panic_semihosting;
extern crate smoltcp;
extern crate stm32f1xx_hal as hal;

use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use enc28j60::Enc28j60;
use hal::delay::Delay;
use hal::device;
use hal::prelude::*;
use hal::serial::Serial;
use hal::spi::Spi;
use rt::entry;

use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::phy::{self, Device, DeviceCapabilities};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};
use smoltcp::Result;

/* Constants */
const KB: u16 = 1024; // bytes
const SRC_MAC: [u8; 6] = [0x20, 0x18, 0x03, 0x01, 0x00, 0x00];

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    cp.DWT.enable_cycle_counter();

    // LED
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // turn the LED off during initialization
    led.set_high();

    // Serial
    let mut serial = {
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let rx = gpiob.pb7;
        let serial = Serial::usart1(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            115_200.bps(),
            clocks,
            &mut rcc.apb2,
        );

        serial.split().0
    };
    writeln!(serial, "serial start").unwrap();

    // SPI
    let mut ncs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    ncs.set_high();
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        enc28j60::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );
    writeln!(serial, "spi initialized").unwrap();

    // ENC28J60
    let mut reset = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    reset.set_high();
    let mut delay = Delay::new(cp.SYST, clocks);
    let enc28j60 = Enc28j60::new(
        spi,
        ncs,
        enc28j60::Unconnected,
        reset,
        &mut delay,
        7 * KB,
        SRC_MAC,
    )
    .ok()
    .unwrap();
    writeln!(serial, "enc26j60 initialized").unwrap();

    let mut eth = EncPhy::new(enc28j60);
    writeln!(serial, "eth initialized").unwrap();

    let local_addr = Ipv4Address::new(192, 168, 1, 2);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let ethernet_addr = EthernetAddress(SRC_MAC);
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();
    writeln!(serial, "iface initialized").unwrap();

    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);
    writeln!(serial, "sockets initialized").unwrap();

    // LED on after initialization
    led.set_low();

    // FIXME some frames are lost when sent right after initialization
    delay.delay_ms(100_u8);

    loop {
        match iface.poll(&mut sockets, Instant::from_millis(0)) {
            Ok(b) => {
                if b {
                    let mut socket = sockets.get::<TcpSocket>(server_handle);
                    if !socket.is_open() {
                        socket.listen(80).unwrap();
                    }

                    if socket.can_send() {
                        writeln!(serial, "tcp:80 send greeting").unwrap();
                        write!(socket, "HTTP/1.1 200 OK\r\n\r\ntest string\n").unwrap();
                        writeln!(serial, "tcp:80 close").unwrap();
                        socket.close();
                    }
                }
            }
            Err(e) => {
                writeln!(serial, "Error: {:?}", e).unwrap();
            }
        }
    }
}

struct EncPhy {
    inner: Enc28j60<
        hal::spi::Spi<
            hal::pac::SPI1,
            (
                hal::gpio::gpioa::PA5<hal::gpio::Alternate<hal::gpio::PushPull>>,
                hal::gpio::gpioa::PA6<hal::gpio::Input<hal::gpio::Floating>>,
                hal::gpio::gpioa::PA7<hal::gpio::Alternate<hal::gpio::PushPull>>,
            ),
        >,
        hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>,
        enc28j60::Unconnected,
        hal::gpio::gpioa::PA3<hal::gpio::Output<hal::gpio::PushPull>>,
    >,
    rxbuf: [u8; 1024],
}

impl EncPhy {
    pub fn new(
        inner: Enc28j60<
            hal::spi::Spi<
                hal::pac::SPI1,
                (
                    hal::gpio::gpioa::PA5<hal::gpio::Alternate<hal::gpio::PushPull>>,
                    hal::gpio::gpioa::PA6<hal::gpio::Input<hal::gpio::Floating>>,
                    hal::gpio::gpioa::PA7<hal::gpio::Alternate<hal::gpio::PushPull>>,
                ),
            >,
            hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>,
            enc28j60::Unconnected,
            hal::gpio::gpioa::PA3<hal::gpio::Output<hal::gpio::PushPull>>,
        >,
    ) -> Self {
        EncPhy {
            inner,
            rxbuf: [0u8; 1024],
        }
    }
}

impl<'a, 'b> Device<'a> for &'b mut EncPhy {
    type RxToken = PhyRxToken<'a>;
    type TxToken = PhyTxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let packet = self.inner.next_packet().unwrap();
        match packet {
            Some(packet) => {
                packet.read(&mut self.rxbuf[..]).unwrap();
                Some((
                    PhyRxToken(&mut self.rxbuf[..]),
                    PhyTxToken {
                        enc28j60: &mut self.inner,
                        txbuf: [0u8; 1024],
                    },
                ))
            }
            None => None,
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(PhyTxToken {
            enc28j60: &mut self.inner,
            txbuf: [0u8; 1024],
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps
    }
}

struct PhyRxToken<'a>(&'a [u8]);

impl<'a> phy::RxToken for PhyRxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&[u8]) -> Result<R>,
    {
        let result = f(self.0);
        result
    }
}

struct PhyTxToken<'a> {
    enc28j60: &'a mut enc28j60::Enc28j60<
        hal::spi::Spi<
            hal::pac::SPI1,
            (
                hal::gpio::gpioa::PA5<hal::gpio::Alternate<hal::gpio::PushPull>>,
                hal::gpio::gpioa::PA6<hal::gpio::Input<hal::gpio::Floating>>,
                hal::gpio::gpioa::PA7<hal::gpio::Alternate<hal::gpio::PushPull>>,
            ),
        >,
        hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>,
        enc28j60::Unconnected,
        hal::gpio::gpioa::PA3<hal::gpio::Output<hal::gpio::PushPull>>,
    >,
    txbuf: [u8; 1024],
}

impl<'a> phy::TxToken for PhyTxToken<'a> {
    fn consume<R, F>(mut self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        let result = f(&mut self.txbuf[..len]);
        self.enc28j60.transmit(&self.txbuf[..len]).unwrap();
        result
    }
}
