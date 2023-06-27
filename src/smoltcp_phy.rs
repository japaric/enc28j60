//! ENC28J60 wrapper for use as a smoltcp interface

use crate::Enc28j60;
use embedded_hal::{blocking, digital::v2::OutputPin};
use smoltcp::{
    phy::{self, Device, DeviceCapabilities},
    time::Instant,
};

/// Wrapper for use as a `smoltcp` interface for sending and receiving raw network frames.
pub struct Phy<'a, SPI, NCS, INT, RESET> {
    phy: Enc28j60<SPI, NCS, INT, RESET>,
    rx_buf: &'a mut [u8],
    tx_buf: &'a mut [u8],
}

impl<'a, SPI, NCS, INT, RESET> Phy<'a, SPI, NCS, INT, RESET> {
    /// Create a new ethernet interface from an Enc28j60, a receive buffer and a transmit buffer.
    pub fn new(
        phy: Enc28j60<SPI, NCS, INT, RESET>,
        rx_buf: &'a mut [u8],
        tx_buf: &'a mut [u8],
    ) -> Self {
        Phy {
            phy,
            rx_buf,
            tx_buf,
        }
    }
}

impl<'a, E, SPI: 'a, NCS: 'a, INT, RESET> Device for Phy<'a, SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
    INT: crate::sealed::IntPin,
    RESET: crate::sealed::ResetPin,
{
    type RxToken<'token> = RxToken<'token> where Self: 'token;
    type TxToken<'token> = TxToken<'token, SPI, NCS, INT, RESET> where Self: 'token;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let packet = self.phy.next_packet();
        match packet {
            Ok(Some(packet)) => {
                packet.read(&mut self.rx_buf[..]).ok().unwrap();
                Some((
                    RxToken(&mut self.rx_buf[..]),
                    TxToken {
                        phy: &mut self.phy,
                        buf: &mut self.tx_buf,
                    },
                ))
            }
            _ => None,
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(TxToken {
            phy: &mut self.phy,
            buf: &mut self.tx_buf,
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps
    }
}

/// A token to receive a single network packet
pub struct RxToken<'a>(&'a mut [u8]);

impl<'a> phy::RxToken for RxToken<'a> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.0);
        result
    }
}

/// A token to transmit a single network packet.
pub struct TxToken<'a, SPI, NCS, INT, RESET> {
    phy: &'a mut Enc28j60<SPI, NCS, INT, RESET>,
    buf: &'a mut [u8],
}

impl<'a, E, SPI, NCS, INT, RESET> phy::TxToken for TxToken<'a, SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
    INT: crate::sealed::IntPin,
    RESET: crate::sealed::ResetPin,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.buf[..len]);
        self.phy.transmit(&self.buf[..len]).ok().unwrap();
        result
    }
}
