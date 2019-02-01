//! A platform agnostic driver to interface with the ENC28J60 (Ethernet controller)
//!
//! This driver is built on top of the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//!
//! # Examples
//!
//! You should find some examples in the [`stm32f103xx-hal`] crate.
//!
//! [`stm32f103xx-hal`]: https://github.com/japaric/stm32f103xx-hal/tree/master/examples
//!
//! # References
//!
//! - [ENC28J60 Data Sheet (DS39662E)][0]
//! - [ENC28J60 Rev. B7 Silicon Errata (DS80349C)][1]
//!
//! [0]: http://ww1.microchip.com/downloads/en/DeviceDoc/39662e.pdf
//! [1]: http://ww1.microchip.com/downloads/en/DeviceDoc/80349c.pdf

#![deny(missing_docs)]
#![deny(rust_2018_compatibility)]
#![deny(rust_2018_idioms)]
#![deny(warnings)]
#![no_std]

use core::u16;

use as_slice::AsMutSlice;
use byteorder::{ByteOrder, LE};
use cast::usize;
use embedded_hal::{
    blocking::{self, delay::DelayMs},
    digital::{InputPin, OutputPin},
    spi::{Mode, Phase, Polarity},
};
use owning_slice::IntoSliceTo;

use crate::traits::U16Ext;

#[macro_use]
mod macros;
mod bank0;
mod bank1;
mod bank2;
mod bank3;
mod common;
mod phy;
mod sealed;
mod traits;

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

// Total buffer size (see section 3.2)
const BUF_SZ: u16 = 8 * 1024;

// Maximum frame length
const MAX_FRAME_LENGTH: u16 = 1518; // value recommended in the data sheet

// Size of the Frame check sequence (32-bit CRC)
const CRC_SZ: u16 = 4;

/// Error
///
/// *NOTE:* this enum may gain more variants in the future
#[derive(Debug)]
pub enum Error<E> {
    /// Late collision
    LateCollision,
    /// EREVID read as zero; this means that the device is not an ENC28J60 or that it has locked up
    ErevidIsZero,
    /// Transmission was aborted
    TransmitAbort,
    /// SPI error
    Spi(E),
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    _DO_NOT_MATCH_AGAINST_THIS_VARIANT,
}

/// Events that the ENC28J60 can notify about via the INT pin
pub enum Event {
    /// There are packets pending to be processed in the RX buffer
    Pkt,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Spi(e)
    }
}

/// ENC28J60 driver
pub struct Enc28j60<SPI, NCS, INT, RESET> {
    int: INT,
    ncs: NCS,
    reset: RESET,
    spi: SPI,

    // A packet is pending to be read
    pending: bool,

    bank: Bank,

    // address of the next packet in buffer memory
    next_packet: u16,

    /// End of the RX buffer / Start of the TX buffer
    rxnd: u16,
}

const NONE: u16 = u16::MAX;
// See Errata #5
const RXST: u16 = 0;

impl<E, SPI, NCS, INT, RESET> Enc28j60<SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
    INT: crate::sealed::IntPin,
    RESET: crate::sealed::ResetPin,
{
    /* Constructors */
    /// Creates a new driver from a SPI peripheral, a NCS pin, a RESET pin and an INT
    /// (interrupt) pin
    ///
    /// If you haven't physically connected the RESET and / or the INT pin(s) pass the `Unconnected`
    /// value as the `reset` and / or `int` argument(s), respectively.
    ///
    /// `rx_buf_sz` is the size of the ENC28J60 RX buffer in bytes. Note that if `rx_buf_sz` is odd
    /// it will be rounded to an even number.
    ///
    /// `src` is the MAC address to associate to this interface. Note that this MAC address is
    /// only used to decide which frames will be ignored; frames that don't have their destination
    /// address set to broadcast (`ff:ff:ff:ff:ff:ff`) or to `src` will be ignored by the
    /// interface.
    ///
    /// # Panics
    ///
    /// If `rx_buf_sz` is greater than `8192` (8 KibiBytes); that's the size of the ENC28J60
    /// internal memory.
    pub fn new<D>(
        spi: SPI,
        ncs: NCS,
        int: INT,
        reset: RESET,
        delay: &mut D,
        mut rx_buf_sz: u16,
        src: [u8; 6],
    ) -> Result<Self, Error<E>>
    where
        D: DelayMs<u8>,
    {
        // round up `rx_buf_sz` to an even number
        if rx_buf_sz % 2 == 1 {
            rx_buf_sz += 1;
        }

        assert!(rx_buf_sz <= BUF_SZ && rx_buf_sz != 0);

        let mut enc28j60 = Enc28j60 {
            bank: Bank::Bank0,
            int,
            ncs,
            next_packet: RXST,
            pending: false,
            reset,
            rxnd: NONE,
            spi,
        };

        // (software) reset to return to a clean slate state
        if typeid!(RESET == Unconnected) {
            enc28j60.soft_reset()?;

            // work around errata #2
            delay.delay_ms(1);
        } else {
            enc28j60.reset.reset();

            while common::ESTAT(enc28j60.read_control_register(common::Register::ESTAT)?).clkrdy()
                == 0
            {}
        }

        if enc28j60.read_control_register(bank3::Register::EREVID)? == 0 {
            return Err(Error::ErevidIsZero);
        }

        // disable CLKOUT output
        enc28j60.write_control_register(bank3::Register::ECOCON, 0)?;

        // define the boundaries of the TX and RX buffers
        // to workaround errata #5 we do the opposite of what section 6.1 of the data sheet
        // says: we place the RX buffer at address 0 and the TX buffer after it
        let rxnd = rx_buf_sz - 1;
        enc28j60.rxnd = rxnd;

        // RX start
        // "It is recommended that the ERXST Pointer be programmed with an even address"
        enc28j60.write_control_register(bank0::Register::ERXSTL, RXST.low())?;
        enc28j60.write_control_register(bank0::Register::ERXSTH, RXST.high())?;

        // RX read pointer
        // NOTE Errata #14 so we are using an *odd* address here instead of ERXST
        enc28j60.write_control_register(bank0::Register::ERXRDPTL, rxnd.low())?;
        enc28j60.write_control_register(bank0::Register::ERXRDPTH, rxnd.high())?;

        // RX end
        enc28j60.write_control_register(bank0::Register::ERXNDL, rxnd.low())?;
        enc28j60.write_control_register(bank0::Register::ERXNDH, rxnd.high())?;

        // TX start
        // "It is recommended that an even address be used for ETXST"
        let txst = enc28j60.txst();
        debug_assert_eq!(txst % 2, 0);
        enc28j60.write_control_register(bank0::Register::ETXSTL, txst.low())?;
        enc28j60.write_control_register(bank0::Register::ETXSTH, txst.high())?;

        // TX end is set in `transmit`

        // MAC initialization (see section 6.5)
        // 1. Set the MARXEN bit in MACON1 to enable the MAC to receive frames.
        enc28j60.write_control_register(
            bank2::Register::MACON1,
            bank2::MACON1::default()
                .marxen(1)
                .passall(0)
                .rxpaus(1)
                .txpaus(1)
                .bits(),
        )?;

        // 2. Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3.
        enc28j60.write_control_register(
            bank2::Register::MACON3,
            bank2::MACON3::default()
                .frmlnen(1)
                .txcrcen(1)
                .padcfg(0b001)
                .bits(),
        )?;

        // 4. Program the MAMXFL registers with the maximum frame length to be permitted to be
        // received or transmitted
        enc28j60.write_control_register(bank2::Register::MAMXFLL, MAX_FRAME_LENGTH.low())?;
        enc28j60.write_control_register(bank2::Register::MAMXFLH, MAX_FRAME_LENGTH.high())?;

        // 5. Configure the Back-to-Back Inter-Packet Gap register, MABBIPG.
        // Use recommended value of 0x12
        enc28j60.write_control_register(bank2::Register::MABBIPG, 0x12)?;

        // 6. Configure the Non-Back-to-Back Inter-Packet Gap register low byte, MAIPGL.
        // Use recommended value of 0x12
        enc28j60.write_control_register(bank2::Register::MAIPGL, 0x12)?;

        // 9. Program the local MAC address into the MAADR1:MAADR6 registers
        enc28j60.write_control_register(bank3::Register::MAADR1, src[0])?;
        enc28j60.write_control_register(bank3::Register::MAADR2, src[1])?;
        enc28j60.write_control_register(bank3::Register::MAADR3, src[2])?;
        enc28j60.write_control_register(bank3::Register::MAADR4, src[3])?;
        enc28j60.write_control_register(bank3::Register::MAADR5, src[4])?;
        enc28j60.write_control_register(bank3::Register::MAADR6, src[5])?;

        // Set the PHCON2.HDLDIS bit to prevent automatic loopback of the data which is transmitted
        enc28j60.write_phy_register(
            phy::Register::PHCON2,
            phy::PHCON2::default().hdldis(1).bits(),
        )?;

        // Globally enable interrupts
        if typeid!(INT != Unconnected) {
            enc28j60.bit_field_set(common::Register::EIE, common::EIE::mask().intie())?;
        }

        // Set the per packet control byte; we'll always use the value 0
        enc28j60.write_buffer_memory(Some(txst), &[0])?;

        // Enable reception
        enc28j60.bit_field_set(common::Register::ECON1, common::ECON1::mask().rxen())?;

        Ok(enc28j60)
    }
}

impl<E, SPI, NCS, INT, RESET> Enc28j60<SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
{
    /// Flushes the transmit buffer, ensuring all pending transmissions have completed
    /// NOTE: The returned packet *must* be `read` or `ignore`-d, otherwise this method will always
    /// return `None` on subsequent invocations
    pub fn next_packet(&mut self) -> Result<Option<NextPacket<'_, SPI, NCS, INT, RESET>>, E> {
        if self.pending {
            Ok(None)
        } else if self.pending_packets()? == 0 {
            // Errata #6: we can't rely on PKTIF so we check PKTCNT
            Ok(None)
        } else {
            self.pending = true;

            let curr_packet = self.next_packet;

            // read out the first 6 bytes
            let mut temp_buf = [0; 6];
            self.read_buffer_memory(Some(curr_packet), &mut temp_buf)?;

            // next packet pointer
            let next_packet = u16::from_parts(temp_buf[0], temp_buf[1]);
            debug_assert!(next_packet <= self.rxnd);

            // status vector
            let status = RxStatus(LE::read_u32(&temp_buf[2..]));
            let len = status.byte_count() as u16 - CRC_SZ;

            Ok(Some(NextPacket {
                enc28j60: self,
                next_packet,
                len,
            }))
        }
    }

    /// Starts the transmission of `bytes`
    ///
    /// It's up to the caller to ensure that `bytes` is a valid Ethernet frame. The interface will
    /// take care of appending a (4 byte) CRC to the frame and of padding the frame to the minimum
    /// size allowed by the Ethernet specification (64 bytes, or 46 bytes of payload).
    ///
    /// NOTE This method will flush any previous transmission that's in progress
    ///
    /// # Panics
    ///
    /// If `bytes` length is greater than 1514, the maximum frame length allowed by the interface,
    /// or greater than the transmit buffer
    pub fn transmit(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        assert!(
            bytes.len() <= usize(MAX_FRAME_LENGTH - CRC_SZ)
                && bytes.len() <= usize(BUF_SZ - self.rxnd - 1)
        );

        let txst = self.txst();

        // work around errata #12 by resetting the transmit logic before every new
        // transmission
        self.bit_field_set(common::Register::ECON1, common::ECON1::mask().txrst())?;
        self.bit_field_clear(common::Register::ECON1, common::ECON1::mask().txrst())?;
        self.bit_field_clear(common::Register::EIR, {
            let mask = common::EIR::mask();
            mask.txerif() | mask.txif()
        })?;

        // NOTE the plus one is to not overwrite the per packet control byte
        let wrpt = txst + 1;

        // 1. ETXST was set during initialization

        // 2. write the frame to the IC memory
        self.write_buffer_memory(Some(wrpt), bytes)?;

        let txnd = wrpt + bytes.len() as u16 - 1;

        // 3. Set the end address of the transmit buffer
        self.write_control_register(bank0::Register::ETXNDL, txnd.low())?;
        self.write_control_register(bank0::Register::ETXNDH, txnd.high())?;

        // 4. reset interrupt flag
        self.bit_field_clear(common::Register::EIR, { common::EIR::mask().txif() })?;

        // 5. start transmission
        self.bit_field_set(common::Register::ECON1, common::ECON1::mask().txrts())?;

        // Wait until transmission finishes
        while common::ECON1(self.read_control_register(common::Register::ECON1)?).txrts() == 1 {}

        // read the transmit status vector
        let mut tx_stat = [0; 7];
        self.read_buffer_memory(None, &mut tx_stat)?;

        let stat = common::ESTAT(self.read_control_register(common::Register::ESTAT)?);

        if stat.txabrt() == 1 {
            // work around errata #12 by reading the transmit status vector
            if stat.latecol() == 1 || (tx_stat[2] & (1 << 5)) != 0 {
                Err(Error::LateCollision)
            } else {
                Err(Error::TransmitAbort)
            }
        } else {
            Ok(())
        }
    }

    /* Miscellaneous */
    /// Returns the number of packets that have been received but have not been processed yet
    pub fn pending_packets(&mut self) -> Result<u8, E> {
        self.read_control_register(bank1::Register::EPKTCNT)
    }

    /// Adjusts the receive filter to *accept* these packet types
    pub fn accept(&mut self, packets: &[Packet]) -> Result<(), E> {
        let mask = bank1::ERXFCON::mask();
        let mut val = 0;
        for packet in packets {
            match packet {
                Packet::Broadcast => val |= mask.bcen(),
                Packet::Multicast => val |= mask.mcen(),
                Packet::Unicast => val |= mask.ucen(),
                #[cfg(debug_assertions)]
                _ => unreachable!(),
                #[cfg(not(debug_assertions))]
                _ => {},
            }
        }

        self.bit_field_set(bank1::Register::ERXFCON, val)
    }

    /// Adjusts the receive filter to *ignore* these packet types
    pub fn ignore(&mut self, packets: &[Packet]) -> Result<(), E> {
        let mask = bank1::ERXFCON::mask();
        let mut val = 0;
        for packet in packets {
            match packet {
                Packet::Broadcast => val |= mask.bcen(),
                Packet::Multicast => val |= mask.mcen(),
                Packet::Unicast => val |= mask.ucen(),
                #[cfg(debug_assertions)]
                _ => unreachable!(),
                #[cfg(not(debug_assertions))]
                _ => {},
            }
        }

        self.bit_field_clear(bank1::Register::ERXFCON, val)
    }

    /* Private */
    /* Read */
    fn read_control_register<R>(&mut self, register: R) -> Result<u8, E>
    where
        R: Into<Register>,
    {
        self._read_control_register(register.into())
    }

    fn _read_control_register(&mut self, register: Register) -> Result<u8, E> {
        self.change_bank(register)?;

        self.ncs.set_low();
        let mut buffer = [Instruction::RCR.opcode() | register.addr(), 0];
        self.spi.transfer(&mut buffer)?;
        self.ncs.set_high();

        Ok(buffer[1])
    }

    #[allow(dead_code)]
    fn read_phy_register(&mut self, register: phy::Register) -> Result<u16, E> {
        // set PHY register address
        self.write_control_register(bank2::Register::MIREGADR, register.addr())?;

        // start read operation
        self.bit_field_set(bank2::Register::MICMD, bank2::MICMD::mask().miird())?;

        // wait until the read operation finishes
        while self.read_control_register(bank3::Register::MISTAT)? & 0b1 != 0 {}

        self.bit_field_clear(bank2::Register::MICMD, bank2::MICMD::mask().miird())?;

        Ok(
            ((self.read_control_register(bank2::Register::MIRDH)? as u16) << 8)
                | (self.read_control_register(bank2::Register::MIRDL)? as u16),
        )
    }

    /* Write */
    fn _write_control_register(&mut self, register: Register, value: u8) -> Result<(), E> {
        self.change_bank(register)?;

        self.ncs.set_low();
        let buffer = [Instruction::WCR.opcode() | register.addr(), value];
        self.spi.write(&buffer)?;
        self.ncs.set_high();

        Ok(())
    }

    fn write_control_register<R>(&mut self, register: R, value: u8) -> Result<(), E>
    where
        R: Into<Register>,
    {
        self._write_control_register(register.into(), value)
    }

    fn write_phy_register(&mut self, register: phy::Register, value: u16) -> Result<(), E> {
        // set PHY register address
        self.write_control_register(bank2::Register::MIREGADR, register.addr())?;

        self.write_control_register(bank2::Register::MIWRL, (value & 0xff) as u8)?;
        // this starts the write operation
        self.write_control_register(bank2::Register::MIWRH, (value >> 8) as u8)?;

        // wait until the write operation finishes
        while self.read_control_register(bank3::Register::MISTAT)? & 0b1 != 0 {}

        Ok(())
    }

    /* RMW */
    fn modify_control_register<R, F>(&mut self, register: R, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
        R: Into<Register>,
    {
        self._modify_control_register(register.into(), f)
    }

    fn _modify_control_register<F>(&mut self, register: Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self._read_control_register(register)?;
        self._write_control_register(register, f(r))
    }

    /* Auxiliary */
    fn change_bank(&mut self, register: Register) -> Result<(), E> {
        let bank = register.bank();

        if let Some(bank) = bank {
            if self.bank == bank {
                // already on the register bank
                return Ok(());
            }

            // change bank
            self.bank = bank;
            match bank {
                Bank::Bank0 => self.bit_field_clear(common::Register::ECON1, 0b11),
                Bank::Bank1 => {
                    self.modify_control_register(common::Register::ECON1, |r| (r & !0b11) | 0b01)
                }
                Bank::Bank2 => {
                    self.modify_control_register(common::Register::ECON1, |r| (r & !0b11) | 0b10)
                }
                Bank::Bank3 => self.bit_field_set(common::Register::ECON1, 0b11),
            }
        } else {
            // common register
            Ok(())
        }
    }

    /* Primitive operations */
    fn bit_field_clear<R>(&mut self, register: R, mask: u8) -> Result<(), E>
    where
        R: Into<Register>,
    {
        self._bit_field_clear(register.into(), mask)
    }

    fn _bit_field_clear(&mut self, register: Register, mask: u8) -> Result<(), E> {
        debug_assert!(register.is_eth_register());

        self.change_bank(register)?;

        self.ncs.set_low();
        self.spi
            .write(&[Instruction::BFC.opcode() | register.addr(), mask])?;
        self.ncs.set_high();

        Ok(())
    }

    fn bit_field_set<R>(&mut self, register: R, mask: u8) -> Result<(), E>
    where
        R: Into<Register>,
    {
        self._bit_field_set(register.into(), mask)
    }

    fn _bit_field_set(&mut self, register: Register, mask: u8) -> Result<(), E> {
        debug_assert!(register.is_eth_register());

        self.change_bank(register)?;

        self.ncs.set_low();
        self.spi
            .write(&[Instruction::BFS.opcode() | register.addr(), mask])?;
        self.ncs.set_high();

        Ok(())
    }

    fn read_buffer_memory(&mut self, addr: Option<u16>, buf: &mut [u8]) -> Result<(), E> {
        if let Some(addr) = addr {
            self.write_control_register(bank0::Register::ERDPTL, addr.low())?;
            self.write_control_register(bank0::Register::ERDPTH, addr.high())?;
        }

        self.ncs.set_low();
        self.spi.write(&[Instruction::RBM.opcode()])?;
        self.spi.transfer(buf)?;
        self.ncs.set_high();

        Ok(())
    }

    fn soft_reset(&mut self) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.transfer(&mut [Instruction::SRC.opcode()])?;
        self.ncs.set_high();

        Ok(())
    }

    fn write_buffer_memory(&mut self, addr: Option<u16>, buffer: &[u8]) -> Result<(), E> {
        if let Some(addr) = addr {
            self.write_control_register(bank0::Register::EWRPTL, addr.low())?;
            self.write_control_register(bank0::Register::EWRPTH, addr.high())?;
        }

        self.ncs.set_low();
        self.spi.write(&[Instruction::WBM.opcode()])?;
        self.spi.write(buffer)?;
        self.ncs.set_high();
        Ok(())
    }
}

impl<SPI, NCS, INT, RESET> Enc28j60<SPI, NCS, INT, RESET> {
    /// Destroys the driver and returns all the hardware resources that were owned by it
    pub fn free(self) -> (SPI, NCS, INT, RESET) {
        (self.spi, self.ncs, self.int, self.reset)
    }

    fn txst(&self) -> u16 {
        self.rxnd + 1
    }
}

impl<E, SPI, NCS, INT, RESET> Enc28j60<SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
    INT: crate::sealed::IntPin + InputPin,
{
    /// Starts listening for the specified event
    pub fn listen(&mut self, event: Event) -> Result<(), E> {
        match event {
            Event::Pkt => self.bit_field_set(common::Register::EIE, common::EIE::mask().pktie()),
        }
    }

    /// Checks if there's any interrupt pending to be processed by polling the INT pin
    pub fn interrupt_pending(&mut self) -> bool {
        self.int.is_low()
    }

    /// Stops listening for the specified event
    pub fn unlisten(&mut self, event: Event) -> Result<(), E> {
        match event {
            Event::Pkt => self.bit_field_clear(common::Register::EIE, common::EIE::mask().pktie()),
        }
    }
}

/// A packet that has not been read yet
pub struct NextPacket<'a, SPI, NCS, INT, RESET> {
    enc28j60: &'a mut Enc28j60<SPI, NCS, INT, RESET>,
    next_packet: u16,
    len: u16,
}

impl<'a, E, SPI, NCS, INT, RESET> NextPacket<'a, SPI, NCS, INT, RESET>
where
    SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
    NCS: OutputPin,
{
    /// Size of this packet
    pub fn len(&self) -> u16 {
        self.len
    }

    /// Ignores this packet
    pub fn ignore(self) -> Result<(), E> {
        // update ERXRDPT
        // due to Errata #14 we must write an odd address to ERXRDPT
        // we know that ERXST = 0, that ERXND is odd and that next_packet is even
        let rxrdpt = if self.next_packet < 1 || self.next_packet > self.enc28j60.rxnd + 1 {
            self.enc28j60.rxnd
        } else {
            self.next_packet - 1
        };
        // "To move ERXRDPT, the host controller must write to ERXRDPTL first."
        self.enc28j60
            .write_control_register(bank0::Register::ERXRDPTL, rxrdpt.low())?;
        self.enc28j60
            .write_control_register(bank0::Register::ERXRDPTH, rxrdpt.high())?;

        // decrease the packet count
        self.enc28j60
            .bit_field_set(common::Register::ECON2, common::ECON2::mask().pktdec())?;

        self.enc28j60.pending = false;
        self.enc28j60.next_packet = self.next_packet;

        Ok(())
    }

    /// Reads the packet data into the given `buffer`
    ///
    /// # Panics
    ///
    /// If `self.len()` is greater than `buffer.len()`
    pub fn read<B>(self, buffer: B) -> Result<B::SliceTo, E>
    where
        B: IntoSliceTo<u16, Element = u8>,
        B::SliceTo: AsMutSlice<Element = u8>,
    {
        assert!(buffer.as_slice().len() >= usize(self.len()));

        let mut b = buffer.into_slice_to(self.len());
        self.enc28j60.read_buffer_memory(None, b.as_mut_slice())?;

        self.ignore().map(move |_| b)
    }
}

/// Reset pin or interrupt pin left unconnected
pub struct Unconnected;

impl crate::sealed::ResetPin for Unconnected {
    fn reset(&mut self) {}
}

impl<OP> crate::sealed::ResetPin for OP
where
    OP: OutputPin + 'static,
{
    fn reset(&mut self) {
        self.set_low();
        self.set_high();
    }
}

impl crate::sealed::IntPin for Unconnected {}

impl<IP> crate::sealed::IntPin for IP where IP: InputPin + 'static {}

/// Packet type, used to configure receive filters
#[derive(Clone, Copy, Eq, PartialEq)]
pub enum Packet {
    /// Broadcast packets
    Broadcast,
    /// Multicast packets
    Multicast,
    /// Unicast packets
    Unicast,
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    _DO_NOT_MATCH_AGAINST_THIS_VARIANT,
}

#[derive(Clone, Copy, PartialEq)]
enum Bank {
    Bank0,
    Bank1,
    Bank2,
    Bank3,
}

#[derive(Clone, Copy)]
enum Instruction {
    /// Read Control Register
    RCR = 0b000_00000,
    /// Read Buffer Memory
    RBM = 0b001_11010,
    /// Write Control Register
    WCR = 0b010_00000,
    /// Write Buffer Memory
    WBM = 0b011_11010,
    /// Bit Field Set
    BFS = 0b100_00000,
    /// Bit Field Clear
    BFC = 0b101_00000,
    /// System Reset Command
    SRC = 0b111_11111,
}

impl Instruction {
    fn opcode(&self) -> u8 {
        *self as u8
    }
}

#[derive(Clone, Copy)]
enum Register {
    Bank0(bank0::Register),
    Bank1(bank1::Register),
    Bank2(bank2::Register),
    Bank3(bank3::Register),
    Common(common::Register),
}

impl Register {
    fn addr(&self) -> u8 {
        match *self {
            Register::Bank0(r) => r.addr(),
            Register::Bank1(r) => r.addr(),
            Register::Bank2(r) => r.addr(),
            Register::Bank3(r) => r.addr(),
            Register::Common(r) => r.addr(),
        }
    }

    fn bank(&self) -> Option<Bank> {
        Some(match *self {
            Register::Bank0(_) => Bank::Bank0,
            Register::Bank1(_) => Bank::Bank1,
            Register::Bank2(_) => Bank::Bank2,
            Register::Bank3(_) => Bank::Bank3,
            Register::Common(_) => return None,
        })
    }

    fn is_eth_register(&self) -> bool {
        match *self {
            Register::Bank0(r) => r.is_eth_register(),
            Register::Bank1(r) => r.is_eth_register(),
            Register::Bank2(r) => r.is_eth_register(),
            Register::Bank3(r) => r.is_eth_register(),
            Register::Common(r) => r.is_eth_register(),
        }
    }
}

register!(RxStatus, 0, u32, {
    #[doc = "Indicates length of the received frame"]
    byte_count @ 0..15,
    #[doc = "Indicates a packet over 50,000 bit times occurred or that a packet was dropped since the last receive"]
    long_event @ 16,
    #[doc = "Indicates that at some time since the last receive, a carrier event was detected"]
    carrier_event @ 18,
    #[doc = "Indicates that frame CRC field value does not match the CRC calculated by the MAC"]
    crc_error @ 20,
    #[doc = "Indicates that frame length field value in the packet does not match the actual data byte length and specifies a valid length"]
    length_check_error @ 21,
    #[doc = "Indicates that frame type/length field was larger than 1500 bytes (type field)"]
    length_out_of_range @ 22,
    #[doc = "Indicates that at the packet had a valid CRC and no symbol errors"]
    received_ok @ 23,
    #[doc = "Indicates packet received had a valid Multicast address"]
    multicast @ 24,
    #[doc = "Indicates packet received had a valid Broadcast address."]
    broadcast @ 25,
    #[doc = "Indicates that after the end of this packet, an additional 1 to 7 bits were received"]
    dribble_nibble @ 26,
    #[doc = "Current frame was recognized as a control frame for having a valid type/length designating it as a control frame"]
    receive_control_frame @ 27,
    #[doc = "Current frame was recognized as a control frame containing a valid pause frame opcode and a valid destination address"]
    receive_pause_control_frame @ 28,
    #[doc = "Current frame was recognized as a control frame but it contained an unknown opcode"]
    receive_unknown_opcode @ 29,
    #[doc = "Current frame was recognized as a VLAN tagged frame"]
    receive_vlan_type_detected @ 30,
});
