use core::mem;
use core::ops::Range;

use byteorder::{ByteOrder, BE, LE};

pub(crate) trait OffsetSize {
    fn offset(self) -> u8;
    fn size(self) -> u8;
}

impl OffsetSize for u8 {
    fn offset(self) -> u8 {
        self
    }

    fn size(self) -> u8 {
        1
    }
}

impl OffsetSize for Range<u8> {
    fn offset(self) -> u8 {
        self.start
    }

    fn size(self) -> u8 {
        self.end - self.start
    }
}

pub(crate) trait U16Ext {
    fn from_parts(low: u8, high: u8) -> Self;

    fn low(self) -> u8;

    fn high(self) -> u8;

    // Big Endian byte representation
    fn be_repr(self) -> [u8; 2];

    // Little Endian byte representation
    fn le_repr(self) -> [u8; 2];
}

impl U16Ext for u16 {
    fn from_parts(low: u8, high: u8) -> u16 {
        ((high as u16) << 8) + low as u16
    }

    fn low(self) -> u8 {
        (self & 0xff) as u8
    }

    fn high(self) -> u8 {
        (self >> 8) as u8
    }

    fn be_repr(self) -> [u8; 2] {
        let mut bytes: [u8; 2] = unsafe { mem::uninitialized() };
        BE::write_u16(&mut bytes, self);
        bytes
    }

    fn le_repr(self) -> [u8; 2] {
        let mut bytes: [u8; 2] = unsafe { mem::uninitialized() };
        LE::write_u16(&mut bytes, self);
        bytes
    }
}

#[derive(Clone, Copy)]
pub struct Mask;

#[derive(Clone, Copy)]
pub struct R;

#[derive(Clone, Copy)]
pub struct W;
