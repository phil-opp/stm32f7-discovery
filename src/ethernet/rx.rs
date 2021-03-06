use bit_field::BitField;
use core::convert::TryInto;
use volatile::Volatile;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct RxDescriptor {
    word_0: u32,
    word_1: u32,
    word_2: u32,
    word_3: u32,
}

impl RxDescriptor {
    pub const fn empty() -> RxDescriptor {
        RxDescriptor {
            word_0: 0,
            word_1: 0,
            word_2: 0,
            word_3: 0,
        }
    }

    pub fn new(buffer_start: *const u8, buffer_size: usize) -> RxDescriptor {
        let mut descriptor = RxDescriptor::empty();
        descriptor.set_buffer_1(buffer_start, buffer_size);
        descriptor.set_own(true);

        descriptor
    }

    pub fn reset(&mut self) {
        self.word_0 = 0;
        self.set_own(true);
    }

    #[allow(dead_code)]
    pub fn set_next(&mut self, next: *const Volatile<Self>) {
        self.word_3 = (next as usize).try_into().unwrap();
        self.word_1 |= 1 << 14; // RCH: Second address chained
    }

    pub fn set_end_of_ring(&mut self, value: bool) {
        self.word_1.set_bit(15, value);
    }

    pub fn own(&self) -> bool {
        self.word_0.get_bit(31)
    }

    fn set_own(&mut self, value: bool) {
        self.word_0.set_bit(31, value);
    }

    fn set_buffer_1(&mut self, buffer_start: *const u8, buffer_size: usize) {
        assert_eq!(self.buffer_1_address(), 0);
        self.set_buffer_1_address(buffer_start as usize);
        self.set_buffer_1_size(buffer_size);
    }

    fn buffer_1_address(&self) -> usize {
        self.word_2.try_into().unwrap()
    }

    fn set_buffer_1_address(&mut self, buffer_address: usize) {
        self.word_2 = buffer_address.try_into().unwrap();
    }

    fn set_buffer_1_size(&mut self, size: usize) {
        let size = size.try_into().unwrap();
        self.word_1.set_bits(0..13, size);
    }

    pub fn frame_len(&self) -> usize {
        self.word_0.get_bits(16..30).try_into().unwrap()
    }

    pub fn is_last_descriptor(&self) -> bool {
        self.word_0.get_bit(8)
    }

    pub fn is_first_descriptor(&self) -> bool {
        self.word_0.get_bit(9)
    }

    pub fn error(&self) -> bool {
        self.word_0.get_bit(15)
    }

    pub fn checksum_result(&self) -> ChecksumResult {
        let w = self.word_0;
        match (w.get_bit(5), w.get_bit(7), w.get_bit(0)) {
            (false, false, false) => ChecksumResult::NovellRaw,
            (true, false, false) => ChecksumResult::Passed(false, false),
            (true, false, true) => ChecksumResult::Error(false, true),
            (true, true, false) => ChecksumResult::Error(true, false),
            (true, true, true) => ChecksumResult::Error(true, true),
            (false, false, true) => ChecksumResult::Passed(false, true),
            (false, true, true) => ChecksumResult::Passed(true, true),
            (false, true, false) => unreachable!(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChecksumResult {
    Passed(bool, bool),
    Error(bool, bool),
    NovellRaw,
}
