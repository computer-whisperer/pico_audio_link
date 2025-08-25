//! PIO backed I2S microphone (RX) input
//!
//! Captures 30-bit words (only 24 meaningful data bits from most I2S mics).
//! Layout of pushed word: bits 29:6 = 24-bit sample (MSB first), remaining bits ignored.

use fixed::traits::ToFixed;

use embassy_rp::clocks;
use embassy_rp::dma::{AnyChannel, Channel, Transfer};
use embassy_rp::pio::{
    program::pio_asm, Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin,
    ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::Peri;

// ----------------------------------------------------------------------------
// Program
// ----------------------------------------------------------------------------

pub struct PioI2sInProgram<'d, P: Instance> {
    pub(crate) prg: LoadedProgram<'d, P>,
}

impl<'d, P: Instance> PioI2sInProgram<'d, P> {
    pub fn new(common: &mut Common<'d, P>) -> Self {
        // let assembled = pio_asm!(
        //     ".side_set 1 ; BCLK pin",
        //     "ws_toggle:",
        //     "    mov y, !y         side 1",
        //     "    mov osr, y        side 0",
        //     "    set x, 29         side 1",
        //     "    out pins, 1       side 0",
        //     "loop:",
        //     "    in pins, 1        side 1",
        //     "    jmp x-- loop      side 0",
        // );
        let assembled = pio_asm!(
            ".side_set 1", // BCLK only on side-set
            // WS will be on OUT pins
            "start:",
            "    set x, 31        side 0", // Prepare for 32 bits
            "    set y, 0         side 1", // WS = 0 (left channel)
            "    mov osr, y       side 0",
            "    out pins, 1      side 1", // Output WS
            "left_channel:",
            // "    in pins, 1       side 0",     // Read bit on clock low
            "    nop             side 0", // Skip reading left channel (no shift into ISR)
            "    jmp x-- left_channel side 1", // Clock high
            // Right channel
            "    set x, 31        side 0", // Prepare for 32 bits
            "    set y, 1         side 1", // WS = 1 (right channel)
            "    mov osr, !null   side 0", // Load all 1s
            "    out pins, 1      side 1", // Output WS = 1
            "right_channel:",
            "    in pins, 1       side 0",      // Read bit on clock low
            // "    nop             side 0", // Skip reading right channel (no shift into ISR)
            "    jmp x-- right_channel side 1", // Clock high
            "    jmp start        side 0", // Repeat
        );
        let prg = common.load_program(&assembled.program);
        Self { prg }
    }
}

// ----------------------------------------------------------------------------
// Driver
// ----------------------------------------------------------------------------

pub struct PioI2sIn<'d, P: Instance, const S: usize> {
    dma: Peri<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> PioI2sIn<'d, P, S> {
    /// Create and start the I2S microphone state machine.
    ///
    /// Parameters:
    /// - bit_clock_pin: BCLK (sideset)
    /// - ws_pin: Word select / LRCLK (driven via OUT)
    /// - data_pin: Microphone data (input)
    pub fn new(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        bit_clock_pin: Peri<'d, impl PioPin>,
        ws_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        program: &PioI2sInProgram<'d, P>,
    ) -> Self {
        let bit_clock_pin = common.make_pio_pin(bit_clock_pin);
        let ws_pin = common.make_pio_pin(ws_pin);
        let data_pin = common.make_pio_pin(data_pin);

        let cfg = {
            let mut cfg = Config::default();
            // Attach program and declare sideset (BCLK)
            cfg.use_program(&program.prg, &[&bit_clock_pin]);
            // OUT pin (WS)
            cfg.set_out_pins(&[&ws_pin]);
            // IN pin (DATA)
            cfg.set_in_pins(&[&data_pin]);
            // Clock divider: 30 + 128/256 = 30.5 (matches reference C SDK)
            cfg.clock_divider = 30.5f64.to_fixed();
            // cfg.clock_divider = 20.to_fixed();
            // OUT shift (only one bit used per frame for WS)
            cfg.shift_out = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Right,
                auto_fill: false,
            };
            // IN shift: left, autopush every 30 bits
            cfg.shift_in = ShiftConfig {
                // threshold: 30,
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            cfg.fifo_join = FifoJoin::RxOnly;
            cfg
        };

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&bit_clock_pin, &ws_pin]);
        sm.set_pin_dirs(Direction::In, &[&data_pin]);
        // sm.set_enable(true);

        Self {
            dma: dma.into(),
            sm,
        }
    }

    /// Alternate constructor that derives the clock divider from a desired sample rate.
    /// Assumes a 32-bit slot (even though only 24 bits carry data) -> bit clock = sample_rate * 32 * 2.
    pub fn new_with_sample_rate(
        common: &mut Common<'d, P>,
        sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        bit_clock_pin: Peri<'d, impl PioPin>,
        ws_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
        program: &PioI2sInProgram<'d, P>,
    ) -> Self {
        let mut this = Self::new(common, sm, dma, bit_clock_pin, ws_pin, data_pin, program);
        this.set_sample_rate(sample_rate);
        this.sm.set_enable(true);
        this
    }

    /// Adjust the clock divider for a new sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: u32) {
        // bit clock frequency (BCLK) with 32-bit slots (L+R)
        let bclk = sample_rate as f64 * 32.0 * 2.0;
        let div = (clocks::clk_sys_freq() as f64 / bclk / 2.0).to_fixed();
        self.sm.set_clock_divider(div);
    }

    /// Enable the state machine.
    pub fn enable(&mut self) {
        self.sm.set_enable(true);
    }

    /// Disable the state machine.
    pub fn disable(&mut self) {
        self.sm.set_enable(false);
    }

    /// Query enabled state.
    pub fn is_enabled(&self) -> bool {
        self.sm.is_enabled()
    }

    /// Flush any pending words in the RX FIFO.
    // pub fn flush_rx(&mut self) {
    //     while self.sm.rx().pull().is_some() {}
    // }

    /// Abort operation (disables SM and flushes RX FIFO).
    pub fn abort(&mut self) {
        self.disable();
        // self.flush_rx();
    }

    /// Start a DMA transfer reading raw 32-bit words (30 valid bits).
    /// Return an in-progress dma transfer future. Awaiting it will guarantee a complete transfer.
    pub fn read<'b>(&'b mut self, buf: &'b mut [u32]) -> Transfer<'b, AnyChannel> {
        // Ensure RX FIFO is not joined unexpectedly.
        // self.sm.rx().dma_pull(self.dma.reborrow(), buf)
        self.sm.rx().dma_pull(self.dma.reborrow(), buf, false)
    }
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

/// Extract signed 24-bit sample (bits 29:6) from a captured 32-bit word.
pub fn extract_24bit(word: u32) -> i32 {
    // let sample = (word >> 8) & 0xFFFFFF;
    let sample = word & 0xFFFFFF;
    if (sample & 0x800000) != 0 {
        (sample as i32) | !0x00FF_FFFF
    } else {
        sample as i32
    }
}
