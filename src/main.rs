//! Main.rs
//! Copyright © 2026 Sean Springer
//! [This program is licensed under the "MIT License"]
//! Please see the file LICENSE in the source distribution of this software for license terms.
//!

#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::rprintln;
use rtt_target::rtt_init_print;

use core::sync::atomic::{
    AtomicU8,
    Ordering::{Acquire, Release},
};

use cortex_m::asm;
use cortex_m_rt::entry;
use critical_section_lock_mut::LockMut;
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};
use microbit::{
    board::Board,
    display::nonblocking::Display,
    display::nonblocking::{BitImage, GreyscaleImage},
    hal::{
        Timer, gpiote,
        pac::{Interrupt, NVIC, TIMER1, interrupt},
        twim,
    },
    pac::{TIMER0, TIMER2, clock, twim0::frequency::FREQUENCY_A},
};
use tiny_led_matrix::Render;

const TICKS_PER_SECOND: u32 = 1_000_000;
const TICK_PER_MS: u32 = TICKS_PER_SECOND / 1000;
const DEBOUNCE_TIME: u32 = TICK_PER_MS * 100;

static GPIOTE_PERIPHERAL: LockMut<gpiote::Gpiote> = LockMut::new();
static DEBOUNCE_TIMER: LockMut<Timer<TIMER1>> = LockMut::new();
static CLOCK: LockMut<Clock> = LockMut::new();
static CLOCK_TIMER: LockMut<Timer<TIMER2>> = LockMut::new();
static DISPLAY: LockMut<Display<TIMER0>> = LockMut::new();

struct LED {
    pub row: usize,
    pub col: usize,
    pub color: u8,
}

impl LED {
    fn new(row: usize, col: usize, color: u8) -> Self {
        LED { row, col, color }
    }
}

struct Clock {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Clock {
    const hour_hand_color: u8 = 6;
    const minute_hand_color: u8 = 3;

    const hand_loc: [[(usize, usize); 3]; 12] = [
        [(2, 2), (1, 2), (0, 2)], //12
        [(2, 2), (1, 3), (0, 3)], //1
        [(2, 2), (1, 3), (1, 4)], //2
        [(2, 2), (2, 3), (2, 4)], //3
        [(2, 2), (3, 3), (3, 4)], //4
        [(2, 2), (3, 3), (4, 3)], //5
        [(2, 2), (3, 2), (4, 2)], //6
        [(2, 2), (3, 1), (4, 1)], //7
        [(2, 2), (3, 1), (3, 0)], //8
        [(2, 2), (2, 1), (2, 0)], //9
        [(2, 2), (1, 1), (1, 0)], //10
        [(2, 2), (1, 1), (0, 1)], //11
    ];

    fn new() -> Self {
        Clock {
            hour: 0,
            minute: 0,
            second: 0,
        }
    }

    fn tick(&mut self) -> bool {
        let mut needs_update = false;

        self.second += 1;
        if self.second == 60 {
            needs_update = true;

            self.second = 0;
            self.minute += 1;

            if self.minute == 60 {
                self.hour += 1;

                if self.hour == 12 {
                    self.hour = 0;
                }
            }
        }

        needs_update
    }

    fn add_minute(&mut self) {
        self.minute += 1;
        if self.minute == 60 {
            self.minute = 0;
        }
    }

    fn add_hour(&mut self) {
        self.hour += 1;
        if self.hour == 12 {
            self.hour = 0;
        }
    }

    fn round(number: f32) -> usize {
        let mut integer: i32 = number as i32; //round down
        let remainder: f32 = number - (integer as f32); //get decimal remainder
        if remainder >= 0.5 {
            integer += 1;
        }
        integer as usize
    }

    fn render(&self) -> [[u8; 5]; 5] {
        let mut array = [[0u8; 5]; 5];

        let h_index = Clock::hand_loc[self.hour as usize];
        let minute = Clock::round(self.minute as f32 / 5.0) % 12;
        let m_index = Clock::hand_loc[minute];

        array[h_index[0].0][h_index[0].1] += Clock::hour_hand_color;
        array[h_index[1].0][h_index[1].1] += Clock::hour_hand_color;
        array[h_index[2].0][h_index[2].1] += Clock::hour_hand_color;

        array[m_index[0].0][m_index[0].1] += Clock::minute_hand_color;
        array[m_index[1].0][m_index[1].1] += Clock::minute_hand_color;
        array[m_index[2].0][m_index[2].1] += Clock::minute_hand_color;

        array
    }
}

#[interrupt]
fn GPIOTE() {
    // check for bouncing using a 100ms timer based coolddown:
    let mut debounced = false;
    DEBOUNCE_TIMER.with_lock(|debounce_timer| {
        if debounce_timer.read() == 0 {
            debounced = true;
            debounce_timer.start(DEBOUNCE_TIME);
        }
    });

    // grab a mutable reference to the Gpiote instance, determine which button sent the signal,
    // reset the interrupt, and update the RESOULTION atomic if debounced timer as timed out
    GPIOTE_PERIPHERAL.with_lock(|gpiote| {
        if gpiote.channel0().is_event_triggered() {
            //A button press - add hour
            gpiote.channel0().reset_events();
            CLOCK.with_lock(|clock| {
                clock.add_hour();
                DISPLAY.with_lock(|display| {
                    let img = GreyscaleImage::new(&clock.render());
                    display.show(&img);
                })
            });
        } else if gpiote.channel1().is_event_triggered() {
            //B button press - add minute
            gpiote.channel1().reset_events();
            CLOCK.with_lock(|clock| {
                clock.add_minute();
                DISPLAY.with_lock(|display| {
                    let img = GreyscaleImage::new(&clock.render());
                    display.show(&img);
                })
            });
        }
    });
}

#[interrupt]
fn TIMER2() {
    CLOCK_TIMER.with_lock(|clock_timer| {
        clock_timer.reset_event();
        clock_timer.start(TICKS_PER_SECOND);
    });

    CLOCK.with_lock(|clock| {
        if clock.tick() {
            DISPLAY.with_lock(|display| {
                let img = GreyscaleImage::new(&clock.render());
                display.show(&img);
            })
        }
    });
}

#[interrupt]
fn TIMER0() {
    DISPLAY.with_lock(|display| {
        display.handle_display_event();
    })
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let board = Board::take().unwrap();

    //let mut display_timer = Timer::new(board.TIMER0);
    let mut debounce_timer = Timer::new(board.TIMER1);
    let mut clock_timer = Timer::new(board.TIMER2);
    let clock = Clock::new();
    //let mut clock_lag_timer = Timer::new(board.TIMER3);

    let mut display = Display::new(board.TIMER0, board.display_pins);

    // ensure buttons are in Floating mode
    let a_btn = board.buttons.button_a.into_floating_input();
    let b_btn = board.buttons.button_b.into_floating_input();

    //setup GPIOTE for both button press interrupts
    // b increases minutes hand, a incrases hour hand
    let gpiote = gpiote::Gpiote::new(board.GPIOTE);
    let channel0 = gpiote.channel0();
    let channel1 = gpiote.channel1();
    channel0
        .input_pin(&a_btn.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel0.reset_events();
    channel1
        .input_pin(&b_btn.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel1.reset_events();
    GPIOTE_PERIPHERAL.init(gpiote);

    //setup debounce timer
    debounce_timer.disable_interrupt();
    debounce_timer.reset_event();
    DEBOUNCE_TIMER.init(debounce_timer);

    clock_timer.enable_interrupt();
    clock_timer.reset_event();
    clock_timer.start(TICKS_PER_SECOND);
    CLOCK_TIMER.init(clock_timer);

    let img = GreyscaleImage::new(&clock.render());
    display.show(&img);

    CLOCK.init(clock);
    DISPLAY.init(display);

    // Set up the NVIC to handle interrupts.
    unsafe { NVIC::unmask(Interrupt::GPIOTE) }; // allow NVIC to handle GPIOTE signals
    unsafe { NVIC::unmask(Interrupt::TIMER2) }; // allow NVIC to handle GPIOTE signals
    unsafe { NVIC::unmask(Interrupt::TIMER0) }; // allow NVIC to handle GPIOTE signals
    NVIC::unpend(Interrupt::GPIOTE); //clear any currently pending GPIOTE state

    loop {
        asm::wfi();
    }
}
