#![feature(lang_items)]
#![feature(const_fn)]
#![feature(alloc, collections)]
#![feature(asm)]

#![no_std]
#![no_main]

#[macro_use]
extern crate stm32f7_discovery as stm32f7;

// initialization routines for .data and .bss
extern crate r0;
extern crate alloc;
#[macro_use]
extern crate collections;
extern crate splines;

// hardware register structs with accessor methods
use stm32f7::{system_clock, sdram, lcd, i2c, audio, touch, board, ethernet, embedded};
use stm32f7::lcd::Color;
use splines::ControlPoly;
use splines::renderer::{AABB, Renderer};
use splines::renderer::coordinates::{Coord2D, Pixel16};


#[no_mangle]
pub unsafe extern "C" fn reset() -> ! {
    extern "C" {
        static __DATA_LOAD: u32;
        static __DATA_END: u32;
        static mut __DATA_START: u32;

        static mut __BSS_START: u32;
        static mut __BSS_END: u32;
    }

    let data_load = &__DATA_LOAD;
    let data_start = &mut __DATA_START;
    let data_end = &__DATA_END;

    let bss_start = &mut __BSS_START;
    let bss_end = &__BSS_END;

    // initializes the .data section (copy the data segment initializers from flash to RAM)
    r0::init_data(data_start, data_end, data_load);
    // zeroes the .bss section
    r0::zero_bss(bss_start, bss_end);

    stm32f7::heap::init();

    // enable floating point unit
    let scb = stm32f7::cortex_m::peripheral::scb_mut();
    scb.cpacr.modify(|v| v | 0b1111 << 20);
    asm!("DSB; ISB;"::::"volatile"); // pipeline flush

    main(board::hw());
}

// WORKAROUND: rust compiler will inline & reorder fp instructions into
#[inline(never)] //             reset() before the FPU is initialized
fn main(hw: board::Hardware) -> ! {
    use embedded::interfaces::gpio::{self, Gpio};

    let x = vec![1, 2, 3, 4, 5];
    assert_eq!(x.len(), 5);
    assert_eq!(x[3], 4);

    let board::Hardware {
        rcc,
        pwr,
        flash,
        fmc,
        ltdc,
        gpio_a,
        gpio_b,
        gpio_c,
        gpio_d,
        gpio_e,
        gpio_f,
        gpio_g,
        gpio_h,
        gpio_i,
        gpio_j,
        gpio_k,
        i2c_3,
        sai_2,
        syscfg,
        ethernet_mac,
        ethernet_dma,
        ..
    } = hw;

    let mut gpio = Gpio::new(gpio_a,
                             gpio_b,
                             gpio_c,
                             gpio_d,
                             gpio_e,
                             gpio_f,
                             gpio_g,
                             gpio_h,
                             gpio_i,
                             gpio_j,
                             gpio_k);

    system_clock::init(rcc, pwr, flash);

    // enable all gpio ports
    rcc.ahb1enr
        .update(|r| {
            r.set_gpioaen(true);
            r.set_gpioben(true);
            r.set_gpiocen(true);
            r.set_gpioden(true);
            r.set_gpioeen(true);
            r.set_gpiofen(true);
            r.set_gpiogen(true);
            r.set_gpiohen(true);
            r.set_gpioien(true);
            r.set_gpiojen(true);
            r.set_gpioken(true);
        });

    // configure led pin as output pin
    let led_pin = (gpio::Port::PortI, gpio::Pin::Pin1);
    let mut led = gpio.to_output(led_pin,
                                 gpio::OutputType::PushPull,
                                 gpio::OutputSpeed::Low,
                                 gpio::Resistor::NoPull)
        .expect("led pin already in use");

    // turn led on
    led.set(true);

    let button_pin = (gpio::Port::PortI, gpio::Pin::Pin11);
    let button = gpio.to_input(button_pin, gpio::Resistor::NoPull)
        .expect("button pin already in use");

    // init sdram (needed for display buffer)
    sdram::init(rcc, fmc, &mut gpio);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();

    layer_1.clear();
    layer_2.clear();

    stm32f7::init_stdout(layer_2);

    // i2c
    i2c::init_pins_and_clocks(rcc, &mut gpio);
    let mut i2c_3 = i2c::init(i2c_3);
    i2c_3.test_1();
    i2c_3.test_2();

    // sai and stereo microphone
    audio::init_sai_2_pins(&mut gpio);
    audio::init_sai_2(sai_2, rcc);
    assert!(audio::init_wm8994(&mut i2c_3).is_ok());

    // ethernet
    let mut eth_device = ethernet::EthernetDevice::new(Default::default(),
                                                       Default::default(),
                                                       rcc,
                                                       syscfg,
                                                       &mut gpio,
                                                       ethernet_mac,
                                                       ethernet_dma);
    if let Err(e) = eth_device {
        println!("ethernet init failed: {:?}", e);
    }

    println!("Hello World!\n");

    // splines:
    // AABB
    let mut aabb_spk = AABB {
        min: Pixel16 { x: 10, y: 10 },
        max: Pixel16 { x: 120, y: 120 },
    };
    let mut aabb_line = AABB {
        min: Pixel16 { x: 80, y: 80 },
        max: Pixel16 { x: 470, y: 260 },
    };
    // create control polygon
    let mut ctrl_line: ControlPoly = ControlPoly {
        data: vec![Coord2D { x: 0f32, y: 1f32 },
                   Coord2D { x: 1f32, y: 0f32 },
                   Coord2D { x: 2f32, y: 0.9f32 },
                   Coord2D { x: 3f32, y: 0f32 }],
        closed: false, // not yet used
        line_thickness: 10, // not yet used
    };
    ctrl_line.data = aabb_line.fit_in_aabb(ctrl_line.data.as_slice());
    let mut ctrl_spk = ControlPoly {
        data: vec![Coord2D { x: 1f32, y: 2f32 },
                   Coord2D { x: 0.5f32, y: 3f32 },
                   Coord2D {
                       x: 0.6f32,
                       y: 0.8f32,
                   },
                   Coord2D { x: 2f32, y: 2.1f32 },
                   Coord2D {
                       x: 2.2f32,
                       y: 2.2f32,
                   },
                   Coord2D {
                       x: 5.5f32,
                       y: 2.2f32,
                   },
                   Coord2D { x: 5.5f32, y: 0f32 },
                   Coord2D { x: 0f32, y: 0f32 },
                   Coord2D { x: 0f32, y: 2f32 },
                   Coord2D { x: 1f32, y: 2f32 }],
        closed: false,
        line_thickness: 10,
    };
    ctrl_spk.data = aabb_spk.fit_in_aabb(ctrl_spk.data.as_slice());
    // approximate splines
    let draw_spk = ctrl_spk.eval_spline_uniform((aabb_spk.max.x - aabb_spk.min.x) / 10);
    let draw_line = ctrl_line.eval_spline_uniform((aabb_line.max.x - aabb_line.min.x) / 10);
    // set up renderer
    {
        let func = |x: u16, y: u16, c: f32| {
            layer_1.print_point_color_at(x as usize,
                                         y as usize,
                                         Color {
                                             red: 0xFF,
                                             green: 0x00,
                                             blue: 0x00,
                                             alpha: (c * (0xFF as u32) as f32) as u8,
                                         })
        };
        //let func2 = |x: u16, y: u16, c: u32| print!("({},{}), ", x, y);
        let mut r_draw = Renderer { pixel_col_fn: func };
        //let mut r_print = Renderer { pixel_col_fn: func2 };
        // plot data into given AABB.
        //r_print.draw_lines(aabb, draw.data.clone());
        r_draw.draw_lines(aabb_line, ctrl_line.data);
        //r_draw.draw_lines(aabb_spk, draw_spk);
        r_draw.draw_lines(aabb_spk, draw_line);
    }
    touch::check_family_id(&mut i2c_3).unwrap();

    //let mut audio_writer = layer_1.audio_writer();
    let mut last_led_toggle = system_clock::ticks();
    let mut last_color_change = system_clock::ticks();
    let mut button_pressed_old = false;
    loop {
        let ticks = system_clock::ticks();

        // every 0.5 seconds
        if ticks - last_led_toggle >= 500 {
            // toggle the led
            let led_current = led.get();
            led.set(!led_current);
            last_led_toggle = ticks;
        }

        let button_pressed = button.get();
        //if (button_pressed && !button_pressed_old) || ticks - last_color_change >= 1000 {
        //    // choose a new background color
        //    let new_color = ((system_clock::ticks() as u32).wrapping_mul(19801)) % 0x1000000;
        //    lcd.set_background_color(lcd::Color::from_hex(new_color));
        //    last_color_change = ticks;
        //}

        // poll for new audio data
        while !sai_2.bsr.read().freq() {} // fifo_request_flag
        let data0 = sai_2.bdr.read().data();
        while !sai_2.bsr.read().freq() {} // fifo_request_flag
        let data1 = sai_2.bdr.read().data();

        //audio_writer.set_next_col(data0, data1);

        // poll for new touch data
        //for touch in &touch::touches(&mut i2c_3).unwrap() {
        //    audio_writer
        //        .layer()
        //        .print_point_at(touch.x as usize, touch.y as usize);
        //}

        // handle new ethernet packets
        if let Ok(ref mut eth_device) = eth_device {
            loop {
                if let Err(err) = eth_device.handle_next_packet() {
                    match err {
                        stm32f7::ethernet::Error::Exhausted => {}
                        _ => {} // println!("err {:?}", e),
                    }
                    break;
                }
            }
        }

        button_pressed_old = button_pressed;
    }
}
