#![deny(unsafe_code)]
#![no_main]
#![no_std]

/*
    Author: Rajesh Pachaikani
    Date: 09/05/2022
    Title: Firmware for filament extrusion line control and monitoring
    Description:
        Pin Configuration:
            - PA3: Winder Motor Pulse ( Connected to Timer 3 Channel 1)
            - PA4: Winder Motor Direction
            - PB1: Linear Guide Motor Pulse ( Connected to Timer 2 Channel 2)
            - PB10: Linear Guide Motor Direction
            - PB14: Push Button to reverse the direction of Winder Motor
            - PB13: Push Button for Spool Change
            - PB15: Push Button for Fast Forward Winder Motor
            - PB12: Push Button for Start/Stop both Motors
            - PA1: Proximity Sensor
            - PB6: I2C SCL
            - PB7: I2C SDA
            - PA11: USB -ve Line
            - PA12: USB +ve Line
            - PA6: Winder motor speed connected to ADC1_IN1
            - PA7: Linear guide motor speed connected to ADC1_IN2
            - PA2: Linear guide limit switch
*/
use rtic::app;

mod pin_assignments;

use core::sync::atomic::AtomicBool;

static RUN_STATUS: AtomicBool = AtomicBool::new(false);

#[app(device = hal::stm32, peripherals = true)]
mod app {
    // Include Start
    use core::sync::atomic::Ordering;
    use core::fmt::Write;
    use heapless::String;
    use panic_semihosting as _;
    use hal::
    {
        prelude::*,
        gpio,
        i2c::{
            BlockingI2c,
            DutyCycle,
            Mode,
        },
        pac::I2C1,
        gpio::{
            Alternate,
            OpenDrain,
            PB6,
            PB7,
        },
    };
    use hal::gpio::{Edge, ExtiPin, Output, PC13, PushPull};
    use rtic::mutex_prelude::TupleExt02;
    use crate::{pin_assignments, RUN_STATUS};
    use crate::pin_assignments::PinAssignments;
    use systick_monotonic::*;
    // Include End

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>;

    pub struct PushButtons {
        fast_forward: gpio::gpiob::PB15<gpio::Input<gpio::PullUp>>,
        start_stop: gpio::gpiob::PB12<gpio::Input<gpio::PullUp>>,
        spool_change: gpio::gpiob::PB13<gpio::Input<gpio::PullUp>>,
        winder_motor_reverse: gpio::gpiob::PB14<gpio::Input<gpio::PullUp>>,
    }

    pub struct ButtonStates {
        fast_forward: bool,
        start_stop: bool,
        spool_change: bool,
        winder_motor_reverse: bool,
    }

    #[shared]
    struct Shared {
        push_buttons: PushButtons,
        button_states: ButtonStates,
    }

    #[local]
    struct Local {
        i2c: BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>,
        delay: hal::timer::delay::Delay<hal::stm32::TIM4, 10000>,
        led: PC13<Output<PushPull>>,
        debounce_time_variable: u64,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics)
    {
        let mut dp = cx.device;
        let mut cp: cortex_m::Peripherals = cx.core;
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();

        let systick = cp.SYST;

        cp.SCB.set_sleepdeep();

        let _clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        let mono = Systick::new(systick, 48_000_000);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let all_pins: PinAssignments = pin_assignments::AssignPins::new(
            gpioa,
            gpiob,
            gpioc,
        );


        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (all_pins.i2c_scl, all_pins.i2c_sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 5000.Hz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            _clocks,
            1000,
            10,
            1000,
            1000,
        );
        let delay = dp.TIM4.delay::<10000>(&_clocks);

        let mut push_buttons = PushButtons {
            fast_forward: all_pins.fast_forward,
            start_stop: all_pins.start_stop,
            spool_change: all_pins.spool_change,
            winder_motor_reverse: all_pins.winder_motor_reverse,
        };

        let mut led = all_pins.led;
        led.set_low();

        // Proximity Sensor
        let mut proximity_sensor = all_pins.proximity_sensor;
        proximity_sensor.trigger_on_edge(&dp.EXTI, Edge::Falling);

        //Winder Reverse Button
        push_buttons.winder_motor_reverse.trigger_on_edge(&dp.EXTI, Edge::Falling);
        push_buttons.winder_motor_reverse.make_interrupt_source(&mut afio);
        push_buttons.winder_motor_reverse.enable_interrupt(&dp.EXTI);

        //Spool change button
        push_buttons.spool_change.trigger_on_edge(&dp.EXTI, Edge::Falling);
        push_buttons.spool_change.make_interrupt_source(&mut afio);
        push_buttons.spool_change.enable_interrupt(&dp.EXTI);

        //Fast forward button
        push_buttons.fast_forward.trigger_on_edge(&dp.EXTI, Edge::Falling);
        push_buttons.fast_forward.make_interrupt_source(&mut afio);
        push_buttons.fast_forward.enable_interrupt(&dp.EXTI);

        //Start/Stop button
        push_buttons.start_stop.trigger_on_edge(&dp.EXTI, Edge::Falling);
        push_buttons.start_stop.make_interrupt_source(&mut afio);
        push_buttons.start_stop.enable_interrupt(&dp.EXTI);

        let button_states = ButtonStates {
            fast_forward: false,
            start_stop: false,
            spool_change: false,
            winder_motor_reverse: false,
        };
        let debounce_time_variable: u64 = 0;
        (Shared { push_buttons, button_states }, Local { i2c, delay, led, debounce_time_variable }, init::Monotonics(mono))
    }

    fn get_button_states(push_buttons: &mut PushButtons) -> ButtonStates {
        ButtonStates {
            fast_forward: push_buttons.fast_forward.is_low(),
            start_stop: push_buttons.start_stop.is_low(),
            spool_change: push_buttons.spool_change.is_low(),
            winder_motor_reverse: push_buttons.winder_motor_reverse.is_low(),
        }
    }

    // Task to check the start/stop button PB12
    #[task(binds = EXTI15_10, shared = [push_buttons, button_states], local = [led, debounce_time_variable])]
    fn exti15_10(cx: exti15_10::Context) {
        let led = cx.local.led;
        let debounce_time_variable = cx.local.debounce_time_variable;
        (cx.shared.push_buttons, cx.shared.button_states)
            .lock(|push_buttons, button_states| {
                let button_states_n = get_button_states(push_buttons);
                if (button_states_n.start_stop != button_states.start_stop)
                    || (button_states_n.fast_forward != button_states.fast_forward)
                    || (button_states_n.spool_change != button_states.spool_change)
                    || (button_states_n.winder_motor_reverse != button_states.winder_motor_reverse){
                    *debounce_time_variable = monotonics::now().ticks();
                }
                if monotonics::now().ticks() - *debounce_time_variable > 1000 {
                    if button_states_n.fast_forward {
                        let status = RUN_STATUS.load(Ordering::Relaxed);
                        RUN_STATUS.store(!status, Ordering::Relaxed);
                        led.toggle();
                    }
                    if button_states_n.start_stop {
                        let status = RUN_STATUS.load(Ordering::Relaxed);
                        RUN_STATUS.store(!status, Ordering::Relaxed);
                        led.toggle();
                    }

                }
                push_buttons.start_stop.clear_interrupt_pending_bit();
                push_buttons.fast_forward.clear_interrupt_pending_bit();
                push_buttons.spool_change.clear_interrupt_pending_bit();
                push_buttons.winder_motor_reverse.clear_interrupt_pending_bit();
                *button_states = button_states_n;
            });
    }

    #[idle(local = [i2c, delay])]
    fn idle(cx: idle::Context) -> ! {
        let i2c = cx.local.i2c;
        let delay = cx.local.delay;

        let mut lcd = lcd_lcm1602_i2c::Lcd::new(i2c, delay)
            .address(0x27)
            .rows(2)
            .init()
            .unwrap();

        lcd.clear().unwrap();
        loop {
            let running = RUN_STATUS.load(Ordering::Relaxed);
            if running {
                lcd.write_str("Running").unwrap();
                lcd.set_cursor(0, 0).unwrap();
            } else {
                lcd.write_str("Stopped").unwrap();
                lcd.set_cursor(0, 0).unwrap();
            }
        }
    }
}

