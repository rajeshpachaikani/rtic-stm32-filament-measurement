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

use core::sync::atomic::{AtomicBool, AtomicU32};

static LINEAR_GUIDE_DIRECTION: AtomicBool = AtomicBool::new(false);
static RUN_STATUS: AtomicBool = AtomicBool::new(false);
static FILAMENT_LENGTH_PULSE: AtomicU32 = AtomicU32::new(0);

#[app(device = hal::stm32, peripherals = true)]
mod app {
    // Include Start
    use core::sync::atomic::Ordering;
    use core::{f32::consts::PI,
               fmt::Write,
    };
    use cortex_m::asm;
    use cortex_m_semihosting::hprintln;
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
        pac::{I2C1, ADC1},
        gpio::{
            Alternate,
            OpenDrain,
            PB6,
            PB7,
            Edge,
            ExtiPin,
            Output,
            PC13,
            PushPull,
        },
        adc::{Adc},
    };
    use hal::gpio::{PA4, PB1, PB10};
    use hal::pac::TIM3;
    use hal::timer::{
        Channel,
        PwmHz,
        Tim2NoRemap,
        Tim3NoRemap,
        Timer,
    };
    use lcd_lcm1602_i2c::Lcd;
    use rtic::mutex_prelude::TupleExt02;
    use crate::{
        FILAMENT_LENGTH_PULSE,
        LINEAR_GUIDE_DIRECTION,
        pin_assignments,
        pin_assignments::PinAssignments,
        RUN_STATUS,
    };
    use systick_monotonic::*;
    // Include End

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100000>;

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

    pub struct SpeedControl {
        linear_guide_motor_speed: gpio::gpioa::PA7<gpio::Analog>,
        winder_motor_speed: gpio::gpioa::PA6<gpio::Analog>,
        adc: Adc<ADC1>,
    }

    pub struct LimitSwitch {
        limit_switch_state: bool,
        limit_switch: gpio::gpioa::PA2<gpio::Input<gpio::PullUp>>,
        limit_switch_time: u64,
    }

    #[shared]
    struct Shared {
        push_buttons: PushButtons,
        button_states: ButtonStates,
    }

    #[local]
    struct Local {
        direction_pins : DirectionPins,
        i2c: BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>,
        delay: hal::timer::delay::Delay<hal::stm32::TIM4, 10000>,
        led: PC13<Output<PushPull>>,
        proximity_sensor: gpio::PA9<gpio::Input<gpio::PullUp>>,
        speed_control: SpeedControl,
        push_button_time: u64,
        limit_switch: LimitSwitch,
        linear_guide_pulse: PwmHz<
            hal::pac::TIM3,
            Tim3NoRemap,
            hal::timer::Ch<3_u8>,
            gpio::Pin<Alternate<PushPull>, gpio::CRL, 'B', 1_u8>
        >,
        winder_motor_pulse: PwmHz<
            hal::pac::TIM2,
            Tim2NoRemap,
            hal::timer::Ch<3_u8>,
            gpio::Pin<Alternate<PushPull>, gpio::CRL, 'A', 3_u8>>,
    }

    pub struct DirectionPins {
        winder_motor_direction: PA4<Output<PushPull>>,
        linear_guide_motor_direction: PB10<Output<PushPull>>
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics)
    {
        let dp: hal::pac::Peripherals = cx.device;
        let cp: cortex_m::Peripherals = cx.core;
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();
        let systick = cp.SYST;
        let _clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);
        let mono = Systick::new(systick, _clocks.sysclk().to_Hz());
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
                frequency: 400_000.Hz(),
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

        // Winder Motor PWM Configuration
        let winder_motor = Timer::new(
            dp.TIM2,
            &_clocks,
        );
        let mut winder_motor_pulse = winder_motor.pwm_hz(
            all_pins.winder_motor_pulse,
            &mut afio.mapr,
            100.kHz(),
        );
        let max_duty = winder_motor_pulse.get_max_duty();
        winder_motor_pulse.enable(Channel::C4);
        winder_motor_pulse.set_duty(Channel::C4, max_duty / 2);

        // Linear Actuator PWM Configuration
        let linear_actuator_motor = Timer::new(
            dp.TIM3,
            &_clocks,
        );
        let mut linear_guide_pulse = linear_actuator_motor.pwm_hz(
            all_pins.linear_guide_motor_pulse,
            &mut afio.mapr,
            100.kHz(),
        );
        let max_duty = linear_guide_pulse.get_max_duty();
        linear_guide_pulse.enable(Channel::C4);
        linear_guide_pulse.set_duty(Channel::C4, max_duty / 2);

        // Proximity Sensor
        let mut proximity_sensor = all_pins.proximity_sensor;
        proximity_sensor.trigger_on_edge(&dp.EXTI, Edge::Falling);
        proximity_sensor.make_interrupt_source(&mut afio);
        proximity_sensor.enable_interrupt(&dp.EXTI);

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

        // Configure ADC for motor speed input
        let adc = Adc::adc1(
            dp.ADC1,
            _clocks,
        );

        let mut speed_control = SpeedControl {
            winder_motor_speed: all_pins.winder_motor_speed,
            linear_guide_motor_speed: all_pins.linear_guide_motor_speed,
            adc: adc,
        };

        let mut direction_pins = DirectionPins {
            winder_motor_direction: all_pins.winder_motor_direction,
            linear_guide_motor_direction: all_pins.linear_guide_motor_direction,
        };

        let limit_switch_time: u64 = 0;
        let push_button_time: u64 = 0;

        let limit_switch = LimitSwitch {
            limit_switch_state: false,
            limit_switch: all_pins.linear_guide_limit_switch,
            limit_switch_time,
        };


        (
            Shared { push_buttons, button_states },
            Local {
                direction_pins,
                winder_motor_pulse,
                linear_guide_pulse,
                speed_control,
                i2c,
                delay,
                led,
                proximity_sensor,
                limit_switch,
                push_button_time,
            },
            init::Monotonics(mono)
        )
    }

    fn get_button_states(push_buttons: &mut PushButtons) -> ButtonStates {
        ButtonStates {
            fast_forward: push_buttons.fast_forward.is_low(),
            start_stop: push_buttons.start_stop.is_low(),
            spool_change: push_buttons.spool_change.is_low(),
            winder_motor_reverse: push_buttons.winder_motor_reverse.is_low(),
        }
    }

    #[task(binds = EXTI2, local = [limit_switch])]
    fn exti2(cx: exti2::Context) {
        let limit_switch: &mut LimitSwitch = cx.local.limit_switch;
        if limit_switch.limit_switch.is_low() != limit_switch.limit_switch_state {
            limit_switch.limit_switch_time = monotonics::now().ticks();
        }
        if monotonics::now().ticks() - limit_switch.limit_switch_time > 1000 {
            LINEAR_GUIDE_DIRECTION.store(
                !LINEAR_GUIDE_DIRECTION.load(Ordering::Relaxed),
                Ordering::Relaxed,
            );
        }
        limit_switch.limit_switch_state = limit_switch.limit_switch.is_low();
        limit_switch.limit_switch.clear_interrupt_pending_bit();
    }

    // Task to get proximity sensor data
    #[task(binds = EXTI9_5, local = [proximity_sensor])]
    fn exti9_5(cx: exti9_5::Context) {
        let proximity_sensor = cx.local.proximity_sensor;
        if proximity_sensor.is_low() {
            FILAMENT_LENGTH_PULSE.fetch_add(1, Ordering::Relaxed);
        }
        proximity_sensor.clear_interrupt_pending_bit();
    }

    // Task to check the start/stop button PB12
    #[task(binds = EXTI15_10, shared = [push_buttons, button_states], local = [push_button_time])]
    fn exti15_10(cx: exti15_10::Context) {
        let push_button_time = cx.local.push_button_time;
        (cx.shared.push_buttons, cx.shared.button_states)
            .lock(|push_buttons, button_states| {
                let button_states_n = get_button_states(push_buttons);
                if (button_states_n.start_stop != button_states.start_stop)
                    || (button_states_n.fast_forward != button_states.fast_forward)
                    || (button_states_n.spool_change != button_states.spool_change)
                    || (button_states_n.winder_motor_reverse != button_states.winder_motor_reverse)
                {
                    *push_button_time = monotonics::now().ticks();
                }
                if monotonics::now().ticks() - *push_button_time > 1000 {
                    if button_states_n.fast_forward {
                        let status = RUN_STATUS.load(Ordering::Relaxed);
                        RUN_STATUS.store(!status, Ordering::Relaxed);
                    }
                    if button_states_n.start_stop {
                        let status = RUN_STATUS.load(Ordering::Relaxed);
                        RUN_STATUS.store(!status, Ordering::Relaxed);
                    }
                    if button_states_n.spool_change {
                        FILAMENT_LENGTH_PULSE.store(0, Ordering::Relaxed);
                    }
                }
                push_buttons.start_stop.clear_interrupt_pending_bit();
                push_buttons.fast_forward.clear_interrupt_pending_bit();
                push_buttons.spool_change.clear_interrupt_pending_bit();
                push_buttons.winder_motor_reverse.clear_interrupt_pending_bit();
                *button_states = button_states_n;
            });
    }

    // Get filament length from pulse counter
    // Roller Dia = 20cm
    fn get_filament_length() -> u32 {
        let pulse_count = FILAMENT_LENGTH_PULSE.load(Ordering::Relaxed);
        let circumference = 2.0 * PI * 0.2;
        let length = (pulse_count as f32 * circumference) / (2.0);
        length as u32
    }

    fn get_moving_average(data: heapless::Vec<u16, 128>) -> u16 {
        let mut sum: f32 = 0.0;
        for e in data.iter() {
            sum += *e as f32;
        }
        (sum as f32 / 128.0) as u16
    }

    fn get_pulse_count(rpm: u32, u_step: u8) -> u32 {
        ((rpm as f32/60 as f32)*200 as f32* u_step as f32) as u32
    }

    // Mapping function
    // Y = (X-A)/(B-A) * (D-C) + C
    // https://stackoverflow.com/questions/345187/math-mapping-numbers
    fn map_value(x: u16, a: i32, b: i32, c: i32, d: i32) -> u16 {
        let y = ((x as i32 - a) as f32 / (b - a) as f32) * (d - c) as f32 + c as f32;
        y as u16
    }

    #[idle(local = [ i2c, delay, speed_control,linear_guide_pulse, winder_motor_pulse, led ])]
    fn idle(cx: idle::Context) -> ! {
        let i2c = cx.local.i2c;
        let delay = cx.local.delay;
        let led: &mut PC13<Output<PushPull>> = cx.local.led;
        let speed_control: &mut SpeedControl = cx.local.speed_control;
        let winder_motor_speed_adc = &mut speed_control.winder_motor_speed;
        let linear_guide_speed_adc = &mut speed_control.linear_guide_motor_speed;
        let adc = &mut speed_control.adc;
        let linear_guide_pulse : &mut PwmHz<
            hal::pac::TIM3,
            Tim3NoRemap,
            hal::timer::Ch<3_u8>,
            gpio::Pin<
                Alternate<PushPull>,
                gpio::CRL,
                'B',
                1_u8>
        > = cx.local.linear_guide_pulse;
        linear_guide_pulse.set_period(1.Hz());
        let mut lcd = lcd_lcm1602_i2c::Lcd::new(i2c, delay)
            .address(0x27)
            .cursor_on(true)
            .rows(2)
            .init()
            .unwrap();
        lcd.clear().unwrap();
        let mut past: u64 = monotonics::now().ticks();
        let mut adc_event_time: u64 = 0;
        let mut winder_speed: u16 = 0;
        let mut linear_guide_speed: u16 = 0;
        let mut lcd_string: String<16> = String::new();
        lcd.set_cursor(0, 2).unwrap();
        lcd.write_str("makeitnow.in" ).unwrap();

        loop {
            // Poll for ADC update every 100ms
            if (monotonics::now().ticks() - adc_event_time) > 10000 {
                let ws_temp:u16 =  adc.read(winder_motor_speed_adc).unwrap();
                if winder_speed.abs_diff(ws_temp) > 200 {
                    winder_speed = ws_temp;
                    let speed = map_value(winder_speed, 100,3900, 0, 380);
                    let pulse_count = get_pulse_count(speed as u32, 8);
                    lcd.clear().unwrap();
                    lcd.set_cursor(0, 0).unwrap();
                    write!(&mut lcd_string, "PC:{}", pulse_count).unwrap();
                    lcd.write_str(lcd_string.as_str()).unwrap();
                    lcd_string.clear();
                    if pulse_count < 10{
                        linear_guide_pulse.set_period(1.Hz());
                    }else{
                        linear_guide_pulse.set_period(pulse_count.Hz());
                    }
                }

            }

        }
    }
}
