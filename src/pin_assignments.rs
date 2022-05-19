/*
    Pin Configuration:
            - PA3: Winder Motor Pulse ( Connected to Timer 3 Channel 1)
            - PA4: Winder Motor Direction
            - PB1: Linear Guide Motor Pulse ( Connected to Timer 2 Channel 2)
            - PB10: Linear Guide Motor Direction
            - PB14: Push Button to reverse the direction of Winder Motor
            - PB13: Push Button for Spool Change
            - PB15: Push Button for Fast Forward Winder Motor
            - PB12: Push Button for Start/Stop both Motors
            - PA9: Proximity Sensor
            - PB6: I2C SCL
            - PB7: I2C SDA
            - PA11: USB -ve Line
            - PA12: USB +ve Line
            - PA6: Winder motor speed connected to ADC1_IN1
            - PA7: Linear guide motor speed connected to ADC1_IN2
            - PA2: Linear guide limit switch
 */
use hal::{
    gpio,
};

pub struct PinAssignments {
    pub winder_motor_pulse: gpio::gpioa::PA3<gpio::Alternate<gpio::PushPull>>,
    pub winder_motor_direction: gpio::gpioa::PA4<gpio::Output<gpio::PushPull>>,
    pub linear_guide_motor_pulse: gpio::gpiob::PB1<gpio::Alternate<gpio::PushPull>>,
    pub linear_guide_motor_direction: gpio::gpiob::PB10<gpio::Output<gpio::PushPull>>,
    pub winder_motor_reverse: gpio::gpiob::PB14<gpio::Input<gpio::PullUp>>,
    pub spool_change: gpio::gpiob::PB13<gpio::Input<gpio::PullUp>>,
    pub fast_forward: gpio::gpiob::PB15<gpio::Input<gpio::PullUp>>,
    pub start_stop: gpio::gpiob::PB12<gpio::Input<gpio::PullUp>>,
    pub proximity_sensor: gpio::gpioa::PA9<gpio::Input<gpio::PullUp>>,
    pub i2c_scl: gpio::gpiob::PB6<gpio::Alternate<gpio::OpenDrain>>,
    pub i2c_sda: gpio::gpiob::PB7<gpio::Alternate<gpio::OpenDrain>>,
    // pub usb_neg: gpio::gpioa::PA11<gpio::Input<gpio::PullUp>>,
    // pub usb_pos: gpio::gpioa::PA12<gpio::Input<gpio::PullUp>>,
    pub winder_motor_speed: gpio::gpioa::PA6<gpio::Analog>,
    pub linear_guide_motor_speed: gpio::gpioa::PA7<gpio::Analog>,
    pub linear_guide_limit_switch: gpio::gpioa::PA2<gpio::Input<gpio::PullUp>>,
    pub led: gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>,
}

pub trait AssignPins {
    fn new(
        gpioa: hal::gpio::gpioa::Parts,
        gpiob: hal::gpio::gpiob::Parts,
        gpioc: hal::gpio::gpioc::Parts)
        -> Self;
}

impl AssignPins for PinAssignments {
    fn new(
        gpioa: hal::gpio::gpioa::Parts,
        gpiob: hal::gpio::gpiob::Parts,
        gpioc: hal::gpio::gpioc::Parts)
        -> Self {
        let mut gpioa = gpioa;
        let mut gpiob = gpiob;
        let mut gpioc = gpioc;
        Self {
            winder_motor_pulse: gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
            winder_motor_direction: gpioa.pa4.into_push_pull_output(&mut gpioa.crl),
            linear_guide_motor_pulse: gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl),
            linear_guide_motor_direction: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
            winder_motor_reverse: gpiob.pb14.into_pull_up_input(&mut gpiob.crh),
            spool_change: gpiob.pb13.into_pull_up_input(&mut gpiob.crh),
            fast_forward: gpiob.pb15.into_pull_up_input(&mut gpiob.crh),
            start_stop: gpiob.pb12.into_pull_up_input(&mut gpiob.crh),
            proximity_sensor: gpioa.pa9.into_pull_up_input(&mut gpioa.crh),
            i2c_scl: gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
            i2c_sda: gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
            led: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
            linear_guide_limit_switch: gpioa.pa2.into_pull_up_input(&mut gpioa.crl),
            linear_guide_motor_speed: gpioa.pa7.into_analog(&mut gpioa.crl),
            winder_motor_speed: gpioa.pa6.into_analog(&mut gpioa.crl),
        }
    }
}