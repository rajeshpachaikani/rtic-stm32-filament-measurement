#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;

#[rtic::app(device = hal::pac, peripherals = true)]
mod app {
    use cortex_m::asm::delay;
    use cortex_m_semihosting::hprintln;
    use hal::{
        prelude::*,
        stm32,
        stm32::Interrupt,
        gpio::{
            Pin,
            Alternate,
            Edge,
            Output,
            Input,
            PushPull,
            PullUp,
            OpenDrain,
            ExtiPin,
            gpioa::*,
            gpiob::*,
            gpioc::*,
        },
    };
    use hal::pac::rtc::CRH;
    use rtic::export::monotonic;
    use systick_monotonic::*;

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>;

    pub enum PressedButton {
        Fast,
        Stop,
        Start,
        Reverse,
    }

    pub struct Buttons {
        fast: PB0<Input<PullUp>>,
        stop: PA9<Input<PullUp>>,
        start: PB10<Input<PullUp>>,
        reverse: PB1<Input<PullUp>>,
    }

    #[shared]
    struct Shared {
        buttons: Buttons,
        led: PC13<Output<PushPull>>,
        button_state: bool,
    }

    #[local]
    struct Local {
        time: fugit::Instant<u64, 1_u32, 1000_u32>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let cp: cortex_m::Peripherals = cx.core;
        let dp: stm32::Peripherals = cx.device;

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let mut afio = dp.AFIO.constrain();

        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        let systick = cp.SYST;
        let mono = Systick::new(systick, clocks.sysclk().to_Hz());

        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();

        // Configure interrupts in PIN A9, B0, B10, B1
        let mut fast = gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        fast.make_interrupt_source(&mut afio);
        fast.trigger_on_edge(&dp.EXTI, Edge::Falling);
        fast.enable_interrupt(&dp.EXTI);

        let mut stop = gpioa.pa9.into_pull_up_input(&mut gpioa.crh);
        stop.make_interrupt_source(&mut afio);
        stop.trigger_on_edge(&dp.EXTI, Edge::Rising);
        stop.enable_interrupt(&dp.EXTI);

        let mut start = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
        start.make_interrupt_source(&mut afio);
        start.trigger_on_edge(&dp.EXTI, Edge::Rising);
        start.enable_interrupt(&dp.EXTI);

        let mut reverse = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);
        reverse.make_interrupt_source(&mut afio);
        reverse.trigger_on_edge(&dp.EXTI, Edge::Rising);
        reverse.enable_interrupt(&dp.EXTI);

        let mut btns = Buttons {
            fast,
            stop,
            start,
            reverse,
        };

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        let button_state = false;

        // Set monotonic timer to zero
        let time = monotonics::now();

        (Shared { buttons: btns, led, button_state }, Local { time }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }


    #[task(binds = EXTI0, shared = [buttons, led, button_state], local = [time])]
    fn exti0(cx: exti0::Context) {
        // let past = cx.local.time;
        let now = monotonics::now();
        let mut led = cx.shared.led;
        if now.ticks() - cx.local.time.ticks() > 500 {
            *cx.local.time = monotonics::now();
            led.lock(|led| {
                led.toggle();
            });
        }
        let mut buttons = cx.shared.buttons;
        buttons.lock(|buttons| {
            buttons.fast.clear_interrupt_pending_bit();
        });
    }
}