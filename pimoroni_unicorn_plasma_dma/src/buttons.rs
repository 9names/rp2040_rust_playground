use embedded_hal::digital::v2::InputPin;
use rp2040_hal::gpio::DynPin;

pub struct Buttons {
    a: DynPin,
    b: DynPin,
    x: DynPin,
    y: DynPin,
}

impl Buttons {
    fn new(a: DynPin, b: DynPin, x: DynPin, y: DynPin) -> Self {
        Buttons { a, b, x, y }
    }

    pub fn read(&self) -> ButtonState {
        ButtonState::new(
            self.a.is_low().unwrap(),
            self.b.is_low().unwrap(),
            self.x.is_low().unwrap(),
            self.y.is_low().unwrap(),
        )
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ButtonState {
    pub a: bool,
    pub b: bool,
    pub x: bool,
    pub y: bool,
}

impl ButtonState {
    fn new(a: bool, b: bool, x: bool, y: bool) -> Self {
        Self { a, b, x, y }
    }

    /// if a new button pressed, and that press is true
    fn diff(&self, newstate: ButtonState) -> ButtonState {
        ButtonState::new(
            self.a != newstate.a && newstate.a,
            self.b != newstate.b && newstate.b,
            self.x != newstate.x && newstate.x,
            self.y != newstate.y && newstate.y,
        )
    }

    fn and(&self, mask: ButtonState) -> ButtonState {
        ButtonState::new(
            self.a == mask.a,
            self.b == mask.b,
            self.x == mask.x,
            self.y == mask.y,
        )
    }

    fn pressed(&self) -> ButtonState {
        self.and(ButtonState::new(true, true, true, true))
    }
}

pub struct ButtonHandler {
    buttons: Buttons,
    last: ButtonState,
}
impl ButtonHandler {
    pub fn new(a: DynPin, b: DynPin, x: DynPin, y: DynPin) -> Self {
        let buttons = Buttons::new(a, b, x, y);

        let last = buttons.read();
        ButtonHandler { buttons, last }
    }
    pub fn read(&mut self) -> ButtonState {
        let new_buttons = self.buttons.read();
        let pressed = self.last.diff(new_buttons).pressed();
        self.last = new_buttons;
        pressed
    }
}
