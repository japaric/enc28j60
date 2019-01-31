pub trait IntPin: 'static {}

pub trait ResetPin: 'static {
    fn reset(&mut self);
}
