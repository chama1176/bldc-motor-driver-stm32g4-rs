pub trait Indicator {
    fn on(&self);
    fn off(&self);
    fn toggle(&self);
}
