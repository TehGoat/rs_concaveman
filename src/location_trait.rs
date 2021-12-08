use std::fmt::Debug;

pub trait LocationTrait: Debug {
    fn get_x(&self) -> f64;
    fn get_y(&self) -> f64;
}