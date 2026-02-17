#[derive(Debug)]
pub struct Multirotor1dState {
    pub z: f32,
    pub z_dot: f32,
}

#[derive(Debug)]
pub struct Multirotor1dAction {
    pub f1: f32,
}

#[derive(Debug)]
pub struct Multirotor1d {
    pub mass: f32,
    pub g: f32,
    pub dt: f32,
    pub x: Multirotor1dState,
}

impl Multirotor1d {
    pub fn step(&mut self, action: Multirotor1dAction) {
        let z_new = self.x.z + self.x.z_dot * self.dt;
        let z_dot_new = self.x.z_dot + ((action.f1 / self.mass) - self.g) * self.dt;

        self.x.z = z_new;
        self.x.z_dot = z_dot_new;
    }
}
