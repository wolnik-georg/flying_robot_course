#[derive(Debug)]
struct Multirotor1dState {
    z: f32,
    z_dot: f32,
}

#[derive(Debug)]
struct Multirotor1dAction {
    f1: f32,
}

#[derive(Debug)]
struct Multirotor1d {
    mass: f32,
    g: f32,
    dt: f32,
    x: Multirotor1dState,
}

impl Multirotor1d {
    fn step(&mut self, action: Multirotor1dAction) {
        let z_new = self.x.z + self.x.z_dot * self.dt;
        let z_dot_new = self.x.z_dot + ((action.f1 / self.mass) - self.g) * self.dt;

        self.x.z = z_new;
        self.x.z_dot = z_dot_new;
    }
}

fn main() {
    let mut robot = Multirotor1d { mass: 0.1, g: 9.81, dt: 0.1, x: Multirotor1dState { z: 0.0, z_dot: 0.0 } };
    println!("{:?}", robot);
    for _ in 1..100 {
        robot.step(Multirotor1dAction { f1: 1.0 });
        println!("{:?}", robot.x);
    }
}
