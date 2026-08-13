use std::ops::Add;
#[derive(Debug, Copy, Clone)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

fn main() {
    let a = Vec3 {x: 0.0, y: 1.0, z: 2.0};
    let b = Vec3 {x: 3.0, y: 4.0, z: 5.0};
    let c = a + b;
    println!("{:?}", c);
}