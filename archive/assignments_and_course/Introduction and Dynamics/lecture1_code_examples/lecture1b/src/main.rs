use std::error::Error;
use std::time::Duration;

use meshcat::types::*;
use nalgebra::Isometry3;

fn main() -> Result<(), Box<dyn Error>> {

    let meshcat = Meshcat::new("tcp://127.0.0.1:6000");
    meshcat.set_object(
            "/robot",
            LumpedObject::builder()
                .geometries(vec![Geometry::new(GeometryType::Cylinder {
                    radius_top: 0.5,
                    radius_bottom: 0.5,
                    height: 1.0,
                    radial_segments: 32,
                    height_segments: 1,
                    theta_start: 0.0,
                    theta_length: 2.0 * std::f64::consts::PI,
                })])
                .object(Object::new(
                    Isometry3::from_parts(
                        nalgebra::Translation3::new(0.0, -1.0, 0.0),
                        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                    ),
                    ObjectType::Mesh,
                ))
                .material(Material::builder().color(0x00ffff).build())
                .build(),
        )?;

    for z in 1..100 {
        meshcat.set_transform(
                   "/robot",
                   Isometry3::from_parts(
                       nalgebra::Translation3::new(0.0, 0.0, f64::from(z)*0.1),
                       nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                   ),
               )?;
        std::thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
