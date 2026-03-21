//! Safety module: altitude, speed, and geofence limits for multirotor

use crate::math::Vec3;

/// Safety limits configuration
#[derive(Debug, Clone)]
pub struct SafetyLimits {
    pub min_altitude: f32,
    pub max_altitude: f32,
    pub max_speed: f32,
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

impl SafetyLimits {
    /// Clamp position to geofence and altitude limits
    pub fn clamp_position(&self, pos: Vec3) -> Vec3 {
        Vec3::new(
            pos.x.clamp(self.x_min, self.x_max),
            pos.y.clamp(self.y_min, self.y_max),
            pos.z.clamp(self.min_altitude, self.max_altitude),
        )
    }

    /// Clamp velocity to speed limit
    pub fn clamp_velocity(&self, vel: Vec3) -> Vec3 {
        let speed = vel.norm();
        if speed > self.max_speed {
            vel * (self.max_speed / speed)
        } else {
            vel
        }
    }
}

/// Check if position or velocity exceeds safety limits
pub fn check_safety(limits: &SafetyLimits, pos: Vec3, vel: Vec3) -> SafetyStatus {
    let clamped_pos = limits.clamp_position(pos);
    let clamped_vel = limits.clamp_velocity(vel);
    SafetyStatus {
        altitude_ok: (pos.z >= limits.min_altitude) && (pos.z <= limits.max_altitude),
        speed_ok: vel.norm() <= limits.max_speed,
        geofence_ok: (pos.x >= limits.x_min) && (pos.x <= limits.x_max)
            && (pos.y >= limits.y_min) && (pos.y <= limits.y_max),
        clamped_pos,
        clamped_vel,
    }
}

#[derive(Debug, Clone)]
pub struct SafetyStatus {
    pub altitude_ok: bool,
    pub speed_ok: bool,
    pub geofence_ok: bool,
    pub clamped_pos: Vec3,
    pub clamped_vel: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> SafetyLimits {
        SafetyLimits {
            min_altitude: 0.1,
            max_altitude: 2.0,
            max_speed: 1.0,
            x_min: -3.0, x_max: 3.0,
            y_min: -3.0, y_max: 3.0,
        }
    }

    fn approx_eq(a: f32, b: f32) -> bool { (a - b).abs() < 1e-5 }

    #[test]
    fn clamp_position_inside_unchanged() {
        let l = limits();
        let pos = Vec3::new(1.0, -1.0, 0.5);
        let c = l.clamp_position(pos);
        assert!(approx_eq(c.x, 1.0) && approx_eq(c.y, -1.0) && approx_eq(c.z, 0.5));
    }

    #[test]
    fn clamp_position_x_out_of_bounds() {
        let l = limits();
        let pos = Vec3::new(5.0, 0.0, 0.5);
        let c = l.clamp_position(pos);
        assert!(approx_eq(c.x, 3.0));
    }

    #[test]
    fn clamp_position_below_min_altitude() {
        let l = limits();
        let pos = Vec3::new(0.0, 0.0, 0.0); // below 0.1
        let c = l.clamp_position(pos);
        assert!(approx_eq(c.z, 0.1));
    }

    #[test]
    fn clamp_position_above_max_altitude() {
        let l = limits();
        let pos = Vec3::new(0.0, 0.0, 5.0);
        let c = l.clamp_position(pos);
        assert!(approx_eq(c.z, 2.0));
    }

    #[test]
    fn clamp_velocity_under_limit_unchanged() {
        let l = limits();
        let vel = Vec3::new(0.5, 0.0, 0.0); // speed = 0.5 < 1.0
        let c = l.clamp_velocity(vel);
        assert!(approx_eq(c.x, 0.5) && approx_eq(c.y, 0.0));
    }

    #[test]
    fn clamp_velocity_over_limit_scaled_to_max_speed() {
        let l = limits();
        let vel = Vec3::new(2.0, 0.0, 0.0); // speed = 2.0
        let c = l.clamp_velocity(vel);
        let speed = (c.x * c.x + c.y * c.y + c.z * c.z).sqrt();
        assert!(approx_eq(speed, 1.0), "speed after clamp = {speed}");
        // direction preserved
        assert!(approx_eq(c.y, 0.0) && approx_eq(c.z, 0.0));
    }

    #[test]
    fn clamp_velocity_diagonal_direction_preserved() {
        let l = limits();
        let vel = Vec3::new(3.0, 4.0, 0.0); // speed = 5.0
        let c = l.clamp_velocity(vel);
        let speed = (c.x * c.x + c.y * c.y + c.z * c.z).sqrt();
        assert!(approx_eq(speed, 1.0));
        // ratio x/y preserved: 3:4
        assert!((c.x / c.y - 0.75).abs() < 1e-4);
    }

    #[test]
    fn check_safety_all_ok() {
        let l = limits();
        let s = check_safety(&l, Vec3::new(0.0, 0.0, 0.5), Vec3::new(0.3, 0.0, 0.0));
        assert!(s.altitude_ok && s.speed_ok && s.geofence_ok);
    }

    #[test]
    fn check_safety_altitude_violation() {
        let l = limits();
        let s = check_safety(&l, Vec3::new(0.0, 0.0, 3.0), Vec3::new(0.0, 0.0, 0.0));
        assert!(!s.altitude_ok);
        assert!(approx_eq(s.clamped_pos.z, 2.0));
    }

    #[test]
    fn check_safety_geofence_violation() {
        let l = limits();
        let s = check_safety(&l, Vec3::new(5.0, 0.0, 0.5), Vec3::new(0.0, 0.0, 0.0));
        assert!(!s.geofence_ok);
        assert!(approx_eq(s.clamped_pos.x, 3.0));
    }

    #[test]
    fn check_safety_speed_violation() {
        let l = limits();
        let s = check_safety(&l, Vec3::new(0.0, 0.0, 0.5), Vec3::new(2.0, 0.0, 0.0));
        assert!(!s.speed_ok);
        let clamped_speed = (s.clamped_vel.x * s.clamped_vel.x
            + s.clamped_vel.y * s.clamped_vel.y
            + s.clamped_vel.z * s.clamped_vel.z).sqrt();
        assert!(approx_eq(clamped_speed, 1.0));
    }
}
