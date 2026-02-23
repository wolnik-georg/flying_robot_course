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
