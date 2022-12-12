use super::constants;

#[derive(Debug, Clone, Copy, Default)]
pub struct DistanceViolations {
    front_left: bool,
    front_right: bool,
    rear: bool,
    front_bottom: bool,
    rear_bottom: bool,
}

impl DistanceViolations {
    pub fn is_violated(&self) -> bool {
        self.front_bottom && self.front_left && self.front_right && self.rear && self.rear_bottom
    }

    pub fn update_front(&mut self, front_distance: f32) {
        self.front_left = front_distance <= constants::FRONT_WARN_DIST;
        self.front_right = front_distance <= constants::FRONT_WARN_DIST;
    }
}
