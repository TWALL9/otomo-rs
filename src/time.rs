use fugit::Duration;

pub fn dur_from_millis(millis: u64) -> Duration<u64, 1, 1000000> {
    Duration::<u64, 1, 1000000>::millis(millis)
}
