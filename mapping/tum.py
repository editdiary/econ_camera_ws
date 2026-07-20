"""TUM 궤적 라인 포맷 (rclpy 비의존, 테스트 가능)."""


def odom_to_tum_line(stamp_sec, px, py, pz, qx, qy, qz, qw):
    """TUM 한 줄: 't tx ty tz qx qy qz qw' (공백 구분, timestamp 초)."""
    return (
        f"{stamp_sec:.9f} "
        f"{px:.6f} {py:.6f} {pz:.6f} "
        f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
    )
