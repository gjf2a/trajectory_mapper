use crate::point::FloatPoint;
use r2r::nav_msgs::msg::Odometry;

pub fn find_goal_heading(p1: FloatPoint, p2: FloatPoint) -> f64 {
    (p2[1] - p1[1]).atan2(p2[0] - p1[0])
}

pub fn find_euclidean_distance(p1: FloatPoint, p2: FloatPoint) -> f64 {
    p1.euclidean_distance(p2)
}

pub fn find_roll_pitch_yaw(value: Odometry) -> (f64, f64, f64) {
    let (q1, q2, q3, q0) = (
        value.pose.pose.orientation.x,
        value.pose.pose.orientation.y,
        value.pose.pose.orientation.z,
        value.pose.pose.orientation.w,
    );
    (
        (2.0 * (q0 * q1 + q2 * q3))
            .atan2(q0.powf(2.0) - q1.powf(2.0) - q2.powf(2.0) + q3.powf(2.0)),
        (2.0 * (q0 * q2 - q1 * q3)).asin(),
        (q0 * q3 + q1 * q2).atan2(q0.powf(2.0) + q1.powf(2.0) - q2.powf(2.0) - q3.powf(2.0)),
    )
}

const PI: f64 = std::f64::consts::PI;

pub fn find_normalized_angle(angle: f64) -> f64 {
    let mut angle = python_like_modulo(angle, 2.0 * PI);
    if angle > PI {
        angle -= 2.0 * PI;
    }
    angle
}

pub fn find_angle_diff(angle1: f64, angle2: f64) -> f64 {
    find_normalized_angle(angle1 - angle2)
}

// From https://www.perplexity.ai/search/in-python-you-can-use-modulo-w-t4w5re3zRu.X1.UWigEQew
fn python_like_modulo(a: f64, b: f64) -> f64 {
    let r = a % b;
    if r != 0.0 && a.signum() != b.signum() {
        r + b
    } else {
        r
    }
}

/*


def find_angle_diff(angle1: float, angle2: float) -> float:
    """
    Find the shortest difference between two angles.
    Parameters should be in radians.
    """
    return find_normalized_angle(angle1 - angle2)


def find_normalized_angle(angle: float) -> float:
    """
    Ensure that the angle in radians lies between -math.pi and math.pi
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle
*/

#[cfg(test)]
mod tests {
    use super::python_like_modulo;

    #[test]
    fn test_modulo() {
        for (expected, a, b) in [(0.0, 1.75, 0.25), (0.25, 1.75, 0.5), (0.25, -1.75, 0.5), (1.6, 3.7, 2.1), (0.5, -3.7, 2.1)] {
            assert_eq!(expected, python_like_modulo(a, b));
        }
    }

    /*
    Translate these tests from Python to Rust:

        def test_distance(self):
        for a, b, c in [(3, 4, 5), (5, 12, 13), (7, 24, 25), (8, 15, 17), (9, 40, 41)]:
            p1 = Point()
            p2 = Point()
            p1.x = a
            p1.y = b
            self.assertEqual(c, find_euclidean_distance(p1, p2))

    def test_normalized_angle(self):
        for theta, normed in [(2 * math.pi, 0.0), (3/2 * math.pi, -math.pi/2), (9 * math.pi, math.pi)]:
            self.assertEqual(normed, find_normalized_angle(theta))

    def test_angle_diff(self):
        for theta1, theta2, diff in [
            ( 3/4 * math.pi,  1/4 * math.pi,  1/2 * math.pi), #1
            ( 1/4 * math.pi,  3/4 * math.pi, -1/2 * math.pi), #2
            (-1/3 * math.pi,  1/3 * math.pi, -2/3 * math.pi), #3
            (15/8 * math.pi,  1/8 * math.pi, -1/4 * math.pi), #4         
            ( 1/8 * math.pi, 15/8 * math.pi,  1/4 * math.pi), #5
            ( 9/2 * math.pi, -1/2 * math.pi,        math.pi), #6
        ]:
            self.assertAlmostEqual(diff, find_angle_diff(theta1, theta2), places=5)

    def test_roll_pitch_yaw(self):
        for x, y, z, w, rpw in [
            (-0.0031924284994602203, 0.005276054609566927, -0.8277794122695923, 0.561019778251648, 
             (-0.012317164052438935, 0.0006346888584906615, -1.9503363787472408)),
            (-0.002063487656414509,  0.0034074627328664064, -0.9837535619735718, 0.17948013544082642,
             (-0.007445015587338619, -0.002836786557391314, -2.7806632489220133))
        ]:
            q = Quaternion()
            q.x, q.y, q.z, q.w = x, y, z, w
            roll, pitch, yaw = find_roll_pitch_yaw(q)
            print(roll, pitch, yaw)
            self.assertAlmostEqual(rpw[0], roll,  places=5)
            self.assertAlmostEqual(rpw[1], pitch, places=5)
            self.assertAlmostEqual(rpw[2], yaw,   places=5)
     */
}