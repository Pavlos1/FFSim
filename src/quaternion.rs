use std::ops::Mul;

#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    inner: [f32; 4],
}

impl Quaternion {
    pub fn new(inner: [f32; 4]) -> Self {
        Quaternion {
            inner
        }
    }

    pub fn conj(self) -> Self {
        Quaternion {
            inner: [self.inner[0], - self.inner[1], - self.inner[2], - self.inner[3]],
        }
    }

    pub fn rotate(self, vec: [f32; 3]) -> [f32; 3] {
        let res = self * Quaternion::new([0f32, vec[0], vec[1], vec[2]]) * self.conj();
        [res.inner[1], res.inner[2], res.inner[3]]
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let a1 = self.inner[0];
        let b1 = self.inner[1];
        let c1 = self.inner[2];
        let d1 = self.inner[3];

        let a2 = rhs.inner[0];
        let b2 = rhs.inner[1];
        let c2 = rhs.inner[2];
        let d2 = rhs.inner[3];

        Quaternion {
            inner: [
                a1*a2 - b1*b2 - c1*c2 - d1*d2,
                a1*b2 + b1*a2 + c1*d2 - d1*c2,
                a1*c2 - b1*d2 + c1*a2 + d1*b2,
                a1*d2 + b1*c2 - c1*b2 + d1*a2
            ],
        }
    }
}