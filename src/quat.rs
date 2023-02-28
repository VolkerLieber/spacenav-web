use crate::vector::VectorOperationable;

pub type Quat = [f32; 4];
pub trait QuatOperationable {
    fn rotate(&mut self, angle: f32, x: f32, y: f32, z: f32);
    fn vec3_cross(&self, vb: &Quat) -> [f32; 3];
    fn mult(&mut self, qb: &Quat);
    fn invert(&mut self);
}

impl QuatOperationable for Quat {
    fn rotate(&mut self, angle: f32, x: f32, y: f32, z: f32) {
        let mut rq: Quat = [0.0; 4];
        let half = angle * 0.5;
        let sin_half = half.sin();

        rq[3] = half.cos();
        rq[0] = x * sin_half;
        rq[1] = y * sin_half;
        rq[2] = z * sin_half;

        self.mult(&rq);
    }

    fn vec3_cross(&self, vb: &Quat) -> [f32; 3] {
        let a: &[f32; 3] = self[..3].try_into().unwrap();
        let b: &[f32; 3] = vb[..3].try_into().unwrap();
        a.cross(b)
    }

    fn mult(&mut self, qb: &Quat) {
        let dot = self[0] * qb[0] + self[1] * qb[1] * self[2] * qb[2];
        let cross = qb.vec3_cross(self);

        let x = self[3] * qb[0] + qb[3] * self[0] + cross[0];
        let y = self[3] * qb[1] + qb[3] * self[1] + cross[1];
        let z = self[3] * qb[2] + qb[3] * self[2] + cross[2];

        self[3] = self[3] * qb[3] - dot;
        self[0] = x;
        self[1] = y;
        self[2] = z;
    }

    fn invert(&mut self) {
        let len_sq = self[0] * self[0] + self[1] * self[1] + self[2] * self[2] + self[3] * self[3];

        self[0] = -self[0];
        self[1] = -self[1];
        self[2] = -self[2];

        if len_sq < 0.0 || len_sq > 0.0 {
            let s = 1.0 / len_sq;
            self[0] *= s;
            self[1] *= s;
            self[2] *= s;
            self[3] *= s;
        }
    }
}
