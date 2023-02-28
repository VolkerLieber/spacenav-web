use crate::quat::{Quat, QuatOperationable};

pub type Vector = [f32; 3];
pub trait VectorOperationable {
    fn cross(&self, vb: &Vector) -> [f32; 3];
    fn qrot(&mut self, quat: &Quat);
}

impl VectorOperationable for Vector {
    fn cross(&self, vb: &Vector) -> [f32; 3] {
        [
            self[1] * vb[2] - self[2] * vb[1],
            self[2] * vb[0] - self[0] * vb[2],
            self[0] * vb[1] - self[1] * vb[0],
        ]
    }

    fn qrot(&mut self, quat: &Quat) {
        let mut inv_q: Quat = quat.clone();
        let mut tmp_q: Quat = quat.clone();
        let vq: Quat = [self[0], self[1], self[2], 0.0];

        inv_q.invert();
        tmp_q.mult(&vq);
        tmp_q.mult(&inv_q);

        self[0] = tmp_q[0];
        self[1] = tmp_q[1];
        self[2] = tmp_q[2];
    }
}
