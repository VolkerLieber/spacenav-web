use crate::quat::Quat;
use crate::spnav_posrot::Position;

pub type Matrix = [f32; 16];
pub trait MatrixOperationable {
    fn obj(&mut self, pos: &Position);
    fn view(&mut self, pos: &Position);
    fn quat(&mut self, q: &Quat);
    fn translation(&mut self, x: f32, y: f32, z: f32);
    fn mul(&mut self, mb: &Matrix);
}

impl MatrixOperationable for Matrix {
    fn obj(&mut self, pos: &Position) {
        let mut tmp: Matrix = [0.0; 16];
        self.quat(&pos.rot);
        println!("BEF:  {:?}", self);
        tmp.translation(pos.pos[0], pos.pos[1], pos.pos[2]);
        self.mul(&tmp)
    }

    fn view(&mut self, pos: &Position) {
        let mut tmp: Matrix = [0.0; 16];
        self.translation(pos.pos[0], pos.pos[1], pos.pos[2]);
        println!("BEF:  {:?}", self);
        tmp.quat(&pos.rot);
        self.mul(&tmp)
    }

    fn quat(&mut self, q: &Quat) {
        let xsq2 = 2.0 * q[0] * q[0];
        let ysq2 = 2.0 * q[1] * q[1];
        let zsq2 = 2.0 * q[2] * q[2];
        let sx = 1.0 - ysq2 - zsq2;
        let sy = 1.0 - xsq2 - zsq2;
        let sz = 1.0 - xsq2 - ysq2;

        self[3] = 0.0;
        self[7] = 0.0;
        self[11] = 0.0;
        self[12] = 0.0;
        self[13] = 0.0;
        self[14] = 0.0;
        self[15] = 1.0;

        self[0] = sx;
        self[1] = 2.0 * q[0] * q[1] + 2.0 * q[3] * q[2];
        self[2] = 2.0 * q[2] * q[0] - 2.0 * q[3] * q[1];
        self[4] = 2.0 * q[0] * q[1] - 2.0 * q[3] * q[2];
        self[5] = sy;
        self[6] = 2.0 * q[1] * q[2] + 2.0 * q[3] * q[0];
        self[8] = 2.0 * q[2] * q[0] + 2.0 * q[3] * q[1];
        self[9] = 2.0 * q[1] * q[2] - 2.0 * q[3] * q[0];
        self[10] = sz;
    }

    fn translation(&mut self, x: f32, y: f32, z: f32) {
        // [1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1]
        *self = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        self[12] = x;
        self[13] = y;
        self[14] = z;
    }

    fn mul(&mut self, mb: &Matrix) {
        let mut tmp: Matrix = [0.0; 16];
        let mut tmp_index = 0;
        for row in 0..4 {
            for j in 0..4 {
                tmp[tmp_index] = self[row * 4] * mb[j]
                    + self[row * 4 + 1] * mb[4 + j]
                    + self[row * 4 + 2] * mb[8 + j]
                    + self[row * 4 + 3] * mb[12 + j];
                tmp_index += 1;
            }
        }
        *self = tmp;
    }
}
