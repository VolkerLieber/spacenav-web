use crate::{quat::Quat, quat::QuatOperationable, spnav_event_motion, vector::VectorOperationable};

#[derive(Debug)]
pub struct Position {
    pub pos: [f32; 3],
    pub rot: Quat,
}
impl Position {
    pub fn new() -> Position {
        Position {
            // [0.054270774126052856,-0.09371928870677948,0.11698655784130096]
            pos: [0.0, 0.0, 0.0],
            rot: [0.0, 0.0, 0.0, 1.0],
        }
    }
    pub fn move_view(&mut self, motion: &spnav_event_motion) {
        let len: f32 =
            ((motion.rx * motion.rx + motion.ry * motion.ry + motion.rz * motion.rz) as f32).sqrt();

        if len < 0.0 || len > 0.0 {
            let x = (-motion.rx as f32) / len;
            let y = (-motion.ry as f32) / len;
            let z = (-motion.rz as f32) / len;
            self.rot.rotate(len * 0.001, x, y, z);
        }

        let mut trans: [f32; 3] = [
            (-motion.x as f32) * 0.001,
            (-motion.y as f32) * 0.001,
            (motion.z as f32) * 0.001,
        ];

        trans.qrot(&self.rot);

        self.pos[0] += trans[0];
        self.pos[1] += trans[1];
        self.pos[2] += trans[2];
    }

    pub fn move_obj(&mut self, motion: &spnav_event_motion) {
        let len: f32 =
            ((motion.rx * motion.rx + motion.ry * motion.ry + motion.rz * motion.rz) as f32).sqrt();

        self.pos[0] += (motion.x as f32) * 0.001;
        self.pos[1] += (motion.y as f32) * 0.001;
        self.pos[2] += (motion.z as f32) * 0.001;

        if len < 0.0 || len > 0.0 {
            let x = (motion.rx as f32) / len;
            let y = (motion.ry as f32) / len;
            let z = (motion.rz as f32) / len;
            self.rot.rotate(len * 0.001, x, y, z);
        }
    }
}
