use core::{
    f32,
    ops::{Index, IndexMut},
    unimplemented,
};

use na::Rotation2;

use crate::print;

pub const MAP_SIZE: usize = 5;
pub const CORNER_CUT: usize = 1;
pub const LINE_LEN: usize = 6;

pub const FB_SIZE: usize = MAP_SIZE * LINE_LEN + 1;

pub type Point = (usize, usize);

pub struct Map {
    h_walls: [[Option<bool>; MAP_SIZE + 1]; MAP_SIZE],
    v_walls: [[Option<bool>; MAP_SIZE]; MAP_SIZE + 1],
}

#[derive(Debug, Default)]
pub struct Connections {
    n: Option<bool>,
    e: Option<bool>,
    s: Option<bool>,
    w: Option<bool>,
}

#[derive(Debug, Clone, Copy)]
pub enum Heading {
    North,
    East,
    South,
    West,
}

impl Connections {
    pub fn blocked() -> Self {
        Self {
            n: Some(false),
            e: Some(false),
            s: Some(false),
            w: Some(false),
        }
    }
}

impl Heading {
    pub fn between(pi: Point, pf: Point) -> Self {
        if pf.0 > pi.0 {
            Heading::East
        } else if pf.0 < pi.0 {
            Heading::West
        } else if pf.1 > pi.1 {
            Heading::South
        } else if pf.1 < pi.1 {
            Heading::North
        } else {
            unimplemented!()
        }
    }

    pub fn rotl(self) -> Self {
        match self {
            Heading::North => Heading::West,
            Heading::East => Heading::North,
            Heading::South => Heading::East,
            Heading::West => Heading::South,
        }
    }

    pub fn rotr(self) -> Self {
        match self {
            Heading::North => Heading::East,
            Heading::East => Heading::South,
            Heading::South => Heading::West,
            Heading::West => Heading::North,
        }
    }
}

impl From<Heading> for Rotation2<f32> {
    fn from(value: Heading) -> Self {
        match value {
            Heading::North => Rotation2::new(f32::consts::FRAC_PI_2),
            Heading::East => Rotation2::new(0.0),
            Heading::South => Rotation2::new(-f32::consts::FRAC_PI_2),
            Heading::West => Rotation2::new(f32::consts::PI),
        }
    }
}

impl Index<Heading> for Connections {
    type Output = Option<bool>;

    fn index(&self, index: Heading) -> &Self::Output {
        match index {
            Heading::North => &self.n,
            Heading::East => &self.e,
            Heading::South => &self.s,
            Heading::West => &self.w,
        }
    }
}

impl IndexMut<Heading> for Connections {
    fn index_mut(&mut self, index: Heading) -> &mut Self::Output {
        match index {
            Heading::North => &mut self.n,
            Heading::East => &mut self.e,
            Heading::South => &mut self.s,
            Heading::West => &mut self.w,
        }
    }
}

impl Map {
    pub fn get_conns(&self, p: Point) -> Connections {
        print!("{:?}\r\n", p);
        Connections {
            n: self.h_walls[p.0][p.1],
            e: self.v_walls[p.0 + 1][p.1],
            s: self.h_walls[p.0][p.1 + 1],
            w: self.v_walls[p.0][p.1],
        }
    }

    pub fn set_conns(&mut self, p: Point, conns: Connections) {
        if let Some(n) = conns.n {
            let c = &mut self.h_walls[p.0][p.1];
            if c.is_none() {
                *c = Some(n);
            }
        }
        if let Some(e) = conns.e {
            let c = &mut self.v_walls[p.0 + 1][p.1];
            if c.is_none() {
                *c = Some(e);
            }
        }
        if let Some(s) = conns.s {
            let c = &mut self.h_walls[p.0][p.1 + 1];
            if c.is_none() {
                *c = Some(s);
            }
        }
        if let Some(w) = conns.w {
            let c = &mut self.v_walls[p.0][p.1];
            if c.is_none() {
                *c = Some(w);
            }
        }
    }

    pub fn render(&self) -> [[bool; FB_SIZE]; FB_SIZE] {
        let mut fb = [[false; FB_SIZE]; FB_SIZE];

        // Dots
        for y in 0..MAP_SIZE + 1 {
            for x in 0..MAP_SIZE + 1 {
                fb[x * LINE_LEN][y * LINE_LEN] = true;
            }
        }

        // H lines
        for y in 0..MAP_SIZE + 1 {
            for x in 0..MAP_SIZE {
                for i in 1..LINE_LEN {
                    fb[x * LINE_LEN + i][y * LINE_LEN] = match self.h_walls[x][y] {
                        Some(true) => false,
                        Some(false) => true,
                        None => i % 2 == 0, // Dotted line
                    };
                }
            }
        }

        // V lines
        for y in 0..MAP_SIZE {
            for x in 0..MAP_SIZE + 1 {
                for i in 1..LINE_LEN {
                    fb[x * LINE_LEN][y * LINE_LEN + i] = match self.v_walls[x][y] {
                        Some(true) => false,
                        Some(false) => true,
                        None => i % 2 == 0, // Dotted line
                    };
                }
            }
        }

        // Clear corners
        for y in 0..CORNER_CUT * LINE_LEN {
            for x in 0..(CORNER_CUT * LINE_LEN - y) {
                fb[x][y] = false;
                fb[x][FB_SIZE - y - 1] = false;
                fb[FB_SIZE - x - 1][y] = false;
                fb[FB_SIZE - x - 1][FB_SIZE - y - 1] = false;
            }
        }

        fb
    }

    pub fn explored(&self) -> bool {
        for y in 0..MAP_SIZE {
            for x in 0..MAP_SIZE {
                let conns = self.get_conns((x, y));
                if conns.n.is_none() || conns.e.is_none() || conns.s.is_none() || conns.w.is_none()
                {
                    return false;
                }
            }
        }

        true
    }
}

impl Default for Map {
    fn default() -> Self {
        let mut res = Self {
            h_walls: Default::default(),
            v_walls: Default::default(),
        };

        for y in 0..CORNER_CUT {
            for x in 0..(CORNER_CUT - y) {
                res.set_conns((x, y), Connections::blocked());
                res.set_conns((x, MAP_SIZE - y - 1), Connections::blocked());

                res.set_conns((MAP_SIZE - x - 1, y), Connections::blocked());
                res.set_conns((MAP_SIZE - x - 1, MAP_SIZE - y - 1), Connections::blocked());
            }
        }

        res.v_walls[0] = [Some(false); MAP_SIZE];
        res.v_walls[MAP_SIZE] = [Some(false); MAP_SIZE];

        for i in 0..MAP_SIZE {
            res.h_walls[i][0] = Some(false);
            res.h_walls[i][MAP_SIZE] = Some(false);
        }

        res
    }
}
