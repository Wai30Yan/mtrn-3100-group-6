use core::{
    f32,
    ops::{Index, IndexMut},
    unimplemented,
};

use na::Rotation2;

pub const MAP_SIZE: u8 = 9;
pub const CORNER_CUT: u8 = 2;
pub const LINE_LEN: u8 = 6;

pub const FB_SIZE: u8 = MAP_SIZE * LINE_LEN + 1;

pub type Point = (u8, u8);

pub struct Map {
    h_walls: [[Option<bool>; (MAP_SIZE + 1) as usize]; MAP_SIZE as usize],
    v_walls: [[Option<bool>; MAP_SIZE as usize]; (MAP_SIZE + 1) as usize],
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
        Connections {
            n: self.h_walls[p.0 as usize][p.1 as usize],
            e: self.v_walls[(p.0 + 1) as usize][p.1 as usize],
            s: self.h_walls[p.0 as usize][(p.1 + 1) as usize],
            w: self.v_walls[p.0 as usize][p.1 as usize],
        }
    }

    pub fn set_conns(&mut self, p: Point, conns: Connections) {
        if let Some(n) = conns.n {
            let c = &mut self.h_walls[p.0 as usize][p.1 as usize];
            if c.is_none() {
                *c = Some(n);
            }
        }
        if let Some(e) = conns.e {
            let c = &mut self.v_walls[(p.0 + 1) as usize][p.1 as usize];
            if c.is_none() {
                *c = Some(e);
            }
        }
        if let Some(s) = conns.s {
            let c = &mut self.h_walls[p.0 as usize][(p.1 + 1) as usize];
            if c.is_none() {
                *c = Some(s);
            }
        }
        if let Some(w) = conns.w {
            let c = &mut self.v_walls[p.0 as usize][p.1 as usize];
            if c.is_none() {
                *c = Some(w);
            }
        }
    }

    pub fn render(&self) -> [[bool; FB_SIZE as usize]; FB_SIZE as usize] {
        let mut fb = [[false; FB_SIZE as usize]; FB_SIZE as usize];

        // Dots
        for y in 0..MAP_SIZE + 1 {
            for x in 0..MAP_SIZE + 1 {
                fb[(x * LINE_LEN) as usize][(y * LINE_LEN) as usize] = true;
            }
        }

        // H lines
        for y in 0..MAP_SIZE + 1 {
            for x in 0..MAP_SIZE {
                for i in 1..LINE_LEN {
                    fb[(x * LINE_LEN + i) as usize][(y * LINE_LEN) as usize] =
                        match self.h_walls[x as usize][y as usize] {
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
                    fb[(x * LINE_LEN) as usize][(y * LINE_LEN + i) as usize] =
                        match self.v_walls[x as usize][y as usize] {
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
                fb[x as usize][y as usize] = false;
                fb[x as usize][(FB_SIZE - y - 1) as usize] = false;
                fb[(FB_SIZE - x - 1) as usize][y as usize] = false;
                fb[(FB_SIZE - x - 1) as usize][(FB_SIZE - y - 1) as usize] = false;
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

        res.v_walls[0] = [Some(false); MAP_SIZE as usize];
        res.v_walls[MAP_SIZE as usize] = [Some(false); MAP_SIZE as usize];

        for i in 0..MAP_SIZE {
            res.h_walls[i as usize][0] = Some(false);
            res.h_walls[i as usize][MAP_SIZE as usize] = Some(false);
        }

        res
    }
}
