use core::matches;

use alloc::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    vec::Vec,
};
use na::{Rotation2, Translation2};

use crate::{
    CELL_SIZE, END, START, START_H,
    display::Display,
    map::{Connections, Heading, Map, Point},
    motion_manager::Motion,
    print,
};

const LIDAR_THRESH: f32 = 0.15;

enum ExplorerState {
    Begin,
    Nominal,
    Returning,
    End,
}

pub struct Explorer {
    map: Map,
    p: Point,
    h: Heading,
    state: ExplorerState,
}

impl Explorer {
    pub fn step(
        &mut self,
        lidar_l: Option<f32>,
        lidar_r: Option<f32>,
        lidar_f: Option<f32>,
        display: &mut Display,
    ) -> Vec<Motion> {
        if matches!(self.state, ExplorerState::End) {
            return Vec::new();
        }

        let mut conns = Connections::default();
        conns[self.h.rotl()] = Some(!lidar_l.map_or_default(|d| d < LIDAR_THRESH));
        conns[self.h.rotr()] = Some(!lidar_r.map_or_default(|d| d < LIDAR_THRESH));
        conns[self.h] = Some(!lidar_f.map_or_default(|d| d < LIDAR_THRESH));

        self.map.set_conns(self.p, conns);
        display.draw(0, 0, self.map.render());

        if matches!(self.state, ExplorerState::Begin) {
            self.h = self.h.rotl();
            self.state = ExplorerState::Nominal;
            return vec![Motion::Pivot {
                rotation: self.h.into(),
            }];
        }

        if self.map.explored() {
            print!("Complete\r\n");
            self.state = ExplorerState::Returning;
            self.bfs(self.p, |p| p == START).2
        } else if matches!(self.state, ExplorerState::Returning) {
            self.state = ExplorerState::End;
            self.bfs(START, |p| p == END).2
        } else {
            let (p, h, path) = self.bfs(self.p, |p| {
                let conns = self.map.get_conns(p);
                conns[Heading::North].is_none()
                    || conns[Heading::East].is_none()
                    || conns[Heading::South].is_none()
                    || conns[Heading::West].is_none()
            });
            self.p = p;
            self.h = h;
            path
        }
    }

    fn bfs<F>(&self, pi: Point, cond: F) -> (Point, Heading, Vec<Motion>)
    where
        F: Fn(Point) -> bool,
    {
        let mut open = VecDeque::<Point>::new();
        let mut explored = BTreeSet::<Point>::new();
        let mut parents = BTreeMap::<Point, Point>::new();
        open.push_back(pi);

        loop {
            let p = open.pop_front().unwrap();
            if cond(p) {
                let mut curr = p;
                let mut motions = Vec::<Motion>::new();
                let mut final_heading = None;

                while curr != pi {
                    let par = parents[&curr];
                    let rel_h = Heading::between(par, curr);
                    if final_heading.is_none() {
                        final_heading = Some(rel_h);
                    }
                    // Push in reverse order
                    motions.push(Motion::Line {
                        final_position: Translation2::new(
                            CELL_SIZE * (curr.0 as f32 + 0.5),
                            CELL_SIZE * -(curr.1 as f32 + 0.5),
                        ),
                        final_speed: 0.0,
                    });
                    motions.push(Motion::Pivot {
                        rotation: rel_h.into(),
                    });
                    curr = par;
                }

                return (p, final_heading.unwrap(), motions);
            }

            explored.insert(p);
            let conns = self.map.get_conns(p);

            if matches!(conns[Heading::North], Some(true)) {
                let np = (p.0, p.1 - 1);
                if !explored.contains(&np) {
                    open.push_back(np);
                    parents.insert(np, p);
                }
            }
            if matches!(conns[Heading::East], Some(true)) {
                let np = (p.0 + 1, p.1);
                if !explored.contains(&np) {
                    open.push_back(np);
                    parents.insert(np, p);
                }
            }
            if matches!(conns[Heading::South], Some(true)) {
                let np = (p.0, p.1 + 1);
                if !explored.contains(&np) {
                    open.push_back(np);
                    parents.insert(np, p);
                }
            }
            if matches!(conns[Heading::West], Some(true)) {
                let np = (p.0 - 1, p.1);
                if !explored.contains(&np) {
                    open.push_back(np);
                    parents.insert(np, p);
                }
            }
        }
    }
}

impl Default for Explorer {
    fn default() -> Self {
        Self {
            map: Default::default(),
            p: START,
            h: START_H,
            state: ExplorerState::Begin,
        }
    }
}
