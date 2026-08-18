use crate::map::{Direction, MazeMap, Pos};
use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExplorerState {
    NavigatingToGoal,
    ReturningToStart,
    Done,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExplorerAction {
    TurnLeft,
    TurnRight,
    TurnAround,
    MoveForward,
    Wait,
}

pub struct Explorer<const W: usize, const H: usize> {
    current_pos: Pos,
    start_pos: Pos,
    goal_pos: Pos,
    heading: Direction,
    state: ExplorerState,
    path: Vec<Pos>,
}

impl<const W: usize, const H: usize> Explorer<W, H> {
    pub fn new(start_pos: Pos, goal_pos: Pos, start_heading: Direction) -> Self {
        Self {
            current_pos: start_pos,
            start_pos,
            goal_pos,
            heading: start_heading,
            state: ExplorerState::NavigatingToGoal,
            path: Vec::new(),
        }
    }

    pub fn state(&self) -> ExplorerState {
        self.state
    }

    pub fn current_pos(&self) -> Pos {
        self.current_pos
    }

    pub fn heading(&self) -> Direction {
        self.heading
    }

    pub fn find_next_target(&self, map: &MazeMap<W, H>) -> Option<Pos> {
        // Stage 1: check immediate neighbors - return nothing for dead end
        for &dir in &[
            self.heading,
            self.heading.turn_left(),
            self.heading.turn_right(),
            self.heading.opposite(),
        ] {
            if !map.has_wall(self.current_pos, dir) {
                if let Some(next_pos) = map.neighbor_in_dir(self.current_pos, dir) {
                    if !map.is_visited(next_pos) {
                        return Some(next_pos);
                    }
                }
            }
        }

        // When dead end, do Backtracking to see unvisited cells
        let mut best_target: Option<Pos> = None;
        let mut shortest_len = usize::MAX;

        for x in 0..W {
            for y in 0..H {
                let pos = (x as u8, y as u8);
                if !map.is_visited(pos) {
                    if let Some(path) = map.find_shortest_path(self.current_pos, pos) {
                        if path.len() < shortest_len {
                            shortest_len = path.len();
                            best_target = Some(pos);
                        }
                    }
                }
            }
        }

        best_target
    }

    // Desicion Engine for updating Exploration State & Action
    // Run once per control cycle to update internal coordinates, plan paths
    // and return motion commands [MoveForward, TurnLeft, TurnRight, Wait]
    pub fn step(&mut self, map: &mut MazeMap<W, H>) -> ExplorerAction {
        match self.state {
            ExplorerState::Done => ExplorerAction::Wait,

            ExplorerState::NavigatingToGoal => {
                if self.current_pos == self.goal_pos {
                    self.state = ExplorerState::ReturningToStart;
                    self.path.clear();
                    return self.step(map);
                }

                if let Some(full_path) = map.find_shortest_path(self.current_pos, self.goal_pos) {
                    self.path = full_path.into_iter().skip(1).collect();
                } else {
                    return ExplorerAction::Wait;
                }

                self.execute_next_move()
            }

            ExplorerState::ReturningToStart => {
                // UPDATED! instead of hardcoding here, it'll be passed in constructor via main.rs
                if self.current_pos == self.start_pos {
                    self.state = ExplorerState::Done;
                    return ExplorerAction::Wait;
                }

                if let Some(full_path) = map.find_shortest_path(self.current_pos, self.start_pos) {
                    self.path = full_path.into_iter().skip(1).collect();
                } else {
                    return ExplorerAction::Wait;
                }

                self.execute_next_move()
            }
        }
    }

    fn execute_next_move(&mut self) -> ExplorerAction {
        let Some(&next_cell) = self.path.first() else {
            return ExplorerAction::Wait;
        };

        let required_dir = self.direction_to_neighbor(self.current_pos, next_cell);

        match required_dir {
            Some(dir) if dir == self.heading => {
                self.path.remove(0);
                self.current_pos = next_cell;
                ExplorerAction::MoveForward
            }
            Some(dir) if dir == self.heading.turn_left() => {
                self.heading = self.heading.turn_left();
                ExplorerAction::TurnLeft
            }
            Some(dir) if dir == self.heading.turn_right() => {
                self.heading = self.heading.turn_right();
                ExplorerAction::TurnRight
            }
            Some(_) => {
                self.heading = self.heading.opposite();
                ExplorerAction::TurnAround
            }
            None => ExplorerAction::Wait,
        }
    }

    /// Helper to find the cardinal direction
    fn direction_to_neighbor(&self, from: Pos, to: Pos) -> Option<Direction> {
        let (fx, fy) = (from.0 as i16, from.1 as i16);
        let (tx, ty) = (to.0 as i16, to.1 as i16);

        let dx = tx - fx;
        let dy = ty - fy;

        match (dx, dy) {
            (0, 1) => Some(Direction::North),
            (1, 0) => Some(Direction::East),
            (0, -1) => Some(Direction::South),
            (-1, 0) => Some(Direction::West),
            _ => None, // Not directly adjacent
        }
    }
}
