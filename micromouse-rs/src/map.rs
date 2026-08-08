use bitflags::bitflags;
use stm32g4xx_hal::serial::Parity;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    North,
    East,
    South,
    West,
}

impl Direction {
    pub fn turn_left(self) -> Self {
        match self {
            Direction::North => Direction::West,
            Direction::West => Direction::South,
            Direction::South => Direction::East,
            Direction::East => Direction::North,
        }
    }

    pub fn turn_right(self) -> Self {
        match self {
            Direction::North => Direction::East,
            Direction::East => Direction::South,
            Direction::South => Direction::West,
            Direction::West => Direction::North,
        }
    }

    pub fn opposite(self) -> Self {
        match self {
            Direction::North => Direction::South,
            Direction::South => Direction::North,
            Direction::East  => Direction::West,
            Direction::West  => Direction::East,
        }
    }

    pub fn to_wall(self) -> Walls {
        match self {
            Direction::North => Walls::NORTH,
            Direction::East  => Walls::EAST,
            Direction::South => Walls::SOUTH,
            Direction::West  => Walls::WEST,
        }
    }
}

bitflags! {
    /// bitmasking Wall defines as a single u8 (1 byte)
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Walls: u8 {
        const NORTH = 0b0001;
        const EAST  = 0b0010;
        const SOUTH = 0b0100;
        const WEST  = 0b1000;
    }
    
}

pub type Pos = (u8, u8);

pub struct MazeMap<const WIDTH: usize, const HEIGHT: usize> {
    // arrays of array - 2D matrix
    walls: [[Walls; HEIGHT]; WIDTH],
    visited: [[bool; HEIGHT]; WIDTH]
}

impl<const WIDTH: usize, const HEIGHT: usize> MazeMap<WIDTH, HEIGHT> {
    pub fn new() -> Self {
        Self {
            walls: [[Walls::empty(); HEIGHT]; WIDTH],
            visited: [[false; HEIGHT]; WIDTH],
        }
    }

    pub fn mark_visited(&mut self, pos: Pos) {
        self.visited[pos.0 as usize][pos.1 as usize] = true;
    }

    pub fn is_visited(&mut self, pos: Pos) -> bool {
        self.visited[pos.0 as usize][pos.1 as usize]
    }

    pub fn completion_percentage(&self) -> f32 {
        let total = (WIDTH * HEIGHT) as f32;
        let mut count = 0;
        for x in 0..WIDTH {
            for y in 0..HEIGHT {
                if self.visited[x][y] {
                    count += 1;
                }
            }
        }
        (count as f32 / total) * 100.0
    }

    pub fn neighbor_in_dir(&self, pos: Pos, dir: Direction) -> Option<Pos> {
        let (x, y) = (pos.0 as i16, pos.1 as i16);
        let (nx, ny) = match dir {
            Direction::North => (x, y + 1),
            Direction::East  => (x + 1, y),
            Direction::South => (x, y - 1),
            Direction::West  => (x - 1, 1),
        };

        if nx >= 0 && nx < WIDTH as i16 && ny >= 0 && ny < HEIGHT as i16 {
            Some((nx as u8, ny as u8))
        } else {
            None
        }
    }

    pub fn set_wall(&mut self, pos: Pos, dir: Direction) {
        let (x, y) = (pos.0 as usize, pos.1 as usize);
        self.walls[x][y] |= dir.to_wall();

        if let Some(neighbor) = self.neighbor_in_dir(pos, dir) {
            let (nx, ny) = (neighbor.0 as usize, neighbor.1 as usize);
            self.walls[nx][ny] |= dir.opposite().to_wall();
        }
    }

    pub fn has_wall(&self, pos: Pos, dir: Direction) -> bool {
        let (x, y) = (pos.0 as usize, pos.1 as usize);
        self.walls[x][y].contains(dir.to_wall())
    }

    pub fn BFS(&self, start: Pos, goal: Pos) -> Option<alloc::vec::Vec<Pos>> {
        use alloc::collections::VecDeque;
        use alloc::vec::Vec;

        if start == goal {
            let mut path = Vec::new();
            path.push(start);
            return Some(path);
        }

        let mut queue = VecDeque::new();
        let mut parents = [[None; HEIGHT]; WIDTH];
        let mut visited = [[false; HEIGHT]; WIDTH];
    
        queue.push_back(start);
        visited[start.0 as usize][start.1 as usize] = true;

        let mut found = false;

        while let Some(curr) = queue.pop_front() {
            if curr == goal {
                found = true;
                break;
            }

            for &dir in &[Direction::North, Direction::East, Direction::South, Direction::West] {
                if !self.has_wall(curr, dir) {
                    if let Some(next) = self.neighbor_in_dir(curr, dir) {
                        let (nx, ny) = (next.0 as usize, next.1 as usize);
                        if !visited[nx][ny] {
                            visited[nx][ny] = true;
                            parents[nx][ny] = Some(curr);
                            queue.push_back(next);
                        }
                    }
                }
            }
        }
    
        if !found {
            return None;
        }

        let mut path = Vec::new();
        let mut curr = goal;
        path.push(curr);

        while curr != start {
            if let Some(parent) = parents[curr.0 as usize][curr.1 as usize] {
                path.push(parent);
                curr = parent;
            } else {
                break;
            }
        }
    
        path.reverse();
        Some(path)
    
    }
}



