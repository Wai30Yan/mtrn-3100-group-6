use bitflags::bitflags;
use core::{cell, fmt::Write};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;

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
            Direction::East => Direction::West,
            Direction::West => Direction::East,
        }
    }

    pub fn to_wall(self) -> Walls {
        match self {
            Direction::North => Walls::NORTH,
            Direction::East => Walls::EAST,
            Direction::South => Walls::SOUTH,
            Direction::West => Walls::WEST,
        }
    }
}

bitflags! {
    /// bitmasking Wall defines as a single u8 (1 byte)
    /// 8x8 grid only use 64 bytes to store every wall
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
    visited: [[bool; HEIGHT]; WIDTH],
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

    pub fn is_visited(&self, pos: Pos) -> bool {
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
            Direction::East => (x + 1, y),
            Direction::South => (x, y - 1),
            Direction::West => (x - 1, 1),
        };

        if nx >= 0 && nx < WIDTH as i16 && ny >= 0 && ny < HEIGHT as i16 {
            Some((nx as u8, ny as u8))
        } else {
            None
        }
    }

    pub fn set_wall(&mut self, pos: Pos, dir: Direction) {
        let (x, y) = (pos.0 as usize, pos.1 as usize);

        // bitwise OR to set  the wall bit for (x, y)
        self.walls[x][y] |= dir.to_wall();

        if let Some(neighbor) = self.neighbor_in_dir(pos, dir) {
            let (nx, ny) = (neighbor.0 as usize, neighbor.1 as usize);

            // mirror the wall on neighbor's opposite side
            self.walls[nx][ny] |= dir.opposite().to_wall();
        }
    }

    pub fn has_wall(&self, pos: Pos, dir: Direction) -> bool {
        let (x, y) = (pos.0 as usize, pos.1 as usize);
        self.walls[x][y].contains(dir.to_wall())
    }

    pub fn find_shortest_path(&self, start: Pos, goal: Pos) -> Option<alloc::vec::Vec<Pos>> {
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

            for &dir in &[
                Direction::North,
                Direction::East,
                Direction::South,
                Direction::West,
            ] {
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

    pub fn update_from_lidars(
        &mut self,
        current_pos: Pos,
        heading: Direction,
        left_dist: Option<f32>,
        right_dist: Option<f32>,
        front_dist: Option<f32>,
        wall_threshold_m: f32,
    ) {
        self.mark_visited(current_pos);

        if let Some(dist) = front_dist {
            if dist < wall_threshold_m {
                self.set_wall(current_pos, heading);
            }
        }

        if let Some(dist) = left_dist {
            if dist < wall_threshold_m {
                self.set_wall(current_pos, heading.turn_left());
            }
        }

        if let Some(dist) = right_dist {
            if dist < wall_threshold_m {
                self.set_wall(current_pos, heading.turn_right());
            }
        }
    }

    /// Draws 64x64 maze grid on left half of display and completion
    /// percentage/status bar on the right half
    pub fn draw_on_display<D>(&self, display: &mut D, robot_pos: Pos) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        display.clear(BinaryColor::Off)?;

        let cell_size = 64 / WIDTH as i32;

        for x in 0..WIDTH {
            for y in 0..HEIGHT {
                let pos = (x as u8, y as u8);
                let px = x as i32 * cell_size;
                // Invert Y coordinate so (0,0) sits at bottom-left grid
                let py = 63 - ((y as i32 + 1) * cell_size);

                // Marks visited cells (centered dot)
                if self.is_visited(pos) {
                    Pixel(
                        Point::new(px + cell_size / 2, py + cell_size / 2),
                        BinaryColor::On,
                    )
                    .draw(display)?;
                }

                // Draw North Wall
                if self.has_wall(pos, Direction::North) {
                    Line::new(Point::new(px, py), Point::new(px + cell_size, py))
                        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                        .draw(display)?;
                }

                // Draw West Wall
                if self.has_wall(pos, Direction::West) {
                    Line::new(Point::new(px, py), Point::new(px, cell_size + py))
                        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                        .draw(display)?;
                }
            }
        }

        // For Robot's current position/cell
        let rx = robot_pos.0 as i32 * cell_size + 2;
        let ry = 63 - ((robot_pos.1 as i32 + 1) * cell_size) + 2;
        Rectangle::new(
            Point::new(rx, ry),
            Size::new((cell_size - 3) as u32, (cell_size - 3) as u32),
        )
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(display)?;

        // Render Status & Completion percentage on Right Half of OLED (X: 68..128)
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        Text::new("MAP STATUS", Point::new(68, 12), text_style).draw(display)?;

        let percentage = self.completion_percentage();
        let mut buffer: String<32> = String::new();
        let _ = write!(buffer, "Done: {:.0}%", percentage);
        Text::new(buffer.as_str(), Point::new(68, 28), text_style).draw(display)?;

        let bar_width = ((percentage / 100.0) * 50.0) as u32;
        Rectangle::new(Point::new(68, 42), Size::new(52, 10))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)?;
        Rectangle::new(Point::new(69, 43), Size::new(bar_width, 8))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)?;

        Ok(())
    }
}
