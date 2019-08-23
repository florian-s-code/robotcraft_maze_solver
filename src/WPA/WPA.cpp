#include "WPA.hpp"

#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>
int8_t ROSWavefrontPlanner::getOccupancyValue(ROSWavefrontPlanner::Coord cell)
{
    return m_occupancy_grid[cell.y * m_width + cell.x];
}

int ROSWavefrontPlanner::getCellValue(ROSWavefrontPlanner::Coord cell)
{
    return m_WPA_map[cell.y * m_width + cell.x];
}

bool ROSWavefrontPlanner::checkCoord(ROSWavefrontPlanner::Coord cell)
{
    // out-of-bound check
    if (!(cell.x >= 0 && cell.y >= 0 && cell.x < m_height && cell.y < m_width))
        return false;

    // check if the cell is not an obstacle and if it was not already given a value
    return (getOccupancyValue(cell) == 0 && getCellValue(cell) == 0);
}

std::deque<ROSWavefrontPlanner::Coord> ROSWavefrontPlanner::getAdjacentCells(const ROSWavefrontPlanner::Coord& cell)
{
    std::deque<ROSWavefrontPlanner::Coord> cells;

    // iterate over every adjacent cell and add them to list if
    // - it is not out of bound
    // - it is traversable (occupancy value == 0)
    // - they don't already have a value
    for (int i = 0; i < 4; i++)
    {
        ROSWavefrontPlanner::Coord next_cell;
        next_cell.x = cell.x + A[i].x;
        next_cell.y = cell.y + A[i].y;

        if (checkCoord(next_cell))
        {
            cells.push_back(next_cell);  // add to the end of the grid
        }
    }

    return cells;
}

void ROSWavefrontPlanner::getCoordFromIndex(int cell_index, int& x, int& y)
{
    x = cell_index % m_width;
    y = cell_index / m_width;
}

void ROSWavefrontPlanner::setCellValue(ROSWavefrontPlanner::Coord cell, int value)
{
    m_WPA_map[cell.y * m_width + cell.x] = value;
}

ROSWavefrontPlanner::ROSWavefrontPlanner(int width, int height, const std::vector<int8_t>& occupancy_grid)
  : m_width(width), m_height(height)
{
    // copy the occupancy_grid into the object
    m_occupancy_grid = std::vector<int8_t>(occupancy_grid.begin(), occupancy_grid.end());

    // create empty map for storing algorithm results
    m_WPA_map = std::vector<int>(m_width * m_height, 0);

    // create A matrix that stores coord of adjacent cells
    A[0].x = 1;
    A[0].y = 0;
    A[1].x = 0;
    A[1].y = 1;
    A[2].x = -1;
    A[2].y = 0;
    A[3].x = 0;
    A[3].y = -1;

    // default start and end
    m_start_cell.x = 0;
    m_start_cell.y = 0;
    m_end_cell.x = m_width - 1;
    m_end_cell.y = m_height - 1;
}

void ROSWavefrontPlanner::setStart(int x, int y)
{
    m_start_cell.x = x;
    m_start_cell.y = y;
}

void ROSWavefrontPlanner::setEnd(int x, int y)
{
    m_end_cell.x = x;
    m_end_cell.y = y;
}

void ROSWavefrontPlanner::createMap()
{
    // set end cell to 1
    setCellValue(m_end_cell, 1);
    std::deque<ROSWavefrontPlanner::Coord> cells_to_check = { m_end_cell };  // store cells to check, initialized to end
                                                                             // cell

    while (cells_to_check.size() > 0)
    {
        ROSWavefrontPlanner::Coord cell = cells_to_check.front();
        cells_to_check.pop_front();

        int cell_value = getCellValue(cell);
        auto adjacent_cells = getAdjacentCells(cell);

        for (int i = 0; i < adjacent_cells.size(); i++)
        {
            setCellValue(adjacent_cells[i], cell_value + 1);
            cells_to_check.push_back(adjacent_cells[i]);
        }
    }
}

int* ROSWavefrontPlanner::getMap()
{
    return m_WPA_map.data();
}

void ROSWavefrontPlanner::createPath()
{
}
