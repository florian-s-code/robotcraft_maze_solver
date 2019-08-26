#include "WPA.hpp"

#include <stdexcept>
#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

//DEBUG
#include <iostream>


int8_t WavefrontPlanner::getOccupancyValue(WavefrontPlanner::Coord cell)
{
    return m_occupancy_grid[cell.y * m_width + cell.x];
}

int WavefrontPlanner::getCellValue(WavefrontPlanner::Coord cell)
{
    return m_WPA_map[cell.y * m_width + cell.x];
}

bool WavefrontPlanner::isPathableCell(WavefrontPlanner::Coord cell)
{
    // out-of-bound check
    if (!(cell.x >= 0 && cell.y >= 0 && cell.x < m_height && cell.y < m_width))
        return false;

    // check if the cell is not an obstacle and if it was not already given a value
    return (getOccupancyValue(cell) == 0);
}

std::vector<WavefrontPlanner::Coord> WavefrontPlanner::getAdjacentCoords(const WavefrontPlanner::Coord& cell)
{
    std::vector<WavefrontPlanner::Coord> coords;

    // iterate over every adjacent coord and add them to list if
    // - it is not out of bound
    // - it is traversable (occupancy value == 0)
    // - they don't already have a value
    for (int i = 0; i < 4; i++)
    {
        WavefrontPlanner::Coord next_coord;
        next_coord.x = cell.x + A[i].x;
        next_coord.y = cell.y + A[i].y;

        if (isPathableCell(next_coord) ) //check bound and occupancy
        {
            coords.push_back(next_coord);  // add to the end of the grid
        }
    }

    return coords;
}


void WavefrontPlanner::getCoordFromIndex(int cell_index, int& x, int& y)
{
    x = cell_index % m_width;
    y = cell_index / m_width;
}

void WavefrontPlanner::setCellValue(WavefrontPlanner::Coord cell, int value)
{
    m_WPA_map[cell.y * m_width + cell.x] = value;
}

WavefrontPlanner::WavefrontPlanner(int width, int height, const std::vector<int8_t>& occupancy_grid)
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

void WavefrontPlanner::setStart(int x, int y)
{
    m_start_cell.x = x;
    m_start_cell.y = y;
}

void WavefrontPlanner::setEnd(int x, int y)
{
    m_end_cell.x = x;
    m_end_cell.y = y;
}

void WavefrontPlanner::createMap()
{
    // set end cell to 1
    setCellValue(m_end_cell, 1);
    std::deque<WavefrontPlanner::Coord> cells_to_check = { m_end_cell };  // store cells to check, initialized to end
                                                                             // cell

    while (cells_to_check.size() > 0)
    {
        WavefrontPlanner::Coord cell = cells_to_check.front();
        cells_to_check.pop_front();

        int cell_value = getCellValue(cell);
        auto adjacent_cells = getAdjacentCoords(cell);

        for (int i = 0; i < adjacent_cells.size(); i++)
        {
            if(getCellValue(adjacent_cells[i]) == 0) { //make sure it has no value yet
                setCellValue(adjacent_cells[i], cell_value + 1);
                cells_to_check.push_back(adjacent_cells[i]);
            }
        }
    }
}

int* WavefrontPlanner::getMap()
{
    return m_WPA_map.data();
}

WavefrontPlanner::Coord WavefrontPlanner::getNextPathCell(const Coord& current_cell, bool direction_X)
{
    int current_value = getCellValue(current_cell);
    
    auto adjacent_coords = getAdjacentCoords(current_cell);
    std::vector<WavefrontPlanner::Coord> possible_cells;


    for(int i = 0; i < adjacent_coords.size(); i++) {
        if(  !isPathableCell(adjacent_coords[i]) )
            continue;
        else {
            //we check value first
            if(getCellValue(adjacent_coords[i]) > current_value)
                continue;
            //if the cell is in the direction we are travelling, we use it
	    if(adjacent_coords[i].x != current_cell.x && direction_X)
    	        return adjacent_coords[i];
	    if(adjacent_coords[i].y != current_cell.y && !direction_X)
    	        return adjacent_coords[i];
	    //otherwise we just add it as a candidate
            possible_cells.push_back(adjacent_coords[i]);
        }
    }
    if(possible_cells.size() < 1) {
	throw std::logic_error("No path found");
    }
    //we return the first possible candidate, no priority
    return possible_cells[0];
}

void WavefrontPlanner::createPath()
{
    WavefrontPlanner::Coord current_cell = m_start_cell;
    bool direction_X = true; //store if we are currently going on X axis

    std::vector<WavefrontPlanner::Coord> path = {current_cell};

    while(getCellValue(current_cell) != 1) {
        /*while we are not at the end*/
        WavefrontPlanner::Coord next_cell;
        try {
            next_cell = getNextPathCell(current_cell, direction_X);
        } catch(std::logic_error) {
	    break;
        }
        //update the direction of travel
        if(next_cell.x != current_cell.x) {
	     direction_X = true;
        }
        else {
            direction_X = false;
        }

        path.push_back(next_cell);
        current_cell = next_cell;
    }

    path.push_back(m_end_cell); //add the last cell

    m_path = path;

}





