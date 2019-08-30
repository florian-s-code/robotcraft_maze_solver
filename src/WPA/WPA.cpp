#include "WPA.hpp"

#include <stdexcept>
#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

//DEBUG
#include <iostream>

void WavefrontPlanner::flipOccupancyGrid() {
    std::vector<int8_t> flipped_grid;

    m_occupancy_grid.push_back(-100); // we add an element to occupancy grid to be able to select last

    for(int i = m_height-1; i > -1; i--) {
        //std::cout << "Flipping line " << i << std::endl;
	flipped_grid.insert(flipped_grid.end(), &m_occupancy_grid.at(i*m_width), &m_occupancy_grid.at((i+1)*m_width));
    }


/*//DEBUG
#define LINE_LENGTH 63
    for (int i = 0; i < m_width * m_height; i++)
    {
        if((i+1)/m_width > LINE_LENGTH) {
            std::cout << std::endl;
            break;
        }

        if((i+1)%m_width > LINE_LENGTH) {
            if( (i+1)%m_width == LINE_LENGTH )
                std::cout << std::endl;
            continue;
        }

        if (flipped_grid[i] < 10)
            std::cout << static_cast<int>(flipped_grid[i]) << "  ";
        else if (flipped_grid[i] == 100)
            std::cout << 1 << "  ";
        else
            std::cout << static_cast<int>(flipped_grid[i]) << " ";

        if ((i + 1) % m_width == 0)
            std::cout << std::endl;
    }
//*/
    m_occupancy_grid = flipped_grid;
}

int8_t WavefrontPlanner::getOccupancyValue(WavefrontPlanner::Cell cell)
{
    return m_occupancy_grid[cell.y * m_width + cell.x];
}

int WavefrontPlanner::getCellValue(WavefrontPlanner::Cell cell)
{
    return m_WPA_map[cell.y * m_width + cell.x];
}

WavefrontPlanner::Coord WavefrontPlanner::mapToOdom(WavefrontPlanner::Coord coord)
{
    WavefrontPlanner::Coord new_coord;
    new_coord.x = coord.x + m_origin.x;
    new_coord.y = -coord.y + m_origin.y + m_height*m_resolution;
    return new_coord;
}
/* Convert a coordinate in meters into a int coordinate in cell */
WavefrontPlanner::Cell WavefrontPlanner::coordToCell(WavefrontPlanner::Coord coord)
{
    WavefrontPlanner::Cell new_coord;
    new_coord.x = static_cast<int>(coord.x/m_resolution);
    new_coord.y = static_cast<int>(coord.y/m_resolution);
    return new_coord;
}

WavefrontPlanner::Coord WavefrontPlanner::cellToCord(WavefrontPlanner::Cell cell)
{
    WavefrontPlanner::Coord new_coord;
    new_coord.x = cell.x*m_resolution;
    new_coord.y = cell.y*m_resolution;
    return new_coord;
}
WavefrontPlanner::Coord WavefrontPlanner::odomToMap(WavefrontPlanner::Coord coord) 
{
    WavefrontPlanner::Coord new_coord;
    new_coord.x = coord.x - m_origin.x;
    new_coord.y = -(coord.y - m_origin.y - m_height*m_resolution);
    return new_coord;
}

bool WavefrontPlanner::isPathableCell(WavefrontPlanner::Cell cell)
{
    // out-of-bound check
    if (!(cell.x >= 0 && cell.y >= 0 && cell.x < m_height && cell.y < m_width))
        return false;

    // check if the cell is not an obstacle and if it was not already given a value
    return (getOccupancyValue(cell) == 0);
}

std::vector<WavefrontPlanner::Cell> WavefrontPlanner::getAdjacentCoords(const WavefrontPlanner::Cell& cell)
{
    std::vector<WavefrontPlanner::Cell> coords;

    if(cell.x == 1 && cell.y == 39)
        std::cout << "Adjacent occupancy of (" << cell.x << ", " << cell.y <<"), value : " << getCellValue(cell) << " :" << std::endl;

    // iterate over every adjacent coord and add them to list if
    // - it is not out of bound
    // - it is traversable (occupancy value == 0)
    for (int i = 0; i < 4; i++)
    {
        WavefrontPlanner::Cell next_coord;
        next_coord.x = cell.x + A[i].x;
        next_coord.y = cell.y + A[i].y;

        if(cell.x == 1 && cell.y == 39)
             std::cout << static_cast<int>(getCellValue(next_coord)) << std::endl;

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

void WavefrontPlanner::setCellValue(WavefrontPlanner::Cell cell, int value)
{
    m_WPA_map[cell.y * m_width + cell.x] = value;
}

//constructor
WavefrontPlanner::WavefrontPlanner(int width, int height, float origin_x, float origin_y, float resolution, const std::vector<int8_t>& occupancy_grid)
  : m_width( width ), m_height( height ), m_resolution(resolution)
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

    m_origin = { origin_x, origin_y };

    flipOccupancyGrid();
}

void WavefrontPlanner::init() {
    createMap();
    createPath();
}
/*Set starting position to the position (in odom coordinate plane)*/
void WavefrontPlanner::setStart(float x, float y)
{
    WavefrontPlanner::Cell position = coordToCell( odomToMap( { x, y } ) );
    m_start_cell = position;

    std::cout << "Start x: " << position.x << ", y: " << position.y << std::endl;
}

/*Set end position to the position (in odom coordinate plane)*/
void WavefrontPlanner::setEnd(float x, float y)
{
    WavefrontPlanner::Cell position = coordToCell( odomToMap( { x, y } ) );
    m_end_cell = position;

    std::cout << "End x: " << position.x << ", y: " << position.y << std::endl;
}

void WavefrontPlanner::createMap()
{

    m_WPA_map = std::vector<int>(m_width * m_height, 0);

    // set end cell to 1
    setCellValue(m_end_cell, 1);
    std::deque<WavefrontPlanner::Cell> cells_to_check = { m_end_cell };  // store cells to check, initialized to end
                                                                             // cell

    while (cells_to_check.size() > 0)
    {
        WavefrontPlanner::Cell cell = cells_to_check.front();
        cells_to_check.pop_front();

        if( cell.x == m_start_cell.x && cell.y == m_start_cell.y )
            break;

        int cell_value = getCellValue(cell);
        auto adjacent_cells = getAdjacentCoords(cell);

        if(adjacent_cells.size() < 1) {
	    std::cout << "Create bug : " << cell.x << " " << cell.y <<  ". Value " << cell_value << std::endl;
        }

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

WavefrontPlanner::Cell WavefrontPlanner::getNextPathCell(const Cell& current_cell, bool direction_X)
{
    int current_value = getCellValue(current_cell);
    
    auto adjacent_coords = getAdjacentCoords(current_cell);
    std::vector<WavefrontPlanner::Cell> possible_cells;


    for(int i = 0; i < adjacent_coords.size(); i++) {
        if(  !isPathableCell(adjacent_coords[i]) )
            continue;
        else {
            //we check value first
            if(getCellValue(adjacent_coords[i]) >= current_value)
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
        std::cout << "Bug cell : " << current_cell.x << " " << current_cell.y <<std::endl;
	throw std::logic_error("No path found");
    }
    //we return the first possible candidate, no priority
    return possible_cells[0];
}

void WavefrontPlanner::createPath()
{
    createMap();
    WavefrontPlanner::Cell current_cell = m_start_cell;
    bool direction_X = true; //store if we are currently going on X axis

    std::vector<WavefrontPlanner::Cell> path = {current_cell};

    while(getCellValue(current_cell) != 1) {
        /*while we are not at the end*/
        WavefrontPlanner::Cell next_cell;
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

std::vector<WavefrontPlanner::Coord> WavefrontPlanner::getPath()
{
    std::vector<WavefrontPlanner::Coord> path;
    for(int i = 0; i < m_path.size(); i++) {
        path.push_back( mapToOdom(cellToCord( m_path[i] )) );
    }

    return path;
}



