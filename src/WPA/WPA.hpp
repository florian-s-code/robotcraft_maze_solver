#ifndef WPA_HPP
#define WPA_HPP

#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

class WavefrontPlanner
{
  public:
    struct Cell
    {
        int x, y;
    };
    struct Coord
    {
        float x, y;
    };

    WavefrontPlanner(int width, int height, float origin_x, float origin_y, float resolution, const std::vector<int8_t>& occupancy_grid);
    WavefrontPlanner(){};
    void setStart(float x, float y);
    void setEnd(float x, float y);
    void createPath();
    int* getMap();
    std::vector<Coord> getPath();

  private:
    int m_width, m_height;
    float m_resolution;
    std::vector<int8_t> m_occupancy_grid;
    std::vector<int> m_WPA_map;
    std::vector<Cell> m_path;

    Cell A[4];
    Coord m_origin;
    Cell m_start_cell;
    Cell m_end_cell;

    void init();
    void createMap();
    void flipOccupancyGrid();
    int8_t getOccupancyValue(Cell cell);
    int getCellValue(Cell cell);
    Coord mapToOdom(Coord coord);
    Coord odomToMap(Coord coord);
    Cell coordToCell(Coord coord);
    Coord cellToCord(Cell cell);
    bool isPathableCell(Cell cell);
    std::vector<Cell> getAdjacentCoords(const Cell& cell);
    void getCoordFromIndex(int cell_index, int& x, int& y);
    void setCellValue(Cell cell, int value);
    Cell getNextPathCell(const Cell& cell, bool direction_X);
};

#endif /* WPA_HPP */
