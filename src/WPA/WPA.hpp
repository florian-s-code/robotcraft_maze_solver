#ifndef WPA_HPP
#define WPA_HPP

#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

class WavefrontPlanner
{
  public:
    struct Coord
    {
        int x, y;
    };

    WavefrontPlanner(int width, int height, const std::vector<int8_t>& occupancy_grid);
    WavefrontPlanner(){};
    void setStart(int x, int y);
    void setEnd(int x, int y);
    void createMap();
    void createPath();
    int* getMap();
    std::vector<Coord> getPath() { return m_path; };

  private:
    int m_width, m_height;
    std::vector<int8_t> m_occupancy_grid;
    std::vector<int> m_WPA_map;
    std::vector<Coord> m_path;

    Coord A[4];
    Coord m_start_cell;
    Coord m_end_cell;

    int8_t getOccupancyValue(Coord cell);
    int getCellValue(Coord cell);
    bool isPathableCell(Coord cell);
    std::vector<Coord> getAdjacentCoords(const Coord& cell);
    void getCoordFromIndex(int cell_index, int& x, int& y);
    void setCellValue(Coord cell, int value);
    Coord getNextPathCell(const Coord& cell, bool direction_X);
};

#endif /* WPA_HPP */
