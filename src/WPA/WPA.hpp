#ifndef WPA_HPP
#define WPA_HPP

#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

class ROSWavefrontPlanner
{
  public:
    struct Coord
    {
        int x, y;
    };

    ROSWavefrontPlanner(int width, int height, const std::vector<int8_t>& occupancy_grid);
    ROSWavefrontPlanner(){};
    void setStart(int x, int y);
    void setEnd(int x, int y);
    void createMap();
    int* getMap();

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
};

#endif /* WPA_HPP */
