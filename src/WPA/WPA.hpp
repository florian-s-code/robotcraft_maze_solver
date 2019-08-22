#ifndef WPA_HPP
#define WPA_HPP


#include <stdint.h>
#include <algorithm>
#include <deque>
#include <vector>

class ROSWavefrontPlanner {
  private:
    int m_width, m_height;
    std::vector<int8_t> m_occupancy_grid;
    std::vector<int> m_WPA_map;

    struct Coord {
	int x, y;
    };

    Coord A[4];
    Coord m_start_cell;
    Coord m_end_cell;

    int8_t getOccupancyValue(Coord cell);
    int getCellValue(Coord cell);
    bool checkCoord(Coord cell);
    std::deque<Coord> getAdjacentCells(const Coord& cell);
    void getCoordFromIndex(int cell_index, int& x, int& y);
    void setCellValue(Coord cell, int value);

  public:
    ROSWavefrontPlanner(int width, int height, const int8_t * occupancy_grid);
    void setStart(int x, int y);
    void setEnd(int x, int y);
    void createMap();
    int* getMap();

};

#endif /* WPA_HPP */
