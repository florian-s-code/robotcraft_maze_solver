
#include <iostream>
#include "WPA.hpp"

int main()
{
    int8_t occupancy_grid[81] = { 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1,
                                  1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1,
                                  1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1 };
    int width = 9;
    int height = 9;
    int start_x = 0;
    int start_y = 0;
    int end_x = 8;
    int end_y = 8;

    auto planner = ROSWavefrontPlanner(width, height, occupancy_grid);
    planner.setEnd(end_x, end_y);
    planner.setStart(start_x, start_y);
    planner.createMap();
    auto map = planner.getMap();

    for (int i = 0; i < width * height; i++)
    {
        if (map[i] < 10)
            std::cout << map[i] << "  ";
        else
            std::cout << map[i] << " ";

        if ((i + 1) % width == 0)
            std::cout << std::endl;
    }
}
