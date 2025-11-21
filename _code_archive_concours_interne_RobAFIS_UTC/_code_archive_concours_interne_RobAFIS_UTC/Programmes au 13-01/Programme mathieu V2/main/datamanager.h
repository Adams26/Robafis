#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <ArduinoSTL.h>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include "MeMegaPi.h" // Bibliothèque Makeblock pour la carte MegaPi
#include "Wire.h"      // Bibliothèque pour la communication I2C

struct Movement {
    int point;            
    bool columns_first;   
    int dx;              
    int dy;              
    bool needs_first_rotation;
};

struct DepositMovement {
    std::string color;
    int target_row;
    bool needs_back_maneuver;
    int row_displacement;
};

class DataManager {
    private:
        std::map<std::string, int> color_positions;
        const std::vector<std::string> colors = {"B", "J", "N", "V", "R"};
        std::vector<std::string> frame_list;
        std::vector<int> converted_frame_list;
        std::vector<COLORTYPES> color_list;
        std::vector<bool> directions_list;
        double current_angle = 0;
        std::string current_case = "A1";
        std::vector<Movement> movements;
        int final_direction;
        int final_position_number;

        void processScenario();
        void caseValueConversion();
        void executeNextDeposit(std::string last_cube_color, std::string next_cube_color);
        void executeFirstDeposit(std::string first_cube_color);
        void doPathTraveling(const std::vector<Movement>& moves, const Movement& final_mov);

    public:
        DataManager() = default;
        ~DataManager() = default;
        
        void findingShortestPath();
        void initRobot();
        std::string processMessage(std::string datagram);
        std::vector<Movement> getMovements();
        Movement getFinalMovement();
        std::vector<bool> getDirections();
        int getFinalPosition();
        int getFinalDirection();
        std::vector<COLORTYPES> getColorList();
        Movement moveToNewColor(COLORTYPES target_color, const std::vector<COLORTYPES>& color_positions, int current_position, int current_direction);
};

#endif
