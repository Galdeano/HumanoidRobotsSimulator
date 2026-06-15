#define BUILD_SETUP_CPP
#include "Setup.h"
#include <pugixml.hpp>
#include <iostream>

SimuConfig config;

void init_default_config() {
    config.play_csv = 0;
    config.zmp_filtering = 1;
    config.zmp_feedback = 1;
    config.old_zmp = 0;
    config.save_data_long = 0;
    config.save_data_quick = 1;
    config.save_data_quick_temp = 1;
    config.file_motor = 0;
    config.network = 0;
    config.local = 1;
    config.replay = 0;
    config.Tasks = 1;
    config.file_human = 0;
    config.file_hoap = 0;
    config.file_walk = 1;
    config.Light = 0;
    config.shadow = 1;
    config.colorsGL = 0;
    config.materials = 1;
    config.mathGL = 0;
    config.GroundResolution = 20;
    config.oritrunk = 1;
    config.oritrunk3 = 1;
    config.LoadObj = 1;
    config.Scenarios = 0;
    config.Trajectories = 1;
    config.Ext_traj = 0;
    config.Video = 0;
    config.Suspendu = 0;
    config.Visualisation = 1;
    config.Te = 0.005;
    config.Dtime = 0.0001;
    config.ROBOT_SHERPA = 0;
    config.ROBOT_GENERIC = 1;
    config.ROBOT_HUMAN = 0;
    config.CMD_PD = 0;
    config.CMD_DYNAMIC = 0;
    config.CMD_TASK = 1;
    config.StaticCOM = 0;
    config.VisuArticularsLimits = 0;
    config.VisuTorquesColor = 1;
    config.baseFoot = 13;
    config.xContact = 5;
    config.yContact = 4;
    config.linearDamper = 1;
    config.nonLinearDamper = 0;
    config.active_scenario = 4;
}

bool SimuConfig::loadFromFile(const std::string& filepath) {
    init_default_config();
    
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filepath.c_str());
    if (!result) {
        std::cerr << "Warning: Could not load config file " << filepath << " (" << result.description() << "). Using default settings." << std::endl;
        return false;
    }

    pugi::xml_node root = doc.child("Config");
    if (!root) {
        std::cerr << "Warning: No <Config> root node found in " << filepath << ". Using default settings." << std::endl;
        return false;
    }

    #define PARSE_INT(name) if (root.child(#name)) { name = root.child(#name).text().as_int(); }
    #define PARSE_DOUBLE(name) if (root.child(#name)) { name = root.child(#name).text().as_double(); }

    PARSE_INT(play_csv)
    PARSE_INT(zmp_filtering)
    PARSE_INT(zmp_feedback)
    PARSE_INT(old_zmp)
    PARSE_INT(save_data_long)
    PARSE_INT(save_data_quick)
    PARSE_INT(save_data_quick_temp)
    PARSE_INT(file_motor)
    PARSE_INT(network)
    PARSE_INT(local)
    PARSE_INT(replay)
    PARSE_INT(Tasks)
    PARSE_INT(file_human)
    PARSE_INT(file_hoap)
    PARSE_INT(file_walk)
    PARSE_INT(Light)
    PARSE_INT(shadow)
    PARSE_INT(colorsGL)
    PARSE_INT(materials)
    PARSE_INT(mathGL)
    PARSE_INT(GroundResolution)
    PARSE_INT(oritrunk)
    PARSE_INT(oritrunk3)
    PARSE_INT(LoadObj)
    PARSE_INT(Scenarios)
    PARSE_INT(Trajectories)
    PARSE_INT(Ext_traj)
    PARSE_INT(Video)
    PARSE_INT(Suspendu)
    PARSE_INT(Visualisation)
    PARSE_DOUBLE(Te)
    PARSE_DOUBLE(Dtime)
    PARSE_INT(ROBOT_SHERPA)
    PARSE_INT(ROBOT_GENERIC)
    PARSE_INT(ROBOT_HUMAN)
    PARSE_INT(CMD_PD)
    PARSE_INT(CMD_DYNAMIC)
    PARSE_INT(CMD_TASK)
    PARSE_INT(StaticCOM)
    PARSE_INT(VisuArticularsLimits)
    PARSE_INT(VisuTorquesColor)
    PARSE_INT(baseFoot)
    PARSE_INT(xContact)
    PARSE_INT(yContact)
    PARSE_INT(linearDamper)
    PARSE_INT(nonLinearDamper)
    PARSE_INT(active_scenario)

    #undef PARSE_INT
    #undef PARSE_DOUBLE

    return true;
}
