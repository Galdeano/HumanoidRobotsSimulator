#ifndef H_Setup
#define H_Setup

#include <string>

struct SimuConfig {
    int play_csv;
    int zmp_filtering;
    int zmp_feedback;
    int old_zmp;
    int save_data_long;
    int save_data_quick;
    int save_data_quick_temp;
    int file_motor;
    int network;
    int local;
    int replay;
    int Tasks;
    int file_human;
    int file_hoap;
    int file_walk;
    int Light;
    int shadow;
    int colorsGL;
    int materials;
    int mathGL;
    int GroundResolution;
    int oritrunk;
    int oritrunk3;
    int LoadObj;
    int Scenarios;
    int Trajectories;
    int Ext_traj;
    int Video;
    int Suspendu;
    int Visualisation;
    double Te;
    double Dtime;
    int ROBOT_SHERPA;
    int ROBOT_GENERIC;
    int ROBOT_HUMAN;
    int CMD_PD;
    int CMD_DYNAMIC;
    int CMD_TASK;
    int StaticCOM;
    int VisuArticularsLimits;
    int VisuTorquesColor;
    int baseFoot;
    int xContact;
    int yContact;
    int linearDamper;
    int nonLinearDamper;
    int active_scenario;

    bool loadFromFile(const std::string& filepath);
};

extern SimuConfig config;

#ifndef BUILD_SETUP_CPP
// For compatibility, define macros that expand to config fields:
#define play_csv config.play_csv
#define zmp_filtering config.zmp_filtering
#define zmp_feedback config.zmp_feedback
#define old_zmp config.old_zmp
#define save_data_long config.save_data_long
#define save_data_quick config.save_data_quick
#define save_data_quick_temp config.save_data_quick_temp
#define file_motor config.file_motor
#define network config.network
#define local config.local
#define replay config.replay
#define Tasks config.Tasks
#define file_human config.file_human
#define file_hoap config.file_hoap
#define file_walk config.file_walk
#define Light config.Light
#define shadow config.shadow
#define colorsGL config.colorsGL
#define materials config.materials
#define mathGL config.mathGL
#define GroundResolution config.GroundResolution
#define oritrunk config.oritrunk
#define oritrunk3 config.oritrunk3
#define LoadObj config.LoadObj
#define Scenarios config.Scenarios
#define Trajectories config.Trajectories
#define Ext_traj config.Ext_traj
#define Video config.Video
#define Suspendu config.Suspendu
#define Visualisation config.Visualisation
#define Te config.Te
#define Dtime config.Dtime
#define ROBOT_SHERPA config.ROBOT_SHERPA
#define ROBOT_GENERIC config.ROBOT_GENERIC
#define ROBOT_HUMAN config.ROBOT_HUMAN
#define CMD_PD config.CMD_PD
#define CMD_DYNAMIC config.CMD_DYNAMIC
#define CMD_TASK config.CMD_TASK
#define StaticCOM config.StaticCOM
#define VisuArticularsLimits config.VisuArticularsLimits
#define VisuTorquesColor config.VisuTorquesColor
#define baseFoot config.baseFoot
#define xContact config.xContact
#define yContact config.yContact
#define linearDamper config.linearDamper
#define nonLinearDamper config.nonLinearDamper

// Scenario macro definitions for scenario selection
#define Sc1 (config.active_scenario == 1)
#define Sc2 (config.active_scenario == 2)
#define Sc3 (config.active_scenario == 3)
#define Sc4 (config.active_scenario == 4)
#define Sc5 (config.active_scenario == 5)
#define Sc6 (config.active_scenario == 6)
#define Sc7 (config.active_scenario == 7)
#endif

// Generic Robot model joint index definitions
#define RLEG 2
#define LLEG 8
#define SPINE 14
#define RARM 16
#define LARM 19

#endif
