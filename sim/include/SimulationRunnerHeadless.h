/*!
 * @file SimControlPanel.h
 * @brief QT gui for the simulator
 */

#ifndef SIMULATORRUNNERHEADLESS_H
#define SIMULATORRUNNERHEADLESS_H

#include <thread>
#include "ControlParameters/SimulatorParameters.h"
//#include "Graphics3D.h"
#include "RobotInterface.h"
#include "Simulation.h"

#define DEFAULT_TERRAIN_FILE "/default-terrain.yaml"
#define DEFAULT_USER_FILE "/default-user-parameters-file.yaml"

#include <lcm-cpp.hpp>
#include <src/MiniCheetahDebug.h>
#include <leg_control_data_lcmt.hpp>
#include "rs_pointcloud_t.hpp"
#include "heightmap_t.hpp"
#include "traversability_map_t.hpp"
#include "obstacle_visual_t.hpp"
#include "velocity_visual_t.hpp"

//namespace Ui {
//class SimulationRunnerHeadless;
//}

enum class SimulationRunnerHeadlessState {
  STOPPED,
  RUNNING,
  ERROR
};

class SimulationRunnerHeadless{
//  Q_OBJECT

 public:
  explicit SimulationRunnerHeadless();
  ~SimulationRunnerHeadless();



public slots:
//  void update_ui();
  void errorCallback(std::string errorMessage);

  void run();

  void stop();


 private:

  std::string getDefaultUserParameterFileName();
//  void updateUiEnable();
  bool isStopped() {
    return _state == SimulationRunnerHeadlessState::STOPPED;
  }

  bool isError() {
    return _state == SimulationRunnerHeadlessState::ERROR;
  }

  bool isRunning() {
    return _state == SimulationRunnerHeadlessState::RUNNING;
  }


  std::thread _simThread;
  //bool _started = false;
  SimulationRunnerHeadlessState _state = SimulationRunnerHeadlessState::STOPPED;
//  Ui::SimControlPanel* ui;
  Simulation* _simulation = nullptr;
  PeriodicTaskManager* _interfaceTaskManager = nullptr;
  RobotInterface* _robotInterface = nullptr;
  void* _graphicsWindow = nullptr;
  SimulatorControlParameters _parameters;
  ControlParameters _userParameters;
  bool _simulationMode = false;
  bool _firstStart = true;
  bool _ignoreTableCallbacks = false;
  bool _loadedUserSettings = false;
  std::string _terrainFileName;

  void handleSpiDebug(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const leg_control_data_lcmt* msg);

  lcm::LCM _heightmapLCM;
  lcm::LCM _pointsLCM;
  lcm::LCM _indexmapLCM;
  lcm::LCM _ctrlVisionLCM;
  lcm::LCM _miniCheetahDebugLCM;

  std::thread _pointsLCMThread;
  std::thread _heightmapLCMThread;
  std::thread _indexmapLCMThread;
  std::thread _ctrlVisionLCMThread;
  std::thread _miniCheetahDebugLCMThread;

  MiniCheetahDebug _mcDebugWindow;

};

#endif  // SIMCONTROLPANEL_H
