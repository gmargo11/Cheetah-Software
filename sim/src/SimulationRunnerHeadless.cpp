#include "SimulationRunnerHeadless.h"
#include <ControlParameters/ControlParameters.h>
#include <ParamHandler.hpp>
#include <leg_control_command_lcmt.hpp>
#include "JoystickTest.h"


/*!
 * Display an error messagebox with the given text
 */

static void createErrorMessage(const std::string& text) {
  //QMessageBox mb;
  //mb.setText(QString(text.c_str()));
  //mb.exec();
  printf("%s\n", text.c_str());
}

std::string SimulationRunnerHeadless::getDefaultUserParameterFileName() {
  std::string path = getConfigDirectoryPath() + DEFAULT_USER_FILE;
  ParamHandler paramHandler(path);

  if(!paramHandler.fileOpenedSuccessfully()) {
    throw std::runtime_error("Could not open yaml file for default user parameter file: " + path);
  }

  std::string collectionName;
  if(!paramHandler.getString("__collection-name__", collectionName)) {
    throw std::runtime_error("Could not find __collection-name__ parameter in default user parameter file");
  }

  if(collectionName != "user-parameter-file") {
    throw std::runtime_error("default user parameter file has the wrong collection name, should be user-parameter-file");
  }

  std::string fileName;

  if(!paramHandler.getString("file_name", fileName)) {
    throw std::runtime_error("default user parameter file does not have a parameter named file_name");
  }

  return fileName;
}
/*!
 * Init sim window
 */
SimulationRunnerHeadless::SimulationRunnerHeadless()
    : _userParameters("user-parameters"),
      _heightmapLCM(getLcmUrl(255)),
      _pointsLCM(getLcmUrl(255)),
      _indexmapLCM(getLcmUrl(255)),
      _ctrlVisionLCM(getLcmUrl(255)),
      _miniCheetahDebugLCM(getLcmUrl(255))
{

  // attempt to load default user settings.
  _loadedUserSettings = true;

  try {
    _userParameters.defineAndInitializeFromYamlFile(getConfigDirectoryPath() + getDefaultUserParameterFileName());
  } catch (std::runtime_error& ex) {
    _loadedUserSettings = false;
  }

  if(!_loadedUserSettings) {
    printf("[SimulationRunnerHeadless] Failed to load default user settings!\n");
    throw std::runtime_error("Failed to load default user settings!");
  } else {
    // display user settings in qtable if we loaded successfully
    //loadUserParameters(_userParameters);
  }

  // load simulator parameters
  printf("[SimulationRunnerHeadless] Init simulator parameters...\n");
  _parameters.initializeFromYamlFile(getConfigDirectoryPath() +
                                     SIMULATOR_DEFAULT_PARAMETERS);
  if (!_parameters.isFullyInitialized()) {
    printf(
        "[ERROR] Simulator parameters are not fully initialized.  You forgot: "
        "\n%s\n",
        _parameters.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  } else {
    printf("\tsim parameters are all good\n");
  }
  //loadSimulationParameters(_parameters);

  // subscribe mc debug
  _miniCheetahDebugLCM.subscribe("leg_control_data", &SimulationRunnerHeadless::handleSpiDebug, this);
  _miniCheetahDebugLCMThread = std::thread([&](){
   for(;;)
     _miniCheetahDebugLCM.handle();
  });

}

void SimulationRunnerHeadless::handleSpiDebug(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                     const leg_control_data_lcmt *msg) {
  (void)rbuf;
  (void)chan;
  MiniCheetahDebugData ddata;

  u32 idx = 0;
  for(u32 leg = 0; leg < 4; leg++) {
    for(u32 joint = 0; joint < 3; joint++) {
      ddata.p[leg][joint] = msg->q[idx];
      ddata.v[leg][joint] = msg->qd[idx];
      idx++;
    }
  }

  if(_mcDebugWindow.setDebugData(ddata)) {
    MiniCheetahDebugCommand cmd;
    _mcDebugWindow.getDebugCommand(cmd);

    leg_control_command_lcmt lcm_cmd;
    memset(&lcm_cmd, 0, sizeof(leg_control_command_lcmt));
    idx = 0;
    for(u32 leg = 0; leg < 4; leg++) {
      for(u32 joint = 0; joint < 3; joint++) {
        if(cmd.enable[leg][joint]) {
          lcm_cmd.q_des[idx] = cmd.qd[leg][joint];
          lcm_cmd.kp_joint[idx] = cmd.kp[leg][joint];
          lcm_cmd.kd_joint[idx] = cmd.kd[leg][joint];
        }
        idx++;
      }
    }

    _miniCheetahDebugLCM.publish("spi_debug_cmd", &lcm_cmd);
  }

}


SimulationRunnerHeadless::~SimulationRunnerHeadless() {
  delete _simulation;
  delete _interfaceTaskManager;
  delete _robotInterface;
  delete _graphicsWindow;
  //delete ui;
}



/*!
 * Simulation error
 */
void SimulationRunnerHeadless::errorCallback(std::string errorMessage) {
  _state = SimulationRunnerHeadlessState::ERROR; // go to error state
  //updateUiEnable(); // update UI
  createErrorMessage("Simulation Error\n" + errorMessage); // display error dialog
}

/*!
 * Start a simulation/robot run
 */
void SimulationRunnerHeadless::run() {
  // get robot type
  RobotType robotType;

  //if (ui->cheetah3Button->isChecked()) {
  //  robotType = RobotType::CHEETAH_3;
  //} else if (ui->miniCheetahButton->isChecked()) {
  robotType = RobotType::MINI_CHEETAH;
  //} else {
  //  createErrorMessage("Error: you must select a robot");
  //  return;
  //}

  // get run type
  //if (!ui->simulatorButton->isChecked() && !ui->robotButton->isChecked()) {
  //  createErrorMessage(
  //      "Error: you must select either robot or simulation mode");
  //  return;
  //}

  _simulationMode = true;//ui->simulatorButton->isChecked();

  // graphics
  printf("[SimulationRunnerHeadless] Initialize Graphics...\n");
  //_graphicsWindow = new Graphics3D();
  //_graphicsWindow->show();
  //_graphicsWindow->resize(1280, 720);

  if (_simulationMode) {
    // run a simulation

    try {
      printf("[SimulationRunnerHeadless] Initialize simulator...\n");
      _simulation = new Simulation(robotType, _parameters, _userParameters,
        // this will allow the simulation thread to poke us when there's a state change
        [this](){
        //QMetaObject::invokeMethod(this,"update_ui");
      });
      //loadSimulationParameters(_simulation->getSimParams());
      //loadRobotParameters(_simulation->getRobotParams());

      // terrain
      printf("[SimControlParameter] Load terrain...\n");
      _simulation->loadTerrainFile(_terrainFileName);
    } catch (std::exception& e) {
      createErrorMessage("FATAL: Exception thrown during simulator setup\n" + std::string(e.what()));
      throw e;
    }




    // start sim
    _simThread = std::thread(

      // simulation function
      [this]() {

        // error callback function
        std::function<void(std::string)> error_function = [this](std::string str) {
          // Qt will take care of doing the call in the UI event loop
          //QMetaObject::invokeMethod(this, [=]() {
          //  this->errorCallback(str);
          //});
	  this->errorCallback(str);
        };

        try {
          // pass error callback to simulator
          _simulation->runAtSpeed(error_function);
        } catch (std::exception &e) {
          // also catch exceptions
          error_function("Exception thrown in simulation thread: " + std::string(e.what()));
        }

      });

    // graphics start
    //_graphicsWindow->setAnimating(true);
  } else {
    printf("[SimulationRunnerHeadless] Init Robot Interface...\n");
    _interfaceTaskManager = new PeriodicTaskManager;
    _robotInterface =
        new RobotInterface(robotType, _graphicsWindow, _interfaceTaskManager, _userParameters);
    //loadRobotParameters(_robotInterface->getParams());
    _robotInterface->startInterface();
    //_graphicsWindow->setAnimating(true);
  }

  _state = SimulationRunnerHeadlessState::RUNNING;
  //updateUiEnable();
}

/*!
 * Stop the currently running simulation or robot connection
 */
void SimulationRunnerHeadless::stop() {
  if (_simulation) {
    _simulation->stop();
    _simThread.join();
  } else {
    _robotInterface->stopInterface();
  }

  if (_graphicsWindow) {
    _graphicsWindow->setAnimating(false);
    _graphicsWindow->hide();
  }

  delete _interfaceTaskManager;
  delete _robotInterface;
  delete _simulation;
  delete _graphicsWindow;

  _simulation = nullptr;
  _graphicsWindow = nullptr;
  _robotInterface = nullptr;
  _interfaceTaskManager = nullptr;

  _state = SimulationRunnerHeadlessState::STOPPED;
  //updateUiEnable();
}

