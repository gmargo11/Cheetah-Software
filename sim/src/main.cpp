/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include "Collision/CollisionPlane.h"
#include "DrawList.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimControlPanel.h"
#include "Simulation.h"
#include "SimulationRunnerHeadless.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <stdio.h>
#include <unistd.h>
#include <thread>

/*!
 * Setup QT and run a simulation
 */
int main(int argc, char *argv[]) {
  install_segfault_handler(nullptr);
  // set up Qt
  QApplication a(argc, argv);

  // open simulator UI
  //SimulationRunnerHeadless simrunner;
  //simrunner.run();
  SimControlPanel panel;
  panel.show();

  // Run Simulator
  panel->ui->miniCheetahButton->setChecked(true);
  panel->ui->simulatorButton->setChecked(true);
  panel.on_startButton_clicked();

  // run the Qt program
  a.exec();

  return 0;
}
