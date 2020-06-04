/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"
#include "state_estimator_lcmt.hpp"
#include "StateEstimatorContainer.h"


/*!
 * Dummy State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class DummyStateEstimatorContainer : public StateEstimatorContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   */
  DummyStateEstimatorContainer(StateEstimate<T>* stateEstimate) {
    _data.result = stateEstimate;
  }

  /*!
   * Run all estimators
   */
  void run(CheetahVisualization* visualization = nullptr) {
    ;
  }

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }

  /*!
   * Set the contact phase
   */
  void setContactPhase(Vec4<T>& phase) { 
    ;
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    ;
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    ;
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    ;
  }

  ~DummyStateEstimatorContainer() {
    ;
  }

 private:
  StateEstimatorData<T> _data;
  std::vector<GenericEstimator<T>*> _estimators;
  Vec4<T> _phase;
};

#endif  // PROJECT_STATEESTIMATOR_H
