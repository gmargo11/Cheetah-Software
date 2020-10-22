#include <iostream>
//#include <utilities/pretty_print.h>
#include <Utilities/Utilities_print.h>
#include <eigen3/Eigen/Core>
#include <unistd.h>

#include "NeuralMPCLocomotion.h"
#include "NeuralMPC_interface.h"


///////////////
// GAIT
///////////////
NeuralGait::NeuralGait(int nMPC_segments, Vec4<int> offsets_prev, Vec4<int> durations_prev, Vec4<int> offsets,
    Vec4<int> durations, Vec4<int> offsets_next, Vec4<int> durations_next, const std::string &name) :
  _offsets_prev(offsets_prev.array()),
  _durations_prev(durations_prev.array()),
  _offsets(offsets.array()),
  _durations(durations.array()),
  _offsets_next(offsets_next.array()),
  _durations_next(durations_next.array()),
  _nIterations(nMPC_segments)
{
  _mpc_table = new int[nMPC_segments * 4];

  _offsetsPrevFloat = offsets_prev.cast<float>() / (float) nMPC_segments;
  _durationsPrevFloat = durations_prev.cast<float>() / (float) nMPC_segments;
  _offsetsFloat = offsets.cast<float>() / (float) nMPC_segments;
  _durationsFloat = durations.cast<float>() / (float) nMPC_segments;
  _offsetsNextFloat = offsets_next.cast<float>() / (float) nMPC_segments;
  _durationsNextFloat = durations_next.cast<float>() / (float) nMPC_segments;
  /*
  std::cout << "NeuralGait " << name << "\n";
  std::cout << "nMPC_segments    : " << _nIterations << "\n";
  std::cout << "offsets (int)    : " << _offsets.transpose() << "\n";
  std::cout << "durations (int)  : " << _durations.transpose() << "\n";
  std::cout << "offsets (float)  : " << _offsetsFloat.transpose() << "\n";
  std::cout << "durations (float): " << _durationsFloat.transpose() << "\n";
  std::cout << "\n\n";
  */
  (void)name;
  
  // do not allow wrapping
  /*
  for(int i = 0; i < 4; i++){
    if(_offsets[i] + _durations[i] > nMPC_segments){ // wrapping
        _durations[i] = nMPC_segments - _offsets[i];
        _durationsFloat[i] = 1.0 - _offsetsFloat[i];
    }
    if(_offsets_next[i] + _durations_next[i] > nMPC_segments){
	_durations_next[i] = nMPC_segments - _offsets_next[i];
	_durationsNextFloat[i] = 1.0 - _offsetsNextFloat[i];
    }
  }
  */
  
  _stance = _durations;
  Vec4<int> segments_list(nMPC_segments, nMPC_segments, nMPC_segments, nMPC_segments);
  _swing = segments_list - durations;
  //_swing = _offsets_next - ((offsets + durations) % nMPC_segments)
  

}


NeuralGait::~NeuralGait() {
  delete[] _mpc_table;
}


Vec4<float> NeuralGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(_phase > _offsetsFloat[i] and _phase <= _offsetsFloat[i] + _durationsFloat[i]){ // in stance, back half
        if(_offsets[i] + _durations[i] < _nIterations){ // no overlap
            progress[i] = (_phase - _offsetsFloat[i]) / _durationsFloat[i];
        }
        else if(_offsets_next[i] + _durations_next[i] >= _nIterations){ // overlap
	          progress[i] = (_phase - _offsetsFloat[i]) / ((1. - _offsetsFloat[i]) + (_durationsNextFloat[i] + _offsetsNextFloat[i] - 1.));
        }
        else{ // cycle ends, but no overlap
            progress[i] = (_phase - _offsetsFloat[i]) / (1. - _offsetsFloat[i]);
        }
      }
    else if(_offsets[i] + _durations[i] > _nIterations and _phase < (_durationsFloat[i] + _offsetsFloat[i] - 1.)){ // in stance, front half
       if(_offsets_prev[i] + _durations_prev[i] < _nIterations){ // no overlap from previous
            progress[i] = _phase / (_durationsFloat[i] + _offsetsFloat[i] - 1.);
        }
        else{ // overlap from previous
            progress[i] = (_phase + (1. - _offsetsPrevFloat[i])) / ((_durationsFloat[i] + _offsetsFloat[i] - 1.) + (1. - _offsetsPrevFloat[i]));
        }
    }
    else{
      progress[i] = 0;
    }

  }

  return progress.matrix();
}

Vec4<float> NeuralGait::getSwingState() {
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  //Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(_phase >= _offsetsFloat[i] + _durationsFloat[i]){ // in swing, back half
        if(_offsets_next[i] + _durations_next[i] <= _nIterations){ // no overlap, next cycle
            progress[i] = (_phase - _offsetsFloat[i] - _durationsFloat[i]) / (1. - _offsetsFloat[i] - _durationsFloat[i] + _offsetsNextFloat[i]);
        }
	//else if(offsets_next[i] == 0){
        //    progress[i] = (_phase - _offsetsFloat[i] - _durationsFloat[i]) / (1 - _offsetsFloat[i] - _durationsFloat[i] + _offsetsNextFloat
	//}
        else{ // overlap, next cycle
            progress[i] = (_phase - _offsetsFloat[i] - _durationsFloat[i]) / ((1. - _offsetsFloat[i] - _durationsFloat[i])); // swing ends at start of next cycle
        }
      }
    else if(_phase < _offsetsFloat[i] and _offsets[i] + _durations[i] > _nIterations and _phase >= (_offsetsFloat[i] + _durationsFloat[i] - 1.)){ // in swing, middle part 
      progress[i] = (_phase + (1 - _offsetsFloat[i] - _durationsFloat[i])) / (1. - _durationsFloat[i]);
    }
    else if(_phase < _offsetsFloat[i] and _offsets[i] + _durations[i] <= _nIterations){ // in swing, front half
       if(_offsets_prev[i] + _durations_prev[i] < _nIterations){ // no overlap from previous cycle
            progress[i] = (_phase + (1. - _offsetsPrevFloat[i] - _durationsPrevFloat[i])) / ((_offsetsFloat[i]) + (1. - _offsetsPrevFloat[i] - _durationsPrevFloat[i]));
        }
        else{ // overlap, swing begins at start of cycle
            progress[i] = _phase / (_offsetsFloat[i]);
        }
    }
    else{
      progress[i] = 0;
    }

  }

  
  /*
  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])

    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }
  */

  return progress.matrix();
}

int* NeuralGait::mpc_gait() {

  for(int i = _nIterations - _iteration - 1; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets_next;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations_next[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
  }

  for(int i = 0; i < _nIterations - _iteration - 1; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
  }

   
  for(int i = 0; i < 4; i++){
    // update swing times
    if(_iteration < _offsets[i]){ //old gait swing time
      if(_offsets[i] + _durations[i] > _nIterations){ // cycling
        _swing[i] = _nIterations - _durations[i];
      } else if(_offsets_prev[i] + _durations_prev[i] > _nIterations){ // swing started at cycle start
        _swing[i]  = _offsets[i];
      } else{ // swing connected from previous cycle
        _swing[i] = _offsets[i] + (_nIterations - _durations_prev[i] - _offsets_prev[i]);
      }
    }
    else{ // new gait swing time
      if(_offsets[i] + _durations[i] > _nIterations){ // cycling -- just the front part of next swing
        if(_offsets_next[i] + _durations_next[i] > _nIterations){ // next also cycles
          _swing[i] = _nIterations - _durations_next[i]; // just the swing time
        }
        else{
          _swing[i] = _offsets_next[i];
        }
      }
      else{ // no cycling - end of this swing plus front of next swing
        if(_offsets_next[i] + _durations_next[i] > _nIterations){ // next cycles
          _swing[i] = _nIterations - _offsets[i] - _durations[i];
        }
        else{
          _swing[i] = _offsets_next[i] + (_nIterations - _offsets[i] - _durations[i]);
        }
      }
      }

    //update stance times
    if(_iteration > _offsets[i] + _durations[i]){ // in next swing
      if(_offsets_next[i] + _durations_next[i] > _nIterations){ // next cycles
          _stance[i] = _offsets_next[i] + _durations_next[i] - _nIterations;
      } else{ // next doesn't cycle
        _stance[i] = _durations_next[i];
      }
      //}
    } else{ // before next swing
      if(_offsets[i] + _durations[i] > _nIterations){ // current cycles
        if(_offsets_prev[i] + _durations_prev[i] >= _nIterations){ // prev cycles
          _stance[i] = (_offsets[i] + _durations[i] - _nIterations) + (_nIterations - _offsets_prev[i]);
        }
        else{ // prev doesn't cycle
          _stance[i] = (_offsets[i] + _durations[i] - _nIterations);
        }
      } else{ //current doesn't cycle
        _stance[i] = _durations[i];
      }
    }
  }
  
  /*
  std::cout << "offsets_prev: " << _offsets_prev[0] << ", " <<_offsets_prev[1] << ", " << _offsets_prev[2] << ", " << _offsets_prev[3] <<  "\n";
  std::cout << "durations_prev: " << _durations_prev[0] << ", " <<_durations_prev[1] << ", " << _durations_prev[2] << ", " << _durations_prev[3] <<  "\n";
  std::cout << "offsets: " << _offsets[0] << ", " <<_offsets[1] << ", " << _offsets[2] << ", " << _offsets[3] <<  "\n";
  std::cout << "durations: " << _durations[0] << ", " <<_durations[1] << ", " << _durations[2] << ", " << _durations[3] <<  "\n";
  std::cout << "offsets_next: " << _offsets_next[0] << ", " <<_offsets_next[1] << ", " << _offsets_next[2] << ", " << _offsets_next[3] <<  "\n";
  std::cout << "durations_next: " << _durations_next[0] << ", " <<_durations_next[1] << ", " << _durations_next[2] << ", " << _durations_next[3] <<  "\n";
  std::cout << "iterations: " << _iteration << "\n";
  
  std::cout << "mpc_table: \n ";
  for(int i=0; i<_nIterations; i++){
	  for(int j=0; j<4; j++){
		  std::cout << _mpc_table[i*4+j] << " ";
	  }
          std::cout << "\n";
  }
  std::cout << "stances: " << _stance[0] << ", " <<_stance[1] << ", " << _stance[2] << ", " << _stance[3] <<  "\n";
  std::cout << "swings: " << _swing[0] << ", " <<_swing[1] << ", " << _swing[2] << ", " << _swing[3] <<  "\n";
  */
  /*for(int i=0; i < 4; i++){
    if(_stance[i] >= 10){
      std::cout << "Big stance value!";
    }
    if(_swing[i] >= 10){
      std::cout << "Big swing value!";
    }
  }*/

  return _mpc_table;
}

void NeuralGait::setIterations(int iterationsPerMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
  //std::cout << "iteration " << _iteration << " phase " << _phase << "\n";
}


////////////////////
// Controller
////////////////////

NeuralMPCLocomotion::NeuralMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10), Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5), Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5), "Trotting"),
  bounding(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10), Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3), Vec4<int>(5,5,0,0), Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength,  Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4), Vec4<int>(0,0,0,0), Vec4<int>(4,4,4,4),"Pronking"),
  galloping(horizonLength,  Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3), Vec4<int>(0,2,7,9), Vec4<int>(3,3,3,3),"Galloping"),
  standing(horizonLength,  Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10), Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),"Standing"),
  trotRunning(horizonLength,  Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3), Vec4<int>(0,5,5,0), Vec4<int>(3,3,3,3),"Trot Running"),
  cyclic(horizonLength,  Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),Vec4<int>(0,2,4,6),Vec4<int>(8,8,8,8), Vec4<int>(0,2,4,6), Vec4<int>(8,8,8,8),"Cyclic Walk")
//  _neuralLCM(getLcmUrl(255))
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  //printf("[Neural MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
  //    dt, iterationsBetweenMPC, dtMPC);
  neural_setup_problem(dtMPC, horizonLength, 0.4, 120);
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  
  //_neuralLCM.subscribe("rl_gait_action", &NeuralMPCLocomotion::handleActionLCM, this);
  //_neuralLCMThread = std::thread(&NeuralMPCLocomotion::neuralLCMThread, this);

  iterationsBetweenMPC_cmd = iterationsBetweenMPC;
  // initialization
  for(int i=0; i<3; i++){ vel_act[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_prev[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_next[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = 0.0;}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = 0.0;} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = 0.0;}
  footswing_height_act = 0.06;
  offsets_act_prev[0] = 0; offsets_act_prev[1] = 0; offsets_act_prev[2] = 0; offsets_act_prev[3] = 0;
  for(int i=0; i<4; i++){ durations_act_prev[i] = 10; }
  offsets_act[0] = 0; offsets_act[1] = 0; offsets_act[2] = 0; offsets_act[3] = 0;
  for(int i=0; i<4; i++){ durations_act[i] = 10; }
  offsets_act_next[0] = 0; offsets_act_next[1] = 5; offsets_act_next[2] = 5; offsets_act_next[3] = 0;
  for(int i=0; i<4; i++){ durations_act_next[i] = 5; }
   
}

void NeuralMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  rpy_des.setZero();
  v_rpy_des.setZero();

  _policyRecieved = 0;
  // initialization
  for(int i=0; i<3; i++){ vel_act[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_prev[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_next[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = 0.0;}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = 0.0;} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = 0.0;}
  footswing_height_act = 0.06;
  offsets_act_prev[0] = 0; offsets_act_prev[1] = 0; offsets_act_prev[2] = 0; offsets_act_prev[3] = 0;
  for(int i=0; i<4; i++){ durations_act_prev[i] = 10; }
  offsets_act[0] = 0; offsets_act[1] = 0; offsets_act[2] = 0; offsets_act[3] = 0;
  offsets_act_next[0] = 0; offsets_act_next[1] = 5; offsets_act_next[2] = 5; offsets_act_next[3] = 0;
  for(int i=0; i<4; i++){ durations_act[i] = 10; }
  for(int i=0; i<4; i++){ durations_act_next[i] = 5; }
   
  iterationsBetweenMPC_cmd = iterationsBetweenMPC;
}
void NeuralMPCLocomotion::_UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,
    const DMat<float> & height_map){


    (void)body_pos;
    (void)height_map;
    //Vec3<float> local_pf = foot - body_pos;

    //int row_idx_half = height_map.rows()/2;
    //int col_idx_half = height_map.cols()/2;

    //int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    //int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;

    //int x_idx_selected = x_idx;
    //int y_idx_selected = y_idx;

    //std::cout << local_pf[0] << " " << local_pf[1];
    //std::cout << " " << x_idx << " " << y_idx << "\n";

    foot[2] = 0;
    /*
    if(0 > x_idx or x_idx >= height_map.rows() or 0 > y_idx or y_idx >= height_map.cols()){
    	foot[2] = 0;
	//std::cout << "step planned beyond heightmap! \n";
    }
    else{
    	//_IdxMapChecking(x_idx, y_idx, x_idx_selected, y_idx_selected, idx_map);

    	// Get foot position from neural planner



    	//foot[0] = (x_idx_selected - row_idx_half)*grid_size + body_pos[0];
    	//foot[1] = (y_idx_selected - col_idx_half)*grid_size + body_pos[1];
    	foot[2] = height_map(x_idx_selected, y_idx_selected);
    	//std::cout << "Target height:" << foot[2] << "\n";
    }*/

}

float NeuralMPCLocomotion::getFootHeight(Vec3<float>& foot, const Vec3<float>& body_pos,
        const DMat<float> & height_map){

    //std::cout << "body_pos " << body_pos << "\n";
    //std::cout << "foot " << foot << "\n";
    Vec3<float> local_pf = foot - body_pos;
    //std::cout << "local_pf " << local_pf << "\n";

    int row_idx_half = height_map.rows()/2;
    int col_idx_half = height_map.rows()/2;

    int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;

    
    (void)x_idx;
    (void)y_idx;
    //std::cout << "x_idx, y_idx " << x_idx << " " << y_idx << "\n";

    return 0;//height_map(x_idx, y_idx);

}

void NeuralMPCLocomotion::_IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected, 
    const DMat<int> & idx_map){

  if(idx_map(x_idx, y_idx) == 0){ // (0,0)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx) == 0){ // (1, 0)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx+1) == 0){ // (1, 1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx, y_idx+1) == 0){ // (0, 1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx+1) == 0){ // (-1, 1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx) == 0){ // (-1, 0)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx-1, y_idx-1) == 0){ // (-1, -1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx, y_idx-1) == 0){ // (0, -1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+1, y_idx-1) == 0){ // (1, -1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx-1) == 0){ // (2, -1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx) == 0){ // (2, 0)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+2, y_idx+1) == 0){ // (2, 1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx+2, y_idx+2) == 0){ // (2, 2)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+2;

  }else{
    printf("no proper step location (%d, %d)\n", x_idx, y_idx);
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;
  }
}

void NeuralMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
    (void)data;
    /*if( data._quadruped->_robotType == RobotType::MINI_CHEETAH ||
        data._quadruped->_robotType == RobotType::MINI_CHEETAH_VISION ){
	    _body_height = 0.29;
    }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
	    _body_height = 0.45;
    }else{
	    assert(false);
    }

    for(int i=0; i<3; i++){ vel_act[i] = data._desiredStateCommand->data.action[i];}
    for(int i=0; i<3; i++){ vel_rpy_act[i] = data._desiredStateCommand->data.action[i+3];}
    for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = data._desiredStateCommand->data.action[i+6];} 
    for(int i=0; i<4; i++){ fh_rel_act[i] = data._desiredStateCommand->data.action[i+14];}
    footswing_height_act = data._desiredStateCommand->data.action[18];
    for(int i=0; i<4; i++){ offsets_act[i] = data._desiredStateCommand->data.action[i+19]; }
    for(int i=0; i<4; i++){ durations_act[i] = data._desiredStateCommand->data.action[i+23]; }
 
    //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
    int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
    float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
    iterationCounter = _phase * data._desiredStateCommand->data.action[27] * horizonLength + _iteration * data._desiredStateCommand->data.action[27];
    std::cout << "iteration " << _iteration << " phase " << _phase << "\n";

    iterationsBetweenMPC_cmd = data._desiredStateCommand->data.action[27];
    */
}
	    

template<>
void NeuralMPCLocomotion::runParamsFixed(ControlFSMData<float>& data,
    const Vec3<float> & vel_cmd, const Vec3<float> & vel_rpy_cmd, const Vec2<float> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, const Vec4<int> & offsets_prev, const Vec4<int> & durations_prev, const Vec4<int> & offsets_cmd,
    const Vec4<int> & durations_cmd, const Vec4<int> & offsets_next, const Vec4<int> & durations_next, const float footswing_height, const int iterationsBetweenMPC_cmd, const DMat<float> & height_map) {
    
  
	
  //std::cout << "NeuralMPCLocomotion::run" << "\n";
  
  (void)fp_rel_cmd;
  (void)offsets_cmd;
  (void)durations_cmd;

  _SetupCommand(data);

  auto& seResult = data._stateEstimator->getResult();

  if(data.controlParameters->use_rc ){
    data.userParameters->cmpc_gait = data._desiredStateCommand->rcCommand->variable[0];
  }
  
  gaitNumber = data.userParameters->cmpc_gait;

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  NeuralGait* gait = &trotting;
  if(gaitNumber == 1)         gait = &bounding;
  else if(gaitNumber == 2)    gait = &pronking;
  else if(gaitNumber == 3)    gait = &galloping;
  else if(gaitNumber == 4)    gait = &standing;
  else if(gaitNumber == 5)    gait = &trotRunning;
  current_gait = gaitNumber;

  if(iterationsBetweenMPC != iterationsBetweenMPC_cmd){
    int iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
    float phase = (float)(iterationCounter % (iterationsBetweenMPC * horizonLength)) / (float) (iterationsBetweenMPC * horizonLength);
    iterationCounter = phase * iterationsBetweenMPC_cmd * horizonLength + iteration * iterationsBetweenMPC_cmd;
    //std::cout << "iteration " << iteration << " phase " << phase << "Iteration counter" << iterationCounter << "\n";

    iterationsBetweenMPC = iterationsBetweenMPC_cmd;
    dtMPC = dt * iterationsBetweenMPC;
    //printf("[Neural MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
    //   dt, iterationsBetweenMPC, dtMPC);
  }
  dtMPC = dt * iterationsBetweenMPC;
  //printf("[Neural MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
  //    dt, iterationsBetweenMPC, dtMPC);
  //neural_setup_problem(dtMPC, horizonLength, 0.4, 120);
  
  NeuralGait custom(horizonLength, offsets_prev, durations_prev, offsets_cmd, durations_cmd, offsets_next, durations_next, "Cyclic Walk");
  
  // Can modify
  gait = &custom; // set custom gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  
  int* mpcTable = gait->mpc_gait();
  updateMPCIfNeeded(mpcTable, data);

  // integrate position setpoint
  v_des_world[0] = vel_cmd[0];
  v_des_world[1] = vel_cmd[1];
  v_des_world[2] = vel_cmd[2];
  //rpy_des[0] = seResult.rpy[0];
  //rpy_des[1] = seResult.rpy[1];
  rpy_des[2] = seResult.rpy[2];// + dt * vel_rpy_cmd[2];
  v_rpy_des[0] = vel_rpy_cmd[0];
  v_rpy_des[1] = vel_rpy_cmd[1]; // Pitch not yet accounted for in MPC
  v_rpy_des[2] = vel_rpy_cmd[2];
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");
  
  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2) {  //avoid dividing by zero 
    rpy_int[1] += dt*(rpy_des[1] - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt*(rpy_des[0] - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);// v_des_world[2]);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];//seResult.position[2];

    for(int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(0.06);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }
  
  // foot placement
  swingTimes[0] = dtMPC * gait->_swing[0]; // swing_time_cmd;
  swingTimes[1] = dtMPC * gait->_swing[1]; // swing_time_cmd;
  swingTimes[2] = dtMPC * gait->_swing[2]; // swing_time_cmd;
  swingTimes[3] = dtMPC * gait->_swing[3]; // swing_time_cmd;

  float side_sign[4] = {-1, 1, -1, 1};

  for(int i = 0; i < 4; i++) {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
      //std::cout << "First Swing! " << i ;
    } else {
      swingTimeRemaining[i] -= dt;
    }
     // std::cout << " swing time: " << swingTimeRemaining[i] << "swing time: " << gait->_swing[i] <<  " stance time: " << gait->_stance[i] << "\n";

    // Swing Height
    //footSwingTrajectories[i].setHeight(_fin_foot_loc[i][2] + footswing_height);
    footSwingTrajectories[i].setHeight(footswing_height);

    // Foot under the hip
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    
    // Account for desired yaw rate
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, 
          -v_rpy_des[2] * gait->_stance[i] * dtMPC / 2) * pRobotFrame;

    // Account for body velocity
    Vec3<float> des_vel = seResult.rBody * v_des_world;
    Vec3<float> Pf = seResult.position +
      seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    // Heuristics for foot placement
    float p_rel_max = 0.3f;
    float pfx_rel = seResult.vWorld[0] * .5 * gait->_stance[i] * dtMPC +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*v_rpy_des[2]);

    float pfy_rel = seResult.vWorld[1] * .5 * gait->_stance[i] * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*v_rpy_des[2]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = 0.;

#ifdef DRAW_DEBUG_FOOTHOLD // Nominal foothold location
        footPlaceSafe = true;
        Vec3<float> Pf_nom = Pf;
        _UpdateFoothold(Pf_nom, seResult.position, height_map);
        auto* nominalSphere = data.visualizationData->addSphere();
        nominalSphere->position = Pf_nom;
        nominalSphere->radius = 0.02;
        nominalSphere->color = {0.9, 0.55, 0.05, 0.7}; // orange = nominal
#endif

    Pf[0] = Pf[0] + fp_rel_cmd[i][0];
    Pf[1] = Pf[1] + fp_rel_cmd[i][1];
    //std::cout << fp_rel_cmd[i][0] << ", " << fp_rel_cmd[i][1] << " == " << Pf[0] << ", " << Pf[1] << "\n";

    _UpdateFoothold(Pf, seResult.position, height_map);
    Pf[2] = Pf[2] + fh_rel_cmd[i];

    _fin_foot_loc[i] = Pf;
    //Pf[2] -= 0.003;
    //printf("%d, %d) foot: %f, %f, %f \n", x_idx, y_idx, local_pf[0], local_pf[1], Pf[2]);
    footSwingTrajectories[i].setFinalPosition(Pf);

#ifdef DRAW_DEBUG_FOOTHOLD // Corrected foothold location
        auto* correctedSphere = data.visualizationData->addSphere();
        correctedSphere->position = Pf;
        correctedSphere->radius = 0.02;
        if (footPlaceSafe)
            correctedSphere->color = {0.1, 0.9, 0.1, 0.7}; // green = safe
        else
            correctedSphere->color = {0.9, 0.1, 0.1, 0.7}; // red = unsafe
#endif
  }

  // load LCM leg swing gains
  Kp << 700, 0, 0,
    0, 700, 0,
    0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 11, 0, 0,
    0, 11, 0,
    0, 0, 11;
  Kd_stance = Kd;
  
  // gait update
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  iterationCounter++;

  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();

 // std::cout << "contactStates: " << contactStates[0] << " " << contactStates[1] << " " << contactStates[2] << " " << contactStates[3] << " swingStates: " << swingStates[0] << " " << swingStates[1] << " " << swingStates[2] << " " << swingStates[3] << "\n";
  //std::cout << "iteration: " << (iterationCounter / iterationsBetweenMPC) / 10 << " offsets: " << offsets_cmd[0] << " " << offsets_cmd[1] << " " << offsets_cmd[2] << " " << offsets_cmd[3] << " durations: " << durations_cmd[0] << " " << durations_cmd[1] << " " << durations_cmd[2] << " " << durations_cmd[3] << "\n";

  //int* mpcTable = gait->mpc_gait();
  //updateMPCIfNeeded(mpcTable, data);

  Vec4<float> se_contactState(0,0,0,0);
  Vec4<float> se_footHeight(0,0,0,0);

  // Body height
  float floor_height = 0;
  int stance_leg_cnt = 0;

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];//;contact_cmd[foot]
    float swingState = swingStates[foot];

    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
            auto* debugPath = data.visualizationData->addPath();
            if(debugPath) {
                debugPath->num_points = 100;
                debugPath->color = {0.2,1,0.2,0.5};
                float step = (1.f - swingState) / 100.f;
                for(int i = 0; i < 100; i++) {
                    footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
                    debugPath->position[i] = footSwingTrajectories[foot].getPosition();
                }
            }

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
            auto* actualSphere = data.visualizationData->addSphere();
            actualSphere->position = pFoot[foot];
            actualSphere->radius = 0.02;
            actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      //footSwingTrajectories[foot].setHeight(_fin_foot_loc[foot][2] + footswing_height);
      footSwingTrajectories[foot].setHeight(footswing_height);
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);


      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;

        //singularity barrier
        data._legController->commands[foot].tauFeedForward[2] = 
          50*(data._legController->datas[foot].q(2)<.1)*data._legController->datas[foot].q(2);
      }
      se_footHeight[foot] = pDesFootWorld[2];
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      stance_leg_cnt++;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }
      floor_height += pDesFootWorld[2];
      se_contactState[foot] = contactState;

      se_footHeight[foot] = getFootHeight(pFoot[foot], seResult.position, height_map);
    }
  }

  // Get floor height
  floor_height /= stance_leg_cnt;
  //Vec3<float> avgStanceFootPos = data._stateEstimator->getAverageStanceFootPosWorld();

  // Set contact/foot height info
  data._stateEstimator->setContactPhase(se_contactState);
  //data._stateEstimator->setFootHeights(se_footHeight);

  // Body Height
  const float HEIGHT_ABOVE_GROUND = 0.31;
  //_body_height = avgStanceFootPos[2] + HEIGHT_ABOVE_GROUND;
  // TEST
  //_body_height = floor_height + HEIGHT_ABOVE_GROUND;
  _body_height = HEIGHT_ABOVE_GROUND;

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height; //world_position_desired[2];

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.; //v_des_world[2];

  pBody_RPY_des[0] = 0.; //rpy_des[0];
  pBody_RPY_des[1] = 0., //rpy_des[1]; 
  pBody_RPY_des[2] = rpy_des[2];

  vBody_Ori_des[0] = 0.; //v_rpy_des[0];
  vBody_Ori_des[1] = 0.; //v_rpy_des[1];
  vBody_Ori_des[2] = v_rpy_des[2];

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
}

template<>
void NeuralMPCLocomotion::run(ControlFSMData<float>& data,
    const Vec3<float> & vel_cmd, const Vec3<float> & vel_rpy_cmd, const Vec2<float> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, const Vec4<int> & offsets_cmd,
    const Vec4<int> & durations_cmd, const float footswing_height_cmd, const int iterationsBetweenMPC_cmd, const DMat<float> & height_map){
  /*
  if((iterationCounter % (iterationsBetweenMPC * 10)) == 0){
  //auto& seResult = data._stateEstimator->getResult();
  //auto& limbData = data._stateEstimator->getLimbData();
  
  //bool use_LCM = true;
  
  //if(use_LCM == true){
  
    auto& seResult = data._stateEstimator->getResult();
    // Build 58-dimensional robot state
    rl_obs_t rl_obs;

    // body height
    rl_obs.body_ht = seResult.position[2];
    for(int i=0; i<3; i++){ rl_obs.robot_world_pos[i] = seResult.position[i]; } // not used in policy, for ground truth logging
    // body orientation
    for(int i=0; i<3; i++){ rl_obs.rpy[i] = seResult.rpy[i]; }
    // joint angles
    for(int i=0; i<4; i++){
  	 for(int j=0; j<3; j++){ 
  		 rl_obs.q[i*3+j] = data._stateEstimator->getLimbData(i)->q[j]; 
	 }
    }
    // body (linear and angular) velocities
    for(int i=0; i<3; i++){ rl_obs.vBody[i] =  seResult.vBody[i]; }
    for(int i=0; i<3; i++){ rl_obs.omegaWorld[i] = seResult.omegaWorld[i]; }
    // previous command
    //
    //obs_accessor[34:38] = 
    //obs_accessor[38:42] = 
    //obs_accessor[42:45] = 
    //obs_accessor[45] = 
    // joint velocities
    for(int i=0; i<4; i++){
	 for(int j=0; j<3; j++){
		 rl_obs.qd[i*3+j] = data._stateEstimator->getLimbData(i)->qd[j]; 
	 }
    }
    // Append heightmap to observation
    for(int i=0; i<height_map.cols(); i++){
	  for(int j=0; j<height_map.rows(); j++){
		  rl_obs.height_map[i][j] = height_map(i, j);
          }
    }
    rl_obs.mpc_progress = (iterationCounter / iterationsBetweenMPC) % 10; 

    std::cout << rl_obs.mpc_progress;
    
    // store prev vel (for smooth transition)
    for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
  

    // send obs to python via LCM
    // request action from python

    _policyRecieved = 0;
    printf("Sending policy request...");
    _neuralLCM.publish("rl_gait_obs", &rl_obs);

    //wait for response
    //while( _policyRecieved < 1){
    //  usleep(100);
    //  printf("Waiting for a response from python...");
      //_neuralLCM.publish("rl_gait_obs", &rl_obs);
    //}
    
    //std::cout << "vel_act: " << vel_act << "vel_rpy_act" << vel_rpy_act << "\n";
  
  } else{
    for(int i=0; i<3; i++){ vel_act[i] = data._desiredStateCommand->data.action[i];}
    for(int i=0; i<3; i++){ vel_rpy_act[i] = data._desiredStateCommand->data.action[i+3];}
    for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = data._desiredStateCommand->data.action[i+6];} 
    for(int i=0; i<4; i++){ fh_rel_act[i] = data._desiredStateCommand->data.action[i+14];}
    footswing_height_act = data._desiredStateCommand->data.action[18];
    for(int i=0; i<4; i++){ offsets_act[i] = (int)data._desiredStateCommand->data.action[i+19]; }
    for(int i=0; i<4; i++){ durations_act[i] = (int)data._desiredStateCommand->data.action[i+23]; }
    //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
    //int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
    //float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
    //iterationCounter = _phase * data._desiredStateCommand->data.action[27] * horizonLength + _iteration * data._desiredStateCommand->data.action[27];

    //iterationsBetweenMPC_cmd = (int)data._desiredStateCommand->data.action[27];
    //
    //std::cout << "UPDATE \n";

    }
  }
  
    //std::cout << "vel_act: " << vel_act << "vel_rpy_act" << vel_rpy_act << "fp_rel_act " << fp_rel_act << "fh_rel_act" << fh_rel_act << "footswing_height_act" <<  footswing_height_act << "offsets_act: " << offsets_act << "durations_act" << durations_act << "\n";
  */
  
  // smooth offsets, durations transition
  int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
 
  if(_iteration == 0 and iterationCounter % (iterationsBetweenMPC * 10) == 0){ // updated command!
     for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
     for(int i=0; i<4; i++){ offsets_act_prev[i] = offsets_act[i];}
     for(int i=0; i<4; i++){ offsets_act[i] = offsets_act_next[i];}
     for(int i=0; i<4; i++){ durations_act_prev[i] = durations_act[i];}
     for(int i=0; i<4; i++){ durations_act[i] = durations_act_next[i];}
  }
  //for(int i = 0; i < 4; i++){
  //  if((offsets_act[i] + durations_act[i]) % horizonLength == _iteration){ //foot lifting off
  //    offsets_act[i] = offsets_act_next[i]; // update offsets, durations on liftoff
  
  
  for(int i=0; i<3; i++){ vel_act_next[i] = vel_cmd[i];}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = vel_rpy_cmd[i];}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = fp_rel_cmd[i%4][i/4];} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = fh_rel_cmd[i];}
  footswing_height_act = footswing_height_cmd;
  for(int i=0; i<4; i++){ offsets_act_next[i] = offsets_cmd[i]; }
  for(int i=0; i<4; i++){ durations_act_next[i] = durations_cmd[i]; }
  
  // limit duration commands to not cycle over
  //for(int i = 0; i < 4; i++){
  //  if(offsets_act_next[i] + durations_act_next[i] > 10){ // wrapping
  //      durations_act_next[i] = 10 - offsets_act_next[i];
  //  }
  //}
   
  //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
  
  float _phase = (float)(iterationCounter % (iterationsBetweenMPC * horizonLength)) / (float) (iterationsBetweenMPC * horizonLength);
  if(iterationsBetweenMPC_cmd != iterationsBetweenMPC){
  	iterationCounter = _phase * iterationsBetweenMPC_cmd * horizonLength + _iteration * iterationsBetweenMPC_cmd;
  	iterationsBetweenMPC = iterationsBetweenMPC_cmd;
  }

  _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
  for(int i=0; i<3; i++){ vel_act[i] = (vel_act_next[i] - vel_act_prev[i]) * _phase + vel_act_prev[i];}  

  // smooth offsets, durations transition
  //for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act[i];}
  //for(int i=0; i<3; i++){ offsets_act_prev[i] = offsets_act_next[i];}
  //for(int i=0; i<3; i++){ durations_act[i] = durations_act_next[i];}

  //for(int i = 0; i < 4; i++){
  //  if((offsets_act[i] + durations_act[i]) % horizonLength == _iteration){ //foot lifting off
  //    offsets_act[i] = offsets_act_next[i]; // update offsets, durations on liftoff
  //    durations_act[i] = durations_act_next[i];
   // }
    //if((offsets_act[i]) % horizonLength == _iteration){ // update on landing
    //  offsets_act[i] = offsets_act_next[i];
    //  durations_act[i] = durations_act_next[i];
    //}
  //}

   
  
  runParamsFixed(data, vel_act, vel_rpy_act, fp_rel_act, fh_rel_act, offsets_act_prev, durations_act_prev, offsets_act, durations_act, offsets_act_next, durations_act_next, footswing_height_act, iterationsBetweenMPC_cmd, height_map);
}


void NeuralMPCLocomotion::handleActionLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string &chan,
    const rl_action_lcmt *msg) {
  (void)rbuf;
  (void)chan;

  printf("Recieved action\n");
  std::cout << "ok\n"; 
  std::cout << vel_act << "\n"; 
  std::cout << msg->vel_act << "ok\n"; 
  
  for(int i=0; i<3; i++){ vel_act_next[i] = msg->vel_act[i];}
  std::cout << "ok\n"; 
  for(int i=0; i<3; i++){ vel_rpy_act[i] = msg->vel_rpy_act[i];}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = msg->fp_rel_act[i];} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = msg->fh_rel_act[i];}
  std::cout << "ok\n"; 
  footswing_height_act = msg->footswing_height_act;
  for(int i=0; i<4; i++){ offsets_act_next[i] = msg->offsets_act[i]; }
  for(int i=0; i<4; i++){ durations_act_next[i] = msg->durations_act[i]; }
   
  std::cout << "ok\n";
  std::cout << iterationCounter << " " << iterationsBetweenMPC_cmd << " " << horizonLength << " \n";
  //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
  int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
  float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
  iterationCounter = _phase * msg->iterationsBetweenMPC_act * horizonLength + _iteration * msg->iterationsBetweenMPC_act;
  iterationsBetweenMPC_cmd = msg->iterationsBetweenMPC_act;
  std::cout << "iteration " << _iteration << " phase " << _phase << "\n";

  

  printf("Successfully read action\n");

  _policyRecieved = 1;

}


void NeuralMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) {
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    if(current_gait == 4)    {
      float trajInitial[12] = {(float)rpy_des[0], // Roll
                               (float)rpy_des[1], // Pitch
                               (float)stand_traj[5],
                               (float)stand_traj[0],
                               (float)stand_traj[1],
                               (float)_body_height,
                               0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    } else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
                               (float)rpy_comp[1],    // 1
                               (float)rpy_des[2],    // 2
                               xStart,                                   // 3
                               yStart,                                   // 4
                               (float)_body_height,      // 5
                               0,                                        // 6
                               0,                                        // 7
                               (float)v_rpy_des[2],  // 8
                               v_des_world[0],                           // 9
                               v_des_world[1],                           // 10
                               0};                                       // 11

      for(int i = 0; i < horizonLength; i++) {
        for(int j = 0; j < 12; j++)  trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        } else {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * v_rpy_des[2];
        }
      }
    }
    solveDenseMPC(mpcTable, data);
  }
}

void NeuralMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++) r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  dtMPC = dt * iterationsBetweenMPC;
  neural_setup_problem(dtMPC,horizonLength,0.4,120);
  neural_update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = neural_get_solution(leg*3 + axis);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}
