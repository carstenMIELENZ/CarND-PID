#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/


PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd) {

  // Coefficients
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  // Errors
  p_error_     = 0;
  i_error_     = 0;
  d_error_     = 0;
  d_error_prev = 0;  

  // Run status
  init_        = false;

  // Tuning disabled
  tuning_en_   = false;
  steps_       = 0;
}


void PID::UpdateError(double cte) {

  // P component
  p_error_  = cte;

  // I component
  i_error_ += cte;

  // I component
  if (!init_) { 
    init_      = true;
    d_error_   = 0;
  }
  else {
    d_error_   = cte - d_error_prev;
  }
  d_error_prev = cte;  

  // tuning enabled ?
  if (tuning_en_) { 
    tuning_en_ = RunTwiddle(cte);
    Kp_ = tuning_p[0];
    Kd_ = tuning_p[1];
    Ki_ = tuning_p[2];
  }
 
}
 

double PID::TotalError() {
 
  return -1.0 * Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
}


// ------------------------------------------------------------
// TWIDDLE ALGO
// ------------------------------------------------------------


void PID::EnableTuning(double t_p[], double t_pd[], double t_exit, double t_steps, bool t_verbose) {

   // set p & pd
   for (int u=0;u<3;++u) {
      tuning_p[u]    = t_p[u];   
     tuning_pd[u]    = t_pd[u];
   }

   // total error    
   error_         = 0;
   best_error_    = 0;

   // steps & exit criteria
   tuning_step_    = t_steps;
   tuning_exit_    = t_exit;

   // verbose
   tuning_verbose  = t_verbose;

   // tuning status
   tuning_state    = 0; 
   index_          = 0; 
   tuning_en_      = true; 

   if (tuning_verbose) 
     std::cout << "NOTE: tuning set Kp =" << tuning_p[0] << " - Kd = " << tuning_p[1] << " - Ki = " << tuning_p[2] << std::endl;
}


bool PID::RunTwiddle(double cte) {
 
  // increase steps 
  steps_++;
  
  error_ += (cte * cte);  // dont need to mean it

  // tune
  if (steps_ % tuning_step_ == 0) {
 
    // 1st time set best_error
    if (steps_ == tuning_step_)     { error_ = 0;  return true; } 
    if (steps_ == 2 * tuning_step_) { best_error_ = error_; error_ = 0; } 

    if (tuning_verbose) 
      std::cout << std::endl << "NOTE: best error " << best_error_<< " @ steps " << steps_ << std::endl;
   
    // Twiddle control
    while (tuning_pd[0]+tuning_pd[1]+tuning_pd[2] > tuning_exit_) { 
        
       switch (tuning_state) {
               
         case 0:
           tuning_p[index_] += tuning_pd[index_];
           tuning_state      = 1;
           return true;   // wait
 
         case 1:   
           if (error_ < best_error_) {
             best_error_ = error_;
             error_ = 0; 
             tuning_pd[index_] *= 1.1;
             tuning_state = 0;
             
             if (tuning_verbose) {
               std::cout << "NOTE: best error " << best_error_<< " @ steps " << steps_ << std::endl;
               std::cout << "NOTE: Kp = " << tuning_p[0] << " - Kd = " << tuning_p[1] << " - Ki = " << tuning_p[2] << std::endl;
             }
             
             index_ = (index_+1)%3; // next index
             break;
           } 
           else {
             tuning_p[index_] -= (2 * tuning_pd[index_]);
             error_ = 0; // reset
             tuning_state = 2;
             return true;
           }

         case 2: 
           if (error_ < best_error_) {
             best_error_ = error_;
             error_ = 0; // reset
             tuning_pd[index_] *= 1.1;
             
             if (tuning_verbose) {
               std::cout << "NOTE: best error " << best_error_<< " @ steps " << steps_ << std::endl;
               std::cout << "NOTE: Kp = " << tuning_p[0] << " - Kd = " << tuning_p[1] << " - Ki = " << tuning_p[2] << std::endl;
             }

           }
           else {
             tuning_p[index_]  += tuning_pd[index_];
             tuning_pd[index_] *= 0.9;
             error_ = 0; // reset
            }
            tuning_state = 0; 
            index_ = (index_+1)%3;
            break;

         default:
           std::cout << "ERROR: this state is forbidden !" << std::endl;
           break;
 
        } // end switch         

      } // end while

      std::cout << "NOTE: tuning done - Ki = " << tuning_p[0] << " - Kd = " << tuning_p[1] << " - Ki = " << tuning_p[2] << std::endl;
      return false; // tuned !!        
    
    } // end if

  return true;
}
