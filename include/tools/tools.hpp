#ifndef INCLUDE_TOOLS_H
#define INCLUDE_TOOLS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include "tools/fake_sensor.hpp"
#include "tools/PrintThis.hpp"

#ifdef __XENO__
  #include <native/task.h>
  #include <native/timer.h>
  #include <xenomai/rtdk.h> // Contains the definition of rt_printf()
#else
  #include <ctime>
#endif

#define Norm(x,y) ( sqrt( sqr(x) + sqr(y) ) )
#define sqr(x)  ((x)*(x))

#define M_PI 3.14159265358979323846
#define TRUE 	1
#define FALSE 0

#define N_CART 3
#define N_CONTROLLER 5

#define _x_ 0
#define _y_ 1
#define _z_ 2

enum SystemStates{
	PHI 	= 0,
	PHID,
	S,
	SD,
	INT_,
	N_STATES
};

class CartesianState 
{
  public:
  CartesianState();
  std::vector<double> x;    /*!< Position [x,y,z] */ 
  std::vector<double> xd;   /*!< Velocity */
  std::vector<double> xdd;  /*!< Acceleration */
};

class SensorCartesianState {

public:
  SensorCartesianState();
  std::vector< std::shared_ptr<FakeSensor> > x;
  std::vector< std::shared_ptr<FakeSensor> > xd;
  std::vector< std::shared_ptr<FakeSensor> > xdd;
};

typedef struct{
	double th;
	double thd;
}KineticState;

namespace Tools{

	void 		vec_zero(std::vector<double> v);
	void 		vec_zero(std::vector<int> v);
	double 	sign(double expr);
	void 		eulerToRotMat(double a[N_CART], double R[N_CART][N_CART]);

}

// Namespace that contains specific parameters from the robot apollo, taken from SL
namespace apollo_par{

  // define the DOFs of this robot
  enum RobotDOFs {
    R_SFE = 1,
    R_SAA,
    R_HR,
    R_EB,
    R_WR,
    R_WFE,
    R_WAA,

    L_SFE,
    L_SAA,
    L_HR,
    L_EB,
    L_WR,
    L_WFE,
    L_WAA,

    B_HN,
    B_HT,
    B_HR,
    R_EP,
    R_ET,
    L_EP,
    L_ET,

    R_FR,
    R_RF,
    R_MF,
    R_LF,

    L_FR,
    L_RF,
    L_MF,
    L_LF,

    N_ROBOT_DOFS
  };

  const int N_DOFS = N_ROBOT_DOFS-1;
  const int n_endeffs = 2;
}

namespace Tools {

class TaskCounter{
public:
  TaskCounter();
  void restart(void);
  double get_elapsed_time_in_us(void);
  double get_elapsed_time_in_s(void);
private:
  #ifdef __XENO__
    SRTIME time_start;
  #else
    clock_t time_start;
  #endif
};

class SignalTreatment {

  public:
    SignalTreatment(double time_step);
    virtual ~SignalTreatment(){}
    double differentiate(double pos);

  private:
    double pos_old;
    double vel_old;
    int count_steps;
    double time_step;
    bool first_time;
    PrintThis verbosity;
};

// class PrintThis{
// public:
//   PrintThis(std::string my_msg, int print_limit, double time_limit);
//   void print(void);
//   void print(std::string my_msg);
//   virtual ~PrintThis() {};

// private:
//   bool budget_for_printing(void);
//   bool pause_for_printing(void);
//   bool can_we_print(void);
//   std::shared_ptr<TaskCounter> task_counter;
//   int counter;
//   int max_budget;
//   std::string my_msg;
//   double time_limit;
//   double time_elapsed;
//   bool restart_timer;
//   #ifdef __XENO__
//     SRTIME time_start;
//   #else
//     time_t time_start;
//   #endif
// };

} // namespace Tools

#endif /* INCLUDE_TOOLS_H */