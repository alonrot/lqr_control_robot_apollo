#include "tools/PrintMath.hpp"

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

int run(int argc, char** argv){

	PrintMath verb_math;

	int Nmax = 10;
	int i = 0;
	char buff[200];
	size_t Nr = 4;
	size_t Nc = 4;
	Eigen::VectorXd vec(Nc);
	vec.setRandom();
	Eigen::MatrixXd mat(Nr,Nc);
	mat.setRandom();
	double my_var_d = 4.0/7.0;
	float my_var_f = 6.0/7.0;
	int my_var_i = 3;
	size_t my_var_t = 5;

	while(i<Nmax){

		verb_math.print("my_vector",vec);
		verb_math.print("my_matrix",mat);
		verb_math.print("my_var_d",my_var_d);
		verb_math.print("my_var_f",my_var_f);
		verb_math.print("my_var_i",my_var_i);
		verb_math.print("my_var_t",my_var_t);

		++i;
	}

	return 0;
}


int main(int argc, char** argv){
  #ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
  #endif
  run(argc,argv);

  return 0;
}
