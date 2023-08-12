#include "tools/fake_sensor.hpp"

int run(int argc, char** argv)
{
	int delay = 2;
	double std_noise = 0.0;
	int steps = 2;

	// std::shared_ptr<FakeSensor> s;
	// s.reset(new FakeSensor(delay,std_noise));
	FakeSensor * s = new FakeSensor(std_noise,steps,delay);

	int c 		= 0;
	double sig  = 1;
	double sig_del  = 0.0;
	while( c < 100 )
	{
		
		// ++c;
		// if ( c % 3 == 0 )
			++sig;

		sig_del = s->measure(sig);
		printf("sig = %f | sig_del = %f\n",sig,sig_del);
	}

  return 0;
}

int main(int argc, char** argv){
#ifdef __XENO__
  mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
  run(argc,argv);
}