#include "tools/butter_filt.hpp"

ButterFilt::ButterFilt(int which_filter)
{

	// Parameters a,b:
  switch ( which_filter ) {

    // Pre-processing filter parameters:
    case 0:
      this->filter_param.a[0] = 0.0; this->filter_param.a[1] = 0.0; this->filter_param.a[2] = 0.0;
      this->filter_param.b[0] = 0.0; this->filter_param.b[1] = 0.0; this->filter_param.b[2] = 0.0;
      break;
    case 1: // 2nd order Butterworth filter, with fc = 1Hz:
      this->filter_param.a[0] =  1.000000000000000; this->filter_param.a[1] = -1.991114292201654; this->filter_param.a[2] =  0.991153595868935;
      this->filter_param.b[0] =  9.825916820471736e-06; this->filter_param.b[1] =  1.965183364094347e-05; this->filter_param.b[2] =  9.825916820471736e-06;
      break;
    case 2: // 2nd order Butterworth filter, with fc = 5Hz:
      this->filter_param.a[0] =  1.000000000000000; this->filter_param.a[1] = -1.955578240315035; this->filter_param.a[2] =  0.956543676511203;
      this->filter_param.b[0] =  2.413590490419615e-04; this->filter_param.b[1] =  4.827180980839230e-04; this->filter_param.b[2] =  2.413590490419615e-04;
      break;
    case 3: // 2nd order Butterworth filter, with fc = 10Hz:
      this->filter_param.a[0] =  1.000000000000000; this->filter_param.a[1] = -1.911197067426073; this->filter_param.a[2] =  0.914975834801434;
      this->filter_param.b[0] =  9.446918438401619e-04; this->filter_param.b[1] =  0.001889383687680; this->filter_param.b[2] =  9.446918438401619e-04;
      break;
    case 4: // 2nd order Butterworth filter, with fc = 15Hz:
      this->filter_param.a[0] =  1.000000000000000; this->filter_param.a[1] = -1.866892279711715; this->filter_param.a[2] =  0.875214548253684;
      this->filter_param.b[0] =  0.002080567135492; this->filter_param.b[1] =  0.004161134270985; this->filter_param.b[2] =  0.002080567135492;
      break;
    case 5: // 2nd order Butterworth filter, with fc = 20Hz:
      this->filter_param.a[0] =  1.000000000000000; this->filter_param.a[1] = -1.822694925196308; this->filter_param.a[2] =  0.837181651256023;
      this->filter_param.b[0] =  0.003621681514929; this->filter_param.b[1] =  0.007243363029857; this->filter_param.b[2] =  0.003621681514929;
      break;
  }

  // Variables x,y;
  filter_param.x[0] = 0; filter_param.x[1] = 0; filter_param.x[2] = 0;
  filter_param.y[0] = 0; filter_param.y[1] = 0; filter_param.y[2] = 0;

}

ButterFilt::~ButterFilt()
{}

double ButterFilt::apply_filter(double raw_value)
{

  filter_param.x[0] = raw_value;

  // Filtering:
  filter_param.y[0] = 	filter_param.b[0]*filter_param.x[0] + 
		                    filter_param.b[1]*filter_param.x[1] + 
		                    filter_param.b[2]*filter_param.x[2] - 
		                    filter_param.a[1]*filter_param.y[1] - 
		                    filter_param.a[2]*filter_param.y[2];

  // Assignments for next step:
  filter_param.x[2] = filter_param.x[1];
  filter_param.x[1] = filter_param.x[0];

  filter_param.y[2] = filter_param.y[1];
  filter_param.y[1] = filter_param.y[0];

  return filter_param.y[0];

}