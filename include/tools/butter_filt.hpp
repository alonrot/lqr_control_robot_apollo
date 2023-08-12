#ifndef INCLUDE_BUTTER_FILT_H
#define INCLUDE_BUTTER_FILT_H

// Filter structure:
typedef struct{

  double x[3];
  double y[3];
  double a[3];
  double b[3];

} Butter_filt;


class ButterFilt {

public:

	ButterFilt(int which_filt);
	~ButterFilt();

	double apply_filter(double raw_value);

private:
	Butter_filt filter_param;


};

#endif /* INCLUDE_BUTTER_FILT_H */