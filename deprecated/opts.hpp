// Structure of options:

#define n_cart 3

typedef struct
{
	// Flags:
	int    	record_data;

	// Total task time:
	double 	total_tt;

	// Sampling time:
	double 	time_step;

	// // End-effector initial offset:
	// Vector 	cart_offset;

	// End-effector initial offset:
	double * endeff_offset;

	// Rotation matrix (world to local):
	Matrix  rot_mat_;

	// Safety box dimensions:
	double box_length_thres;
	double box_width_thres;
	double box_height_thres;

}Options;