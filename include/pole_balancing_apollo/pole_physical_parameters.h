/*============================================================================
==============================================================================
                      
                              pole_physical_parameters.h
 
==============================================================================
Remarks:
contains definition of a pole data structure


============================================================================*/

#ifndef _POLEPHYSICALPARAMETERS_H_
#define _POLEPHYSICALPARAMETERS_H_

// Physical pole parameters:
typedef struct
{
  double length;  
  double mass;
  double center_of_mass;
  double inertia;
  double damping;

}PoleParameters;

#endif /* _POLEPHYSICALPARAMETERS_H_ */