#ifndef __PRINT_MATH__
#define __PRINT_MATH__

#include <Eigen/Dense>
#include "tools/PrintThis.hpp"

class PrintMath{
public:
    PrintMath(){}
    void print(const char * name, const Eigen::MatrixXd & mat);
    void print(const char * name, const Eigen::VectorXd & vec);
    void print(const char * name, const std::vector<double> & vec);
    void print(const char * name, double num);
    void print(const char * name, int num);
    void print(const char * name, float num);
    void print(const char * name, size_t num);
private:
    PrintThis ver;
    char buff[200];
};

#endif // __PRINT_MATH__