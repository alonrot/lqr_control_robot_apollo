#include "tools/PrintMath.hpp"

void 
PrintMath::print(const char * name, const Eigen::VectorXd & vec){

	// String allocation with std::string should be avoided, as it isn't real-time safe

	size_t Nel = vec.size();

	sprintf(this->buff,"%s = [",name);
  this->ver.print(this->buff);
  for(size_t i=0;i<Nel;++i){
    if(i == Nel-1){
      sprintf(this->buff,"%f]\n",vec(i));
      this->ver.print(this->buff);
    }
    else{
      sprintf(this->buff,"%f,",vec(i));
      this->ver.print(this->buff);
    }
  }

	return;
}

void 
PrintMath::print(const char * name, const std::vector<double> & vec){

	// String allocation with std::string should be avoided, as it isn't real-time safe

	size_t Nel = vec.size();

	sprintf(this->buff,"%s = [",name);
  this->ver.print(this->buff);
  for(size_t i=0;i<Nel;++i){
    if(i == Nel-1){
      sprintf(this->buff,"%f]\n",vec[i]);
      this->ver.print(this->buff);
    }
    else{
      sprintf(this->buff,"%f,",vec[i]);
      this->ver.print(this->buff);
    }
  }

	return;
}

void
PrintMath::print(const char * name, const Eigen::MatrixXd & mat){

	size_t Nr = mat.rows();
	size_t Nc = mat.cols();

	// Verbosity:
	sprintf(this->buff,"%s = \n",name);
	this->ver.print(this->buff);
  for(size_t i=0;i<Nr;++i){
    this->ver.print("[");
    for(size_t j=0;j<Nc;++j){
      if(j == Nc-1){
        sprintf(this->buff,"%f]\n",mat(i,j));
        this->ver.print(this->buff);
      }
      else{
        sprintf(this->buff,"%f,",mat(i,j));
        this->ver.print(this->buff);
      }
    }
  }

  return;
}

void
PrintMath::print(const char * name, double num){

	// Verbosity:
	sprintf(this->buff,"%s = %f\n",name,num);
	this->ver.print(this->buff);

	return;
}

void
PrintMath::print(const char * name, float num){

	// Verbosity:
	sprintf(this->buff,"%s = %f\n",name,num);
	this->ver.print(this->buff);

	return;
}

void
PrintMath::print(const char * name, int num){

	// Verbosity:
	sprintf(this->buff,"%s = %i\n",name,num);
	this->ver.print(this->buff);

	return;
}

void
PrintMath::print(const char * name, size_t num){

	// Verbosity:
	sprintf(this->buff,"%s = %ld\n",name,num);
	this->ver.print(this->buff);

	return;
}