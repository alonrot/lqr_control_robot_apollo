#include "lqr_gains/read_LQR_gains_from_file.hpp"

ReadLQRGains::ReadLQRGains(std::string fpath, std::string fname, std::string TOPIC, int n_gains)
{
  this->n_gains       = n_gains;
  this->set_of_gains  = new double[n_gains];

  this->fpath = cast_string_to_char(fpath);
  this->fname = cast_string_to_char(fname);
  this->TOPIC = cast_string_to_char(TOPIC);

}

ReadLQRGains::~ReadLQRGains() {}

char * ReadLQRGains::cast_string_to_char(std::string my_string)
{
  // Create char* array:
  char * my_char = new char[my_string.size() + 1];

  // Copy the string to the char array:
  std::copy(my_string.begin(), my_string.end(), my_char);

  // Important: add the 0 termination:
  my_char[my_string.size()] = '\0';

  return my_char;
}


bool ReadLQRGains::read_from_file(void)
{
  char file_path[500];

  sprintf(file_path,"%s%s",fpath,fname);

  std::string     control_gain_name;
  double          control_gain_num;
  std::ifstream   infile;
  std::ifstream   check_file;
  infile.open(file_path);
  bool file_is_empty = false;

  // Check whether the file is empty:
  check_file.open(file_path);
  std::string     foo;
  check_file >> foo;
  if(check_file.eof())
    file_is_empty = true;

  if (infile.is_open() && !file_is_empty)
  {

    for(int i=0;i<n_gains;++i)
    {
      infile >> control_gain_name >> control_gain_num;
      // printf("control_gain_name = %s\n",control_gain_name.c_str());
      set_of_gains[i] = control_gain_num;
      // printf("set_of_gains[%i] = %f\n",i,set_of_gains[i]);
    }

    infile.close();
  }
  else
    return false;

  return true;

}

bool ReadLQRGains::get_gains(void)
{
  if ( !this->read_from_file() )
    return false;

  return true;
}