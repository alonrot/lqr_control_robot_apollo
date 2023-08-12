#ifndef __PRINT_THIS__
#define __PRINT_THIS__

#include <string>
#include <iostream>

#ifdef __XENO__
	#include <xenomai/rtdk.h> // Contains the definition of rt_printf()
#endif

class PrintThis{
public:
		PrintThis();
		PrintThis(std::string my_msg,int print_limit);
		void initialize(std::string my_msg,int print_limit);
    void print(void);
    void print(const char * msg_in);
    virtual ~PrintThis() {};

private:
		void print_call(const char * msg_in);
    void update_counter(void);
    int counter;
    int print_limit;
    bool keep_printing;
    std::string my_msg;
};

#endif // __PRINT_THIS__