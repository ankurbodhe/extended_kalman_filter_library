#include <iostream>
#include "file_parse_util.h"


int main(int argc, char *argv[]){

    std::string file_name = argv[1];

    std::cout<<"Reading file ...."<< std::endl;
    
    Data file_data;
    file_data.parse_file(file_name.c_str());
    file_data.display_data_records();

    return 0;
}