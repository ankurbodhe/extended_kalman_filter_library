#ifndef FILE_PARSE_UTIL_HPP_
#define FILE_PARSE_UTIL_HPP_

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

using namespace std;

struct data_val {

    int seq;
    long long timestamp;

    int sensor_number;
    
    char sensor_type;

    float lidar_meas_px;
    float lidar_meas_py;

    float radar_meas_rho;
    float radar_meas_phi;
    float meas_rho_dot;

    float gt_px;
    float gt_py;
    float gt_vx;
    float gt_vy;

};

class Data {

    vector<data_val> data_list;

    public:
        Data();
        void parse_file(string file_name);
        void display_data_records();
        data_val access_data_record(int index);
        long data_record_size();

};

#endif /* MEASURFILE_PARSE_UTIL_HPP_ */