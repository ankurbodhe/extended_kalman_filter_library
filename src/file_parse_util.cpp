#include "file_parse_util.h"
#include <fstream>
#include <sstream>


Data :: Data(){

    data_val temp_dat;
    temp_dat.seq = 0;
    temp_dat.timestamp = 0;
    temp_dat.sensor_type = 'X';
    temp_dat.lidar_meas_px = 0.0;
    temp_dat.lidar_meas_py = 0.0;
    temp_dat.radar_meas_phi = 0.0;
    temp_dat.radar_meas_rho = 0.0;
    temp_dat.meas_rho_dot   = 0.0;
    temp_dat.gt_px = 0.0;
    temp_dat.gt_py = 0.0;
    temp_dat.gt_vx = 0.0;
    temp_dat.gt_vy = 0.0;

    data_list.push_back(temp_dat);
}

void Data::parse_file(string file_name){

    std::ifstream file(file_name);

    data_val dat;

    int i = 0;
    dat.seq = 0;
    
    // read file
    if(file.is_open()){

        std::string line;

        while(std::getline(file, line)){

            istringstream iss(line);

            iss >> dat.sensor_type;

            if(dat.sensor_type == 'L'){

                dat.seq++;
                iss >> dat.lidar_meas_px;
                iss >> dat.lidar_meas_py;
                iss >> dat.timestamp;
                iss >> dat.gt_px;
                iss >> dat.gt_py;
                iss >> dat.gt_vx;
                iss >> dat.gt_vy;

            }
            else if(dat.sensor_type == 'R'){

                dat.seq++;
                iss >> dat.radar_meas_rho;
                iss >> dat.radar_meas_phi;
                iss >> dat.meas_rho_dot;
                iss >> dat.timestamp;
                iss >> dat.gt_px;
                iss >> dat.gt_py;
                iss >> dat.gt_vx;
                iss >> dat.gt_vy;

            }
            if (i == 0)
                data_list[i] = dat;
            else
                data_list.push_back(dat);

            i++;

        }
        file.close();
    }    
}

void Data::display_data_records(){

    cout << " ----- Displaying the data records ----" << std::endl;

    for(long int i = 0 ; i < data_list.size(); i++){

        if (data_list[i].sensor_type == 'L'){
            cout << data_list[i].seq << " " << data_list[i].timestamp << " " << data_list[i].sensor_type
            << " " << data_list[i].lidar_meas_px << " " << data_list[i].lidar_meas_py << " " << data_list[i].gt_px
            << " " << data_list[i].gt_py << " " << data_list[i].gt_vx << " " << data_list[i].gt_vy << " " << std::endl;
        }
        else if (data_list[i].sensor_type == 'R'){
            cout << data_list[i].seq << " " << data_list[i].timestamp << " " << data_list[i].sensor_type
            << " " << data_list[i].radar_meas_rho << " " << data_list[i].radar_meas_phi << data_list[i].meas_rho_dot
            << " " << data_list[i].gt_px << " " << data_list[i].gt_py << " " << data_list[i].gt_vx << " " << data_list[i].gt_vy << " " << std::endl;
        }
    }
}
