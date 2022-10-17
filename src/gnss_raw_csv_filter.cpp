#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <signal.h>
#include <cstdlib>

#include <ublox_msgs/RxmRAWX_Meas.h>
#include <ublox_msgs/RxmRAWX.h>
#include <gnss_filter/gnss_raw_csv.h>
#include <sensor_msgs/NavSatFix.h>

std::ofstream csv_file;

void Siginthandler(int sig){
    std::cout << "\033[31m" << "Ctrl + C !" << "\033[32m" << std::endl;
    csv_file.close();
    ros::shutdown();
}

class Gnss_csv{
    private:
        int _row_num = 0;
        double _utc_time_1980 = 315964800; //1980/01/06 00:00:00
        int _secs_of_week = 604800;
        sensor_msgs::NavSatFix _fix_data;
        ublox_msgs::RxmRAWX _raw_data;
        gnss_filter::gnss_raw_csv _csv_data;
    
    public:
        void fixCallback(const sensor_msgs::NavSatFix& msg);
        void rxmrawCallback(const ublox_msgs::RxmRAWX& msg);
        double gpstime2utctime(const double TOW, const int week, const int leaps);
        void write_csv();
};

void Gnss_csv::fixCallback(const sensor_msgs::NavSatFix& msg){
    _fix_data = msg;

    double fix_time = _fix_data.header.stamp.nsec;
    fix_time = fix_time/1000000000 + _fix_data.header.stamp.sec;
    std::cout << std::setprecision(9);
    std::cout << std::fixed;
    std::cout <<  "fix_time: " << fix_time << std::endl;
    double raw_time = gpstime2utctime(_raw_data.rcvTOW, _raw_data.week, _raw_data.leapS);
    std::cout <<  "raw_time: " << raw_time << std::endl;
    std::cout <<  "-----------------------" << std::endl;

    if (std::abs(fix_time - raw_time) < 0.5){
        write_csv();
    }

}

void Gnss_csv::rxmrawCallback(const ublox_msgs::RxmRAWX& msg){
    _raw_data = msg;

    double fix_time = _fix_data.header.stamp.nsec;
    fix_time = fix_time/1000000000 + _fix_data.header.stamp.sec;
    std::cout << std::setprecision(9);
    std::cout << std::fixed;
    std::cout <<  "fix_time: " << fix_time << std::endl;
    double raw_time = gpstime2utctime(_raw_data.rcvTOW, _raw_data.week, _raw_data.leapS);
    std::cout <<  "raw_time: " << raw_time << std::endl;
    std::cout <<  "-----------------------" << std::endl;

    if (std::abs(fix_time - raw_time) < 0.5){
        write_csv();
    }
}

void Gnss_csv::write_csv(){
    if (_row_num == 0){
        std::vector<std::string> row = {"rcvTOW", "week", "leapS", "numMeas", "latitude", "longitude", "altitude",
        "prMes", "cpMes", "doMes", "gnssId", "svId", "freqId", "locktime", "cno", "prStdev", "cpStdev", "doStdev", "trkStat"};
        for (int i=0;i<row.size();i++)
        {
            csv_file << row[i]<<",";
        }
        csv_file << std::endl;
        _row_num++;
    }
    else{
        for (int i = 0; i < _raw_data.meas.size(); i++){
            csv_file << std::fixed << std::setprecision(9);
            csv_file << _raw_data.rcvTOW <<",";
            csv_file << _raw_data.week <<",";
            csv_file << int(_raw_data.leapS) <<",";
            csv_file << int(_raw_data.numMeas) <<",";
            csv_file << _fix_data.latitude <<",";
            csv_file << _fix_data.longitude <<",";
            csv_file << _fix_data.altitude <<",";
            csv_file << _raw_data.meas[i].prMes <<",";
            csv_file << _raw_data.meas[i].cpMes <<",";
            csv_file << _raw_data.meas[i].doMes <<",";
            csv_file << int(_raw_data.meas[i].gnssId) <<",";
            csv_file << int(_raw_data.meas[i].svId) <<",";
            csv_file << int(_raw_data.meas[i].freqId) <<",";
            csv_file << _raw_data.meas[i].locktime <<",";
            csv_file << int(_raw_data.meas[i].cno) <<",";
            csv_file << int(_raw_data.meas[i].prStdev )<<",";
            csv_file << int(_raw_data.meas[i].cpStdev) <<",";
            csv_file << int(_raw_data.meas[i].doStdev) <<",";
            csv_file << int(_raw_data.meas[i].trkStat);
            csv_file << std::endl;

            // std::cout << std::setprecision(9);
            // std::cout << std::fixed;
            // std::cout << _row_num << std::endl;
            _row_num++;
        }
    }
}

double Gnss_csv::gpstime2utctime(const double TOW, const int week, const int leaps){
    double time = _secs_of_week * week + TOW - leaps + _utc_time_1980;
    return time;
}

bool exists_test(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv){
    signal(SIGINT, Siginthandler);
    ros::init(argc, argv, "gnss_raw_csv_filter");
    ros::NodeHandle n;
    
    std::string dir;
    std::string filename;
    std::string filetype = "_gnssraw";

    ros::param::get("directory", dir);
    ros::param::get("filename", filename);

    if(exists_test(dir + filename + filetype + ".csv")){
        std::cout << "\033[31m" << "File exists! Check the file name!" << "\033[0m" << std::endl;
        return 0;
    }

    csv_file.open(dir + filename + filetype + ".csv");
    Gnss_csv gnss_csv;

    ros::Subscriber sub1 = n.subscribe("/ublox_f9p/fix", 1000, &Gnss_csv::fixCallback, &gnss_csv);
    ros::Subscriber sub2 = n.subscribe("/ublox_f9p/rxmraw", 1000, &Gnss_csv::rxmrawCallback, &gnss_csv);

    ros::Rate loop_rate(1);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}