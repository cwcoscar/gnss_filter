#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <signal.h>

#include <gnss_filter/gnss_csv.h>
#include <sensor_msgs/NavSatFix.h>
#include <ublox_msgs/NavPVT.h>


std::ofstream csv_file;

void Siginthandler(int sig){
    std::cout << "Ctrl + C !" << std::endl;
    csv_file.close();
    ros::shutdown();
}

class Gnss_csv{
    private:
        // std::ofstream _file;
        int _row_num = 0;
        sensor_msgs::NavSatFix _fix_data;
        ublox_msgs::NavPVT _pvt_data;
        gnss_filter::gnss_csv _csv_data;
    
    public:
        void fixCallback(const sensor_msgs::NavSatFix& msg);
        void navpvtCallback(const ublox_msgs::NavPVT& msg);
        void write_csv();
};

void Gnss_csv::fixCallback(const sensor_msgs::NavSatFix& msg){
    _fix_data = msg;
    _csv_data.stamp = _fix_data.header.stamp;
    _csv_data.latitude = _fix_data.latitude;
    _csv_data.longitude = _fix_data.longitude;
    _csv_data.altitude = _fix_data.altitude;

    // if (_fix_data.header.stamp.nsec == _pvt_data.nano || _fix_data.header.stamp.nsec == 1000000000 + _pvt_data.nano){
    //     write_csv();
    // }
    write_csv();
}

void Gnss_csv::navpvtCallback(const ublox_msgs::NavPVT& msg){
    _pvt_data = msg;
    _csv_data.numSV = _pvt_data.numSV;
    _csv_data.fixType = _pvt_data.fixType;
    _csv_data.pDOP = _pvt_data.pDOP;
    _csv_data.pDOP = _csv_data.pDOP * 0.01;
    _csv_data.yaw = _pvt_data.heading;
    _csv_data.yaw = _csv_data.yaw/100000;


    if (_fix_data.header.stamp.nsec == _pvt_data.nano || _fix_data.header.stamp.nsec == 1000000000 + _pvt_data.nano){
        write_csv();
    }
}

void Gnss_csv::write_csv(){
    if (_row_num == 0){
        std::vector<std::string> row = {"sec", "nsec", "numSV", "fixType", "pDOP", "latitude", "longitude", "altitude", "yaw", "pitch", "roll"};
        for (int i=0;i<row.size();i++)
        {
            csv_file << row[i]<<",";
        }
        csv_file << std::endl;
        _row_num++;
    }
    else{
        csv_file << std::fixed << std::setprecision(9);
        csv_file << _csv_data.stamp.sec <<",";
        csv_file << _csv_data.stamp.nsec <<",";
        csv_file << _csv_data.numSV <<",";
        csv_file << _csv_data.fixType <<",";
        csv_file << _csv_data.pDOP <<",";
        csv_file << _csv_data.latitude <<",";
        csv_file << _csv_data.longitude <<",";
        csv_file << _csv_data.altitude <<",";
        csv_file << _csv_data.yaw <<",";

        csv_file << std::endl;
        std::cout << std::setprecision(9);
        std::cout << std::fixed;
        std::cout << _row_num << std::endl;
        std::cout << _csv_data << std::endl;
        _row_num++;
    }
}

bool exists_test(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv){
    signal(SIGINT, Siginthandler);
    ros::init(argc, argv, "gnss_csv_filter");
    ros::NodeHandle n;

    std::string dir;
    std::string filename;
    std::string filetype = "_gnss";

    n.param("directory", dir, std::string("/home/meclab/catkin_ws/src/gnss_filter/data/bag/"));
    n.param("filename", filename, std::string("sunrise-20220928162345_L1C1C2G1G2_route_10-1"));

    if(exists_test(dir + filename + filetype + ".csv")){
        std::cout << "\033[31m" << "File exists! Check the file name!" << "\033[0m" << std::endl;
        return 0;
    }

    csv_file.open(dir + filename + filetype + ".csv");
    Gnss_csv gnss_csv;

    ros::Subscriber sub1 = n.subscribe("/ublox_f9k/fix", 1000, &Gnss_csv::fixCallback, &gnss_csv);
    ros::Subscriber sub2 = n.subscribe("/ublox_f9k/navpvt", 1000, &Gnss_csv::navpvtCallback, &gnss_csv);

    ros::Rate loop_rate(1);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}