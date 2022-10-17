#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include <gnss_filter/ndt_csv.h>

std::ofstream csv_file;

void Siginthandler(int sig){
    std::cout << "Ctrl + C !" << std::endl;
    csv_file.close();
    ros::shutdown();
}

class Ndt_csv{
    private:
        int _row_num = 0;
        geometry_msgs::PoseStamped _pose_data;
        gnss_filter::ndt_csv _csv_data;
    
    public:
        void ndtCallback(const geometry_msgs::PoseStamped& msg);
        void write_csv();
};

void Ndt_csv::ndtCallback(const geometry_msgs::PoseStamped& msg){
    _pose_data = msg;
    _csv_data.stamp = _pose_data.header.stamp;
    _csv_data.x = _pose_data.pose.position.x;
    _csv_data.y = _pose_data.pose.position.y;
    _csv_data.z = _pose_data.pose.position.z;
    _csv_data.orientation_x = _pose_data.pose.orientation.x;
    _csv_data.orientation_y = _pose_data.pose.orientation.y;
    _csv_data.orientation_z = _pose_data.pose.orientation.z;
    _csv_data.orientation_w = _pose_data.pose.orientation.z;

    write_csv();
}


void Ndt_csv::write_csv(){
    if (_row_num == 0){
        std::vector<std::string> row = {"sec", "nsec", "x", "y", "z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"};
        for (int i=0;i<row.size();i++)
        {
            csv_file << row[i]<<",";
        }
        csv_file << std::endl;
        _row_num++;
    }
    else{
        csv_file << _csv_data.stamp.sec <<",";
        csv_file << _csv_data.stamp.nsec <<",";
        csv_file << _csv_data.x <<",";
        csv_file << _csv_data.y <<",";
        csv_file << _csv_data.z <<",";
        csv_file << _csv_data.orientation_x <<",";
        csv_file << _csv_data.orientation_y <<",";
        csv_file << _csv_data.orientation_z <<",";
        csv_file << _csv_data.orientation_w <<",";

        csv_file << std::endl;
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
    ros::init(argc, argv, "ndt_csv_filter");
    ros::NodeHandle n;

    std::string dir;
    std::string filename;
    std::string filetype =  "_ndt";

    n.param("directory", dir, std::string("/home/meclab/catkin_ws/src/gnss_filter/data/bag/"));
    n.param("filename", filename, std::string("sunrise-20220928162345_L1C1C2G1G2_route_10-1"));


    if(exists_test(dir + filename + filetype + ".csv")){
        std::cout << "\033[31m" << "File exists! Check the file name!" << "\033[0m" << std::endl;
        return 0;
    }

    csv_file.open(dir + filename + filetype + ".csv");
    Ndt_csv ndt_csv;

    ros::Subscriber sub1 = n.subscribe("/ndt_pose", 1000, &Ndt_csv::ndtCallback, &ndt_csv);

    ros::Rate loop_rate(1);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}