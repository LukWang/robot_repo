#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F " 

using namespace Eigen;
using namespace std;

static int get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar(); 
    }

    return ch;
}

void iterate(vector<float>& cam_ee_trans, vector<float>& base_ee_trans, MatrixXf& left, MatrixXf& right)
{
    int rows = 3;
    int cols = 12;
    MatrixXf A = MatrixXf::Zero(rows, cols);
    A(0,0) = base_ee_trans[0];
    A(0,1) = base_ee_trans[1];
    A(0,2) = base_ee_trans[2];
    
    A(1,3) = base_ee_trans[0];
    A(1,4) = base_ee_trans[1];
    A(1,5) = base_ee_trans[2];
    
    A(2,6) = base_ee_trans[0];
    A(2,7) = base_ee_trans[1];
    A(2,8) = base_ee_trans[2];

    A(0, 9) = 1;
    A(1,10) = 1;
    A(2,11) = 1;

    Eigen::Vector3f B;
    B << cam_ee_trans[0], cam_ee_trans[1], cam_ee_trans[2];

    MatrixXf l_component(12, 12);
    Eigen::VectorXf r_component;

    l_component = (A.transpose() * A);
    //cout << "left" << endl << l_component << endl;

    r_component = (A.transpose() * B);
    //cout << "right" << endl << r_component << endl;

    left += l_component;

    right += r_component;

    cout << "iterate finished" << endl;
}

bool getSoluton(MatrixXf& left, MatrixXf& right, MatrixXf& solution)
{
    if(left.fullPivLu().isInvertible())
    {
        solution = left.inverse() * right;
        return true;
    }

    else
        return false;
}

bool getOffset(MatrixXf& solution_offset, tf::TransformListener& marker_position_listener)
{
    tf::StampedTransform offset_transform;
    try
    {
        marker_position_listener.lookupTransform("marker_5", "ee_link", ros::Time(0), offset_transform);
    }

    catch(tf::TransformException ex)
    {
        return false;
    }

    solution_offset(9) = -offset_transform.getOrigin().x();
    solution_offset(10) = -offset_transform.getOrigin().y();
    solution_offset(11) = -offset_transform.getOrigin().z();

    return true;
}

bool saveSolution(string filename, MatrixXf offset, tf::TransformListener& marker_position_listener)
{
    ofstream outStream(filename.c_str());
    //tf::TransformListener marker_position_listener;
    tf::StampedTransform desk_base_transform, desk_marker5_transform, desk_ee_link_transform;
    if(outStream)
    {
        try{
        //marker_position_listener.lookupTransform("marker_0", "base_link", ros::Time(0), desk_base_transform);
        marker_position_listener.lookupTransform("marker_0", "base_link", ros::Time(0), desk_base_transform);
        marker_position_listener.lookupTransform("marker_0", "marker_5", ros::Time(0), desk_marker5_transform);
        marker_position_listener.lookupTransform("marker_0", "ee_link", ros::Time(0), desk_ee_link_transform);
        }
        
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        tf::Quaternion nomalized_q = desk_base_transform.getRotation().normalized();


        outStream << "solution tranlation:" << endl;
        outStream << desk_base_transform.getOrigin().x() << endl;
        outStream << desk_base_transform.getOrigin().y() << endl;
        outStream << desk_base_transform.getOrigin().z() << endl;
        
        outStream << "solution quarternion" << endl;
        
        outStream << nomalized_q.x() << endl;
        outStream << nomalized_q.y() << endl;
        outStream << nomalized_q.z() << endl;
        outStream << nomalized_q.w() << endl;

        outStream << "translation offset" << endl;
        outStream << desk_marker5_transform.getOrigin().x() - desk_ee_link_transform.getOrigin().x() << endl;
        outStream << desk_marker5_transform.getOrigin().y() - desk_ee_link_transform.getOrigin().y() << endl;
        outStream << desk_marker5_transform.getOrigin().z() - desk_ee_link_transform.getOrigin().z() << endl;
        
        return true;
    }
}

int main(int argc, char** argv)
{
    static string solution_save_path = "/home/luk/catkin_ws/src/robot_collaboration/robot_localization/solutions/";
    
    system(STTY_US TTY_PATH);
    
    ros::init(argc, argv, "robot_localization");

    ros::NodeHandle nh;

    ros::Rate rate(10.0);

    tf::TransformListener marker_position_listener;

    static tf::TransformBroadcaster robot_localtion_broadcaster;

    tf::StampedTransform cam_base_transform;
    MatrixXf left = MatrixXf::Zero(12, 12);
    MatrixXf right = MatrixXf::Zero(12, 1);
    MatrixXf solution;
    Quaterniond q;

    bool solutionAvailable = false;
    bool offsetUpdateNeeded = false;

    std::vector<float> cam_ee_trans(3, 0.0f);
    std::vector<float> base_ee_trans(3, 0.0f);

    MatrixXf solution_offset = MatrixXf::Zero(12, 1);
    

    int key;
    while(nh.ok())
    {
        //if(offsetUpdateNeeded)
        {
            if(getOffset(solution_offset, marker_position_listener))
                offsetUpdateNeeded = false;
        }
        
        
        if(solutionAvailable)
        {
            tf::Transform transform;
            tf::Transform transform_rect;
            MatrixXf solution_rect;
            
            solution_rect = solution + solution_offset;
            transform.setOrigin(tf::Vector3(solution(9), solution(10), solution(11)));
            transform_rect.setOrigin(tf::Vector3(solution_rect(9), solution_rect(10), solution_rect(11)));
            
            tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
            
            transform.setRotation(q_tf);
            transform_rect.setRotation(q_tf);
            transform = transform.inverse();
            transform_rect = transform_rect.inverse();
            robot_localtion_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_base"));
            robot_localtion_broadcaster.sendTransform(tf::StampedTransform(transform_rect, ros::Time::now(), "base_link", "camera_base_rect"));
        }
         
        //cout << "Press enter too calculate" << endl;
        key = get_char();
        if (key == 32) //space_pressed
        {
            tf::StampedTransform cam_ee_transform;
            tf::StampedTransform base_ee_transform;
            
            std::vector<float> cam_ee_trans_last(3, 0.0f);

            //get average transform in depth image
            int data_count = 0;
            while(data_count < 10)
            {
                tf::StampedTransform cam_ee_transform_temp;
                tf::StampedTransform base_ee_transform_temp;
                try
                {
                    marker_position_listener.lookupTransform("camera_base","marker_5", ros::Time(0), cam_ee_transform);
                    marker_position_listener.lookupTransform("base_link", "ee_link", ros::Time(0), base_ee_transform);
                }

                catch(tf::TransformException ex)
                {
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }

                float temp1[] = {cam_ee_transform.getOrigin().x(), cam_ee_transform.getOrigin().y(), cam_ee_transform.getOrigin().z()};
                std::vector<float> cam_ee_trans_temp(temp1, temp1+3);
                
                float temp2[] = {base_ee_transform.getOrigin().x(), base_ee_transform.getOrigin().y(), base_ee_transform.getOrigin().z()};
                base_ee_trans[0] = temp2[0];
                base_ee_trans[1] = temp2[1];
                base_ee_trans[2] = temp2[2];
                
                if( cam_ee_trans_last != cam_ee_trans_temp)
                {
                    cam_ee_trans[0] += cam_ee_transform.getOrigin().x();
                    cam_ee_trans[1] += cam_ee_transform.getOrigin().y();
                    cam_ee_trans[2] += cam_ee_transform.getOrigin().z();

                    cam_ee_trans_last = cam_ee_trans_temp;

                    data_count++;
                }
            }

            for(int i = 0; i < 3; i++)
            {
                cam_ee_trans[i] /= 10.0f;
            }
            cout << '(' << cam_ee_trans[0] << cam_ee_trans[1] << cam_ee_trans[2] << ')' << endl;
            iterate(cam_ee_trans, base_ee_trans, left, right);

        }
        else if (key == 105) //'i' pressed
        {
            cin.get();
            if(getSoluton(left, right, solution))
            {
                ROS_INFO("Solution get!\n");

                Matrix3d rotation;
                Matrix3f rotationf;
                
                
                
                rotationf << solution(0), solution(1), solution(2), solution(3), solution(4), solution(5), solution(6), solution(7), solution(8);
                rotation = rotationf.cast<double>();
                

                cout << "rotation: "<< endl << rotation << endl;

                q = rotation;

                cout << "quaternion: " << endl << q.coeffs() << endl;

                cout << "solution" << endl << solution << endl;
                solutionAvailable = true;
                offsetUpdateNeeded = true;
            }
            else
                ROS_ERROR("Solution computation not success!");
        }
        
        else if(key == 115) // 's' is pressed
        {
            string filename = solution_save_path + "solution";
            if(saveSolution(filename, solution_offset, marker_position_listener))
                ROS_INFO("solution saved success!");
            else
                ROS_ERROR("solution save fail! No reference marker detected!");

        } 


        
        else if(key == 3) //ctrl+c
        {
            system(STTY_DEF TTY_PATH);
            break;
        }   
    }

    return 0;
}
