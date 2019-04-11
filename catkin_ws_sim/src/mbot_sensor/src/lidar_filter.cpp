#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <iostream>
#include <math.h>
#include "LinkedList.hpp"

using namespace std;

ros::Publisher  filter_pub;                  //雷达消息滤波后的数据帧
sensor_msgs::LaserScan scan_msgs;            //雷达滤波消息
std::string  frame_id;                       //雷达滤波消息帧ID
const float PI = 3.1416926;                  //PI常量
const float sin_zero_point_five = 0.0087265; //sin(0.5)常量
const float sin_one_point_zero  = 0.0174524; //sin(1.0)常量
const float sin_one_point_five  = 0.0261769; //sin(1.5)常量
const float zone_break_threshod = 5.0;       //区域分割的阈值5.0

/**********************************************************************
雷达扫描消息订阅回调处理函数
**********************************************************************/
void lidar_data_callback(const sensor_msgs::LaserScan& lidar_data)
{
    //定义的激光点云数据结构
    int   valid_num     = 0;
    int   seq[360]      = {0};
    float point[360]    = {0};
    float point_x[360]  = {0};
    float point_y[360]  = {0};
    float point_seq_x[360] = {0};
    float point_seq_y[360] = {0};
    float threshod[360] = {0};
    float two_point_distance[360] = {0}; 
    
    //区域分割的分裂点云数据结构
    ListCode infinite_point;
    infinite_point.create_List();
    int list_length = 0;
    float start_num = 0.0;
    float end_num   = 0.0;

    //复制雷达消息数据帧到scan_msgs
    scan_msgs = lidar_data;
    scan_msgs.header.frame_id = frame_id;

    //滤出inf无效的点云数据，提取有效的点云数据
    for(int i=0; i<360; i++)
    {
        if(scan_msgs.ranges[i] != std::numeric_limits<float>::infinity())
        {
            seq[valid_num]   = i;
            point[valid_num] = scan_msgs.ranges[i];
            valid_num++;	    
        }
    }

    //将有效的激光点云数据转换为笛卡尔直角坐标系
    for(int i=0; i<valid_num; i++)
    {
        point_x[i] = point[i] * cos((seq[i] + 179) * PI / 180.0);
        point_y[i] = point[i] * sin((seq[i] + 179) * PI / 180.0);

        point_seq_x[seq[i]] = point_x[i];
        point_seq_y[seq[i]] = point_y[i];
    }

    //进行相邻有效激光点云的动态阈值计算 
    //进行相邻两点的欧几里的实际距离计算
    for(int i=0; i<valid_num-1; i++)
    {
        if(seq[i+1]-seq[i] == 1)
        {
            if(scan_msgs.ranges[seq[i]] >= scan_msgs.ranges[seq[i+1]])
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i+1]] * sin_zero_point_five;
            }
            else
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i]] * sin_zero_point_five;
            }

            two_point_distance[i] =sqrt(fabs(point_x[i] - point_x[i+1]) * fabs(point_x[i] - point_x[i+1]) + \
                                        fabs(point_y[i] - point_y[i+1]) * fabs(point_y[i] - point_y[i+1]));
        }
        else if(seq[i+1]-seq[i] == 2)
        {
            if(scan_msgs.ranges[seq[i]] >= scan_msgs.ranges[seq[i+1]])
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i+1]] * sin_one_point_zero;
            }
            else
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i]] * sin_one_point_zero;
            }

            two_point_distance[i] =sqrt(fabs(point_x[i] - point_x[i+1]) * fabs(point_x[i] - point_x[i+1]) + \
                                        fabs(point_y[i] - point_y[i+1]) * fabs(point_y[i] - point_y[i+1]));
        }
        else if(seq[i+1]-seq[i] == 3)
        {
            if(scan_msgs.ranges[seq[i]] >= scan_msgs.ranges[seq[i+1]])
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i+1]] * sin_one_point_five;
            }
            else
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i]] * sin_one_point_five;
            }

            two_point_distance[i] =sqrt(fabs(point_x[i] - point_x[i+1]) * fabs(point_x[i] - point_x[i+1]) + \
                                        fabs(point_y[i] - point_y[i+1]) * fabs(point_y[i] - point_y[i+1]));
        }
        else
        {
            infinite_point.insert(seq[i]);
            infinite_point.insert(seq[i+1]);

            if(scan_msgs.ranges[seq[i]] >= scan_msgs.ranges[seq[i+1]])
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i+1]] * sin_zero_point_five;
            }
            else
            {
                threshod[i] = 2 * scan_msgs.ranges[seq[i]] * sin_zero_point_five;
            }

            two_point_distance[i] =sqrt(fabs(point_x[i] - point_x[i+1]) * fabs(point_x[i] - point_x[i+1]) + \
                                        fabs(point_y[i] - point_y[i+1]) * fabs(point_y[i] - point_y[i+1]));
        }
    }

    //激光点云数据的首尾相接处理
    if(360 - seq[valid_num-1] + seq[0] == 1)
    {
        if(scan_msgs.ranges[seq[valid_num-1]] >= scan_msgs.ranges[seq[0]])
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[0]] * sin_zero_point_five;
        }
        else
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[valid_num-1]] * sin_zero_point_five;
        }

        two_point_distance[valid_num-1] =sqrt(fabs(point_x[valid_num-1] - point_x[0]) * fabs(point_x[valid_num-1] - point_x[0]) + \
                                              fabs(point_y[valid_num-1] - point_y[0]) * fabs(point_y[valid_num-1] - point_y[0]));
    }
    else if(360 - seq[valid_num-1] + seq[0] == 2)
    {
        if(scan_msgs.ranges[seq[valid_num-1]] >= scan_msgs.ranges[seq[0]])
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[0]] * sin_one_point_zero;
        }
        else
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[valid_num-1]] * sin_one_point_zero;
        }

        two_point_distance[valid_num-1] =sqrt(fabs(point_x[valid_num-1] - point_x[0]) * fabs(point_x[valid_num-1] - point_x[0]) + \
                                              fabs(point_y[valid_num-1] - point_y[0]) * fabs(point_y[valid_num-1] - point_y[0]));
    }
    else if(360 - seq[valid_num-1] + seq[0] == 3)
    {
        if(scan_msgs.ranges[seq[valid_num-1]] >= scan_msgs.ranges[seq[0]])
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[0]] * sin_one_point_five;
        }
        else
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[valid_num-1]] * sin_one_point_five;
        }

        two_point_distance[valid_num-1] =sqrt(fabs(point_x[valid_num-1] - point_x[0]) * fabs(point_x[valid_num-1] - point_x[0]) + \
                                              fabs(point_y[valid_num-1] - point_y[0]) * fabs(point_y[valid_num-1] - point_y[0]));
    }
    else
    {
        infinite_point.insert(seq[0]);
        infinite_point.insert(seq[valid_num-1]);

        if(scan_msgs.ranges[seq[valid_num-1]] >= scan_msgs.ranges[seq[0]])
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[0]] * sin_zero_point_five;
        }
        else
        {
            threshod[valid_num-1] = 2 * scan_msgs.ranges[seq[valid_num-1]] * sin_zero_point_five;
        }

        two_point_distance[valid_num-1] =sqrt(fabs(point_x[valid_num-1] - point_x[0]) * fabs(point_x[valid_num-1] - point_x[0]) + \
                                              fabs(point_y[valid_num-1] - point_y[0]) * fabs(point_y[valid_num-1] - point_y[0]));
    }

    //进行有效激光点云数据的区域细分割处理
    for(int i=0; i<valid_num; i++)
    {
        if(two_point_distance[i] / threshod[i] >= zone_break_threshod)
        {
            if(infinite_point.dataisExist(seq[i]) == false)
            {
                infinite_point.insert(seq[i]);
            }
        }
    }

    //获取的区域点云分割点排序、重复分割点删除和获取分割点数据长度
    infinite_point.bubbleSort();
    infinite_point.deleteSortedRepeatedData();
    infinite_point.listLength(list_length);

    //进行区域分割点的滤波处理，删除相邻过近的分割点
    for(int i=0; i<list_length-1; i++)
    {
        int temp_index_one = 0;
        temp_index_one = i + 1;
        infinite_point.listIndex(i, start_num);
        infinite_point.listIndex(temp_index_one, end_num);
        if(end_num - start_num <= 3 && start_num != 0.0 && end_num != 359.0)
        {
            if(scan_msgs.ranges[start_num-1] == std::numeric_limits<float>::infinity())
            {
                infinite_point.delete_List(start_num);
                for(int j=start_num; j<end_num; j++)
                {
                    scan_msgs.ranges[j] = std::numeric_limits<float>::infinity();
                }
                infinite_point.listLength(list_length);
                i = -1;
            }
            else if(scan_msgs.ranges[end_num+1] == std::numeric_limits<float>::infinity())
            {
                infinite_point.delete_List(end_num);
                for(int j=start_num+1; j<end_num+1; j++)
                {
                    scan_msgs.ranges[j] = std::numeric_limits<float>::infinity();
                }
                infinite_point.listLength(list_length);
                i = -1;
            }
            else
            {
                if(fabs(scan_msgs.ranges[start_num] - scan_msgs.ranges[start_num-1]) >= \
                   fabs(scan_msgs.ranges[end_num] - scan_msgs.ranges[end_num+1]) )
                {
                    infinite_point.delete_List(start_num);
                    for(int j=start_num; j<end_num; j++)
                    {
                        scan_msgs.ranges[j] = std::numeric_limits<float>::infinity();
                    }
                }
                else
                {
                    infinite_point.delete_List(end_num);
                    for(int j=start_num+1; j<end_num+1; j++)
                    {
                        scan_msgs.ranges[j] = std::numeric_limits<float>::infinity();
                    }
                }
                infinite_point.listLength(list_length);
                i = -1;
            }
        }
    }

    //调试显示分割点链表数据
    cout<<"---------------排序后的激光分割点云-----------------"<<endl;
    infinite_point.print();

    //过短激光点云区域的滤波处理
    for(int i=0; i<list_length-1; i++)
    {
        int first_index = 0;
        first_index = i + 1;
        infinite_point.listIndex(i, start_num);
        infinite_point.listIndex(first_index, end_num);
        if(end_num - start_num <= 10.0)
        {
            if(scan_msgs.ranges[start_num+2] == std::numeric_limits<float>::infinity())
            {
                continue;
            }
            else
            {
                if(scan_msgs.ranges[start_num-2] == std::numeric_limits<float>::infinity())
                {
                    infinite_point.delete_List(start_num);
                    scan_msgs.ranges[start_num] = std::numeric_limits<float>::infinity();
                    infinite_point.listLength(list_length);
                }

                if(scan_msgs.ranges[end_num+2] == std::numeric_limits<float>::infinity())
                {
                    infinite_point.delete_List(end_num);
                    scan_msgs.ranges[end_num] = std::numeric_limits<float>::infinity();
                    infinite_point.listLength(list_length);
                }
                for(int j=start_num+1;j<end_num+1;j++)
                {
                    float insert_temp = end_num + 1;
                    infinite_point.update(end_num, insert_temp);
                    scan_msgs.ranges[j] = std::numeric_limits<float>::infinity();
                }
            }
        }
    }

    //调试显示分割点链表数据
    cout<<"------------过短激光点云滤波后的分割点云---------------"<<endl;
    infinite_point.print();

    //激光区域点云的角点特征和直线特征的提取
    for(int i=0; i<list_length-1; i++)
    {
        int first_index = 0;
        first_index = i + 1;
        infinite_point.listIndex(i, start_num);
        infinite_point.listIndex(first_index, end_num);
        float start_point[2] = {point_seq_x[(int)start_num+1],point_seq_y[(int)start_num+1]};
        float end_point[2]   = {point_seq_x[(int)end_num],point_seq_y[(int)end_num]};
        float k = 0.0;
        float b = 0.0;
        float A = 0.0;
        float B = 0.0;
        float C = 0.0;
        float abs_AB = 0.0;
        float d      = 0.0;
        float temp_d = 0.0;
        float insert_node = 0.0;
        if(end_point[0] - start_point[0] != 0.0)
        {
           k = (end_point[1] - start_point[1]) / (end_point[0] - start_point[0]);
           b = start_point[1] - k * start_point[0];
           A = k;
           B = -1;
           C = b;
           abs_AB = sqrt(A*A + B*B);
        }
        for(int j=start_num+1; j<end_num+1; j++)
        {
           if(point_seq_x[j] == 0.0 && point_seq_y[i] == 0.0)
           {
               break;
           }
           d = fabs(A*point_seq_x[j] + B*point_seq_y[j] + C) / abs_AB;
           if(d > temp_d)
           {
               temp_d = d;
               insert_node = j;
           }
        }
        if(temp_d >= 0.05 && insert_node != 0.0)
        {
            infinite_point.insert(insert_node);
            infinite_point.bubbleSort();
            infinite_point.deleteSortedRepeatedData();
            infinite_point.listLength(list_length);
            i = 0;
        }
    }

    //调试显示分割点链表数据
    cout<<"----------增加角点的分割点云--------------"<<endl;
    infinite_point.print();

    //提前将所有激光点云强度信息置零，使RVIZ颜色初始化
    for(int i=0; i<360; i++)
    {
        scan_msgs.intensities[i] = 0.0;
    }
    //设置不同分区颜色在RVIZ上显示
    float intensity = 50.0;
    static bool inf_flag = false;
    infinite_point.listLength(list_length);
    for(int i=0; i<list_length-1; i++)
    {   
        int temp_index  = 0;
        temp_index = i + 1;
        infinite_point.listIndex(i, start_num);
        infinite_point.listIndex(temp_index, end_num);
        if(scan_msgs.ranges[start_num+3] == std::numeric_limits<float>::infinity())
        {
            inf_flag = true;
        }
        else
        {
            for(int j=start_num+1; j<end_num+1; j++)
            {
                if(inf_flag == true)
                {
                    scan_msgs.intensities[start_num] = intensity;
                    inf_flag = false;
                }
                scan_msgs.intensities[j] = intensity;
            }
        }
        if(inf_flag != true)
        {
            intensity += 50.0;
        }
    }

    //设置首尾相接的RVIZ显示处理
    int init_list = 0;
    int list_length_temp = list_length - 1;
    infinite_point.listIndex(list_length_temp, start_num);
    infinite_point.listIndex(init_list, end_num);
    if(start_num < 359)
    {
        if(scan_msgs.ranges[start_num+1] == std::numeric_limits<float>::infinity())
        {
            inf_flag = true;
        }
        else
        {
            for(int i=start_num+1; i<360; i++)
            {
                if(inf_flag == true)
                {
                    scan_msgs.intensities[start_num] = intensity;
                    inf_flag = false;
                }
                scan_msgs.intensities[i] = intensity;
            }

            for(int i=0; i<end_num+1; i++)
            {
                scan_msgs.intensities[i] = intensity;
            }
        }
    }
    else if(start_num == 359)
    {
        if(scan_msgs.ranges[1] == std::numeric_limits<float>::infinity())
        {
            inf_flag = true;
        }
        else
        {
            for(int i=0; i<end_num+1; i++)
            {

                if(inf_flag == true)
                {
                    scan_msgs.intensities[start_num] = intensity;
                    inf_flag = false;
                }
                scan_msgs.intensities[i] = intensity;
            }
        }
    }


    //调试处理
    /*for(int i=0; i<360; i++)
    {
        cout<<"num:"<<i<<"****"<<"distancd:"<<scan_msgs.ranges[i]<<"****"<<"intensity:"<<scan_msgs.intensities[i]<<endl;
    }*/

    /*for(int i=0; i<valid_num; i++)
    {
        cout<<"num:"<<seq[i]<<"****"<<threshod[i]<<"****"<<two_point_distance[i]<<"****"<< two_point_distance[i] / threshod[i] <<endl;
    }*/

    for(int i=0; i<360; i++)
    {
        cout<<"num:"<<i<<"***"<<point_seq_x[i]<<"***"<<point_seq_y[i]<<endl;
    }

    /*for(int i=0; i<valid_num;i++)
    {
        cout<<"num:"<<seq[i]<<"****"<<"point_x:"<<point_x[i]<<"****"<<"point_y:"<<point_y[i]<<endl;
    }*/


    //发布雷达数据消息滤波帧
    filter_pub.publish(scan_msgs);
}


/**********************************************************************
雷达滤波节点的主处理函数
**********************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub_data = nh.subscribe("scan", 1, lidar_data_callback);
    filter_pub = nh.advertise<sensor_msgs::LaserScan>("scan_filter", 1);

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("frame_id", frame_id, "laser_filter");

    ROS_INFO("The lidar filter node is doing.");

    double rate = 50;
    ros::Rate r(rate);

    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}






