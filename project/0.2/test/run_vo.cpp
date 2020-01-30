// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/viz.hpp> 

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] ); // 单例对象实例化
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) ); // 把字符串转换成浮点数
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );
    
    // 没有安装opencv的viz模块，因此此处先注释掉以下代码
    // visualization
    // cv::viz::Viz3d vis("Visual Odometry");
    // cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    // cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    // cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    // vis.setViewerPose( cam_pose );
    
    // world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    // camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    // vis.showWidget( "World", world_coor );
    // vis.showWidget( "Camera", camera_coor );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    // 对于每一输入帧进行vo操作
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        // 初始化
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame ); // 估计位姿
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        
        // 丢失则退出
        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Tcw = pFrame->T_c_w_.inverse(); // 获得求到的位姿
        
        // 本来还会显示世界坐标系和相机坐标系的可视化图
        // show the map and the camera pose 
        // cv::Affine3d M(
        //     cv::Affine3d::Mat3( 
        //         Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
        //         Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
        //         Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
        //     ), 
        //     cv::Affine3d::Vec3(
        //         Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
        //     )
        // );
        
        cv::imshow("image", color );
        cv::waitKey(1);
        // vis.setWidgetPose( "Camera", M);
        // vis.spinOnce(1, false);
    }

    return 0;
}
