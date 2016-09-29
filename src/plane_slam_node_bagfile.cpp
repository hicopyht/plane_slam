#include "kinect_listener.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <signal.h>
#include <termios.h>

#include <Eigen/Geometry>


// Terminal
bool    terminal_modified_;
termios orig_flags_;
fd_set  stdin_fdset_;
int     maxfd_;

void setupTerminal()
{
    if (terminal_modified_)
       return;

    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);
    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
}

void restoreTerminal()
{
    if (!terminal_modified_)
        return;

    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);

    terminal_modified_ = false;
}

char readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#else
    fd_set testfd = stdin_fdset_;
#endif

    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
    return EOF;

    return getc(stdin);
}

using namespace std;

void saveBagFile( const std::string &file_name, rosbag::View &view )
{
    cout << BLUE << "Save bagfile: " << file_name << RESET << endl;

    rosbag::Bag bagfile;
    bagfile.open( file_name, rosbag::bagmode::Write );

    // write
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        // image topic
        sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        if( image_msg )
        {
//            std::string topic = m.getTopic();
//            ros::Time time = m.getTime();
//            sensor_msgs::Image image = *image_msg;
            bagfile.write<sensor_msgs::Image>( m.getTopic(), m.getTime(), *image_msg );
        }
    }

    // close
    bagfile.close();

    cout << BLUE << "Done." << RESET << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_slam_node_bagfile");

    ros::NodeHandle nh_;

    if( argc < 3)
    {
        cerr << endl << "Usage: ./plane_slam_node_bagfile camera_parameter_file bagfile <skip> <duration> <paused>" << endl;
        return 1;
    }

    double skip_time = 0;
    double duration = 0;
    bool is_paused = false;
    bool save_bagfile = false;
    if( argc >= 4 )
        skip_time = atof(argv[3]);  // skip time
    if( argc >= 5 )
        duration = atof(argv[4]);   // play duration
    if( argc >= 6 )
    {
        std::string bool_str = argv[5];
        is_paused = !bool_str.compare("true");  // paused mode ?
    }
    if( argc >= 7 )
    {
        std::string bool_str = argv[6];
        save_bagfile = !bool_str.compare("true");   // save bagfile ?
    }


    // Check parameter file
    std::string camera_parameter_file(argv[1]);
    cv::FileStorage fsp(camera_parameter_file.c_str(), cv::FileStorage::READ);
    if(!fsp.isOpened())
    {
       cerr << "Failed to open camera parameter file at: " << camera_parameter_file << endl;
       exit(-1);
    }

    // Get camera parameters
    PlaneFromLineSegment::CAMERA_PARAMETERS camera;
    camera.fx = fsp["camera.fx"];
    camera.fy = fsp["camera.fy"];
    camera.cx = fsp["camera.cx"];
    camera.cy = fsp["camera.cy"];
    camera.scale = fsp["camera.scale"];
    camera.width = fsp["camera.width"];
    camera.height = fsp["camera.height"];
    std::string depth_topic = fsp["depth_topic"];
    std::string rgb_topic = fsp["rgb_topic"];
    std::string use_odom = fsp["use_odom"];
    float init_pose_x = fsp["initPose.x"];
    float init_pose_y = fsp["initPose.y"];
    float init_pose_z = fsp["initPose.z"];
    float init_pose_roll = fsp["initPose.roll"];
    float init_pose_pitch = fsp["initPose.pitch"];
    float init_pose_yaw = fsp["initPose.yaw"];
    //
    cout << GREEN << " Load camera parameters: " << endl;
    cout << "***************************************" << endl;
    cout << "    camera.fx = " << camera.fx << endl;
    cout << "    camera.fy = " << camera.fy << endl;
    cout << "    camera.cx = " << camera.cx << endl;
    cout << "    camera.cy = " << camera.cy << endl;
    cout << "    camera.scale = " << camera.scale << endl;
    cout << "    camera.width = " << camera.width << endl;
    cout << "    camera.height = " << camera.height << endl;
    cout << "***************************************" << RESET << endl;

    // Bag file
    std::string filename = argv[2];
    // Open a bag file
    rosbag::Bag bag;
    bag.open( filename, rosbag::bagmode::Read );
    cout << GREEN << " Open bag file: " << filename << RESET << endl;

    // depth image topic to load
    if( depth_topic.empty() )
        depth_topic = "/camera/depth/image";
    if( rgb_topic.empty() )
        rgb_topic = "/camera/rgb/image_color";
    std::vector<std::string> topics;
    topics.push_back( depth_topic );
    topics.push_back( rgb_topic );

    // Force odom or not
    bool force_odom = false;
    if( !use_odom.compare("true"))
        force_odom = true;

    // Valid duration
    rosbag::View full_view;
    full_view.addQuery( bag );
    const ros::Time initial_time = full_view.getBeginTime();
    const ros::Time start_time = initial_time + ros::Duration( skip_time );
    ros::Time finish_time = full_view.getEndTime();
    if( !duration == 0)
    {
        finish_time = start_time + ros::Duration( duration );
        if( finish_time > full_view.getEndTime() )
            finish_time = full_view.getEndTime();
    }
    else
        duration = (finish_time - start_time).toSec();

    rosbag::View view;
    view.addQuery( bag, rosbag::TopicQuery( topics ), start_time, finish_time );

    std::vector<size_t> topic_sizes(topics.size());
    for( int i = 0; i < topic_sizes.size(); i++)
    {
        rosbag::View topic_view;
        topic_view.addQuery( bag, rosbag::TopicQuery( topics[i] ), start_time, finish_time );
        topic_sizes[i] = topic_view.size();
    }

    // Print info
    cout << GREEN << " Depth image topic: " << depth_topic << ", size = " << topic_sizes[0] << RESET << endl;
    cout << GREEN << " Rgb image topic: " << rgb_topic << ", size = " << topic_sizes[1] << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;
    cout << GREEN << " Skip = " << skip_time << " seconds, play duration = " << duration << RESET << endl;
    cout << GREEN << " Press space to process one message." << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;

    // Save
    if( save_bagfile )
    {
        stringstream ss;
        ss << "bag_" << timeToStr() << ".bag";
        saveBagFile( ss.str(), view);
    }


    plane_slam::KinectListener kl;
    // set camera parameter
    kl.setCameraParameters( camera );
    // Set initial pose
//    tf::Transform init_pose = tf::Transform::getIdentity();
    // TUM3, rgbd near structure texture, skip = 8.0s, msg = 10600
    // -T(xyz) = -1.29492, 1.24776, 0.902971
    // -R(rpy): -2.07778, -0.0293992, 1.99638
//    tf::Transform init_pose( tf::createQuaternionFromRPY(-2.07778, -0.0293992, 1.99638),
//                             tf::Vector3(-1.29492, 1.24776, 0.902971) );

    // building a floor 2 stair 1, -s 1.0
    // Init pose:
    // - R(rpy): -2.38612, 0.26805, -1.43302
    // - T:      0.436192, -0.192014, 1.1524
//    tf::Transform init_pose( tf::createQuaternionFromRPY(-2.38612, 0.26805, -1.43302),
//                                 tf::Vector3(0.436192, -0.192014, 1.1524) );

    tf::Transform init_pose( tf::createQuaternionFromRPY( init_pose_roll, init_pose_pitch, init_pose_yaw ),
                             tf::Vector3( init_pose_x, init_pose_y, init_pose_z ) );
    kl.setInitPose( init_pose );

    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);
    bool paused = is_paused;

    //
    bool valid_depth = false;
    bool valid_rgb = false;
    sensor_msgs::Image::ConstPtr depth_img_msg;
    sensor_msgs::Image::ConstPtr rgb_img_msg;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {   
        // exit
        if(!ros::ok())
            break;

        // pause
        if( readCharFromStdin() == ' ')
        {
            paused = true;
            while(paused && ros::ok())
            {
                char cin = readCharFromStdin();
                if(cin == ' ')
                {
                    paused = false;
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        // depth topic
        if (m.getTopic() == depth_topic || ("/" + m.getTopic() == depth_topic))
        {
            depth_img_msg = m.instantiate<sensor_msgs::Image>();
            valid_depth = true;

            // check if synchronous
            if( valid_depth && valid_rgb
                    && fabs( (depth_img_msg->header.stamp - rgb_img_msg->header.stamp).toSec() ) < 0.015 )
            {
                // pause before processing
                paused = is_paused;
                while(paused && ros::ok())
                {
                    char cin = readCharFromStdin();
                    if(cin == ' ')
                    {
                        paused = false;
                        break;
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                // process frame
                cout << BLUE << "Processing frame " << depth_img_msg->header.seq
                     << ", time = " << (m.getTime() - start_time).toSec() << " / " << duration << " seconds." << RESET << endl;
                kl.trackDepthRgbImage( rgb_img_msg, depth_img_msg, camera );
                valid_depth = false;
                valid_rgb = false;
            }
        }


        // rgb topic
        if (m.getTopic() == rgb_topic || ("/" + m.getTopic() == rgb_topic))
        {
            rgb_img_msg = m.instantiate<sensor_msgs::Image>();
            valid_rgb = true;

            // check if synchronous
            if( valid_depth && valid_rgb
                    && fabs( (depth_img_msg->header.stamp - rgb_img_msg->header.stamp).toSec() ) < 0.015 )
            {
                // pause before processing
                paused = is_paused;
                while(paused && ros::ok())
                {
                    char cin = readCharFromStdin();
                    if(cin == ' ')
                    {
                        paused = false;
                        break;
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                // process frame
                cout << BLUE << "Processing frame " << depth_img_msg->header.seq
                     << ", time = " << (m.getTime() - start_time).toSec() << " / " << duration << " seconds." << RESET << endl;
                kl.trackDepthRgbImage( rgb_img_msg, depth_img_msg, camera );
                valid_depth = false;
                valid_rgb = false;

            }
        }

    }

    // pause before processing
    cout << MAGENTA << "Processing finished. Press ctrl+c to exit." << RESET << endl;
    while( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Restore terminal
    restoreTerminal();
    bag.close();
    ros::spin();
}

/// Get pose from two vector
//    // lm0 coefficents: 0.939228, -0.115946, -0.32312, 0.620913, centroid: -0.314183, 0.00216019, 1.00759
//    // lm1 coefficents: -0.264852, -0.661149, -0.701951, 1.1524, centroid: 0.256239, 0.353235, 1.21232
//    Eigen::Vector3d ny(-0.939228, 0.115946, 0.32312), nz(-0.264852, -0.661149, -0.701951);
//    Eigen::Vector3d nx = ny.cross( nz );
//    nx.normalized(); ny.normalized(); nz.normalized();
//    Eigen::Matrix3d m33;
//    m33.col(0) = nx;
//    m33.col(1) = ny;
//    m33.col(2) = nz;
//    tf::Transform init_pose;
//    init_pose.setBasis( matrixEigen2TF(m33) );
//    init_pose.setOrigin( tf::Vector3( 0.256239, 0.353235, 1.21232 ) );
//    init_pose = init_pose * tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-1.0, 0, 0));
//    init_pose = init_pose.inverse();
//    gtsam::Pose3 pose3 = tfToPose3( init_pose );
//    printPose3( pose3, "Init pose", CYAN );

