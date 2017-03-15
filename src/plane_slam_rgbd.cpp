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
    ros::init(argc, argv, "plane_slam_node");

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");

    if( argc < 3)
    {
        cerr << endl << "Usage: ./plane_slam_rgbd setting.yaml bagfile <skip> <duration> <paused>" << endl;
        return 1;
    }
    std::string settingfile = argv[1];
    std::string filename = argv[2];
    double skip_time = 0;
    double duration = 0;
    bool is_paused = false;
    if( argc >= 4 )
        skip_time = atof(argv[3]);  // skip time
    if( argc >= 5 )
        duration = atof(argv[4]);   // play duration
    if( argc >= 6 )
    {
        std::string bool_str = argv[5];
        is_paused = !bool_str.compare("true");  // paused mode ?
    }

    cv::FileStorage fs(settingfile, cv::FileStorage::READ);
    double camera_fx = fs["Camera.fx"];
    double camera_fy = fs["Camera.fy"];
    double camera_cx = fs["Camera.cx"];
    double camera_cy = fs["Camera.cy"];
    double camera_width = fs["Camera.width"];
    double camera_height = fs["Camera.height"];
    //
    sensor_msgs::CameraInfo camera;
    camera.K[0] = camera_fx;
    camera.K[4] = camera_fy;
    camera.K[2] = camera_cx;
    camera.K[5] = camera_cy;
    camera.width = camera_width;
    camera.height = camera_height;
    sensor_msgs::CameraInfo::Ptr camera_info_msg = (sensor_msgs::CameraInfo::Ptr)(&camera);

    // Plane slam
    plane_slam::KinectListener kl;

    // Open a bag file
    rosbag::Bag bag;
    bag.open( filename, rosbag::bagmode::Read );
    cout << GREEN << " Open bag file: " << filename << RESET << endl;

    // depth image topic to load
    std::string rgb_topic;
    std::string depth_topic;
    std::string tf_topic = "/tf";
    private_nh.param<string>("topic_image_visual", rgb_topic, "/camera/rgb/image_color");
    private_nh.param<string>("topic_image_depth", depth_topic, "/camera/depth/image");
    //
    std::vector<std::string> topics;
    topics.push_back( rgb_topic );
    topics.push_back( depth_topic );
    topics.push_back( tf_topic );

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

    // View range
    rosbag::View view;
    view.addQuery( bag, rosbag::TopicQuery( topics ), start_time, finish_time );

    std::vector<size_t> topic_sizes(topics.size());
    for( size_t i = 0; i < topic_sizes.size(); i++)
    {
        rosbag::View topic_view;
        topic_view.addQuery( bag, rosbag::TopicQuery( topics[i] ), start_time, finish_time );
        topic_sizes[i] = topic_view.size();
    }

    // Print info
    cout << GREEN << " Rgb image topic: " << rgb_topic << ", size = " << topic_sizes[0] << RESET << endl;
    cout << GREEN << " Depth image topic: " << depth_topic << ", size = " << topic_sizes[1] << RESET << endl;
    cout << GREEN << " TF topic: " << tf_topic << ", size = " << topic_sizes[2] << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;
    cout << GREEN << " Skip = " << skip_time << " seconds, play duration = " << duration << RESET << endl;
    cout << GREEN << " Press space to pause." << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;


    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);
    bool paused = is_paused;


    //
    double dt = 0.015;
    bool valid_rgb = false;
    bool valid_depth = false;
    //
    sensor_msgs::Image::ConstPtr rgb_img_msg;
    sensor_msgs::Image::ConstPtr depth_img_msg;
    tf2_msgs::TFMessage::ConstPtr tf_msg;

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

        // tf topic
        if ( m.getTopic() == "/tf" || ("/" + m.getTopic()) == "/tf")
        {
            tf_msg = m.instantiate<tf2_msgs::TFMessage>();

            for( size_t i = 0; i < tf_msg->transforms.size(); i++)
            {
                tf::StampedTransform stamped_tf;
                tf::transformStampedMsgToTF(tf_msg->transforms[i], stamped_tf);
                kl.setTransform(stamped_tf);
            }

        }

        // rgb topic
        if (m.getTopic() == rgb_topic || ("/" + m.getTopic() == rgb_topic))
        {
            rgb_img_msg = m.instantiate<sensor_msgs::Image>();
            valid_rgb = true;
        }

        // depth topic
        if (m.getTopic() == depth_topic || ("/" + m.getTopic() == depth_topic))
        {
            depth_img_msg = m.instantiate<sensor_msgs::Image>();
            valid_depth = true;
        }


        // check if valid frame
        if( valid_depth && valid_rgb
                && fabs( (depth_img_msg->header.stamp - rgb_img_msg->header.stamp).toSec() ) < dt )
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
//            cout << BOLDWHITE << "Process frame: " << CYAN << rgb_img_msg->header.seq << RESET <<endl;
            kl.noCloudCallback(rgb_img_msg, depth_img_msg, camera_info_msg);

            //
            valid_rgb = false;
            valid_depth = false;
        }

    }

    // close bag file
    bag.close();

    // pause before processing
    cout << MAGENTA << "Processing finished. Press ctrl+c to exit." << RESET << endl;
    while( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Restore terminal
    restoreTerminal();

    ros::spin();
}
