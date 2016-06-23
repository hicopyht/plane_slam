#include "kinect_listener.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <signal.h>
#include <termios.h>


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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_slam_node_depth");

    if( argc != 4)
    {
        cerr << endl << "Usage: ./plane_slam_node_depth camera_parameter_file bagfile topicname" << endl;
        return 1;
    }

    const double skip_time = 8.0;
    const double duration = 10.0;

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
    std::string topic_name = argv[3];
    std::vector<std::string> topics;
    topics.push_back(topic_name);
    cout << GREEN << " Depth image topic name: " << topic_name << RESET << endl;

    rosbag::View full_view;
    full_view.addQuery( bag );
    const ros::Time initial_time = full_view.getBeginTime();
    const ros::Time start_time = initial_time + ros::Duration( skip_time );
    const ros::Time finish_time = start_time + ros::Duration( duration );

    rosbag::View view;
    view.addQuery( bag, rosbag::TopicQuery( topics ), start_time, finish_time );

    KinectListener kl;

    cout << GREEN << "##############################################################" << RESET << endl;
    cout << GREEN << " Skip = " << skip_time << " seconds, play duration = " << duration << RESET << endl;
    cout << GREEN << " Press space to process one message." << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;

    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);
    bool paused = true;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {   
        if (m.getTopic() == topic_name || ("/" + m.getTopic() == topic_name))
        {
            // pause
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
            // exit
            if(!ros::ok())
                break;
            // get message
            sensor_msgs::Image::ConstPtr depth_img_msg = m.instantiate<sensor_msgs::Image>();
//            cout << BLUE << " - depth image topic: " << depth_img_msg->header.seq << RESET << endl;
            // process one cloud message
            kl.trackDepthImage( depth_img_msg, camera );
            // Reset symbol
            paused = true;
        }
    }

    // Restore terminal
    restoreTerminal();
    bag.close();
    ros::spin();
}



