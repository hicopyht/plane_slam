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

/*******************************************************************************/
using namespace std;

struct RgbdAssociation
{
    RgbdAssociation() : seq_rgb(0), seq_depth(0), file_rgb(""), file_depth(""){}

    int seq_rgb;
    int seq_depth;
    std::string file_rgb;
    std::string file_depth;
};

bool readRgbdAssociations(std::vector<RgbdAssociation> &associations,
                      const std::string &filename)
{

    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read rgbd association file: " << filename << ", exit." );
        exit(1);
    }

    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);
        RgbdAssociation asso;
        ss >> asso.seq_depth >> asso.file_depth >> asso.seq_rgb >> asso.file_rgb;
        // Push
        associations.push_back(asso);
    }

    return true;
}

// For tum format, the first variable is timestamp, which is a double value.
// Here is the sequence.
bool readTUMTrajectory( const std::string &filename, std::map<int,geometry_msgs::PoseStamped> &poses )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);

        int seq;
        double x, y, z, qx, qy, qz, qw;
        ss >> seq >> x >> y >> z >> qx >> qy >> qz >> qw;
        // Push
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        //
        ps.header.seq = seq;
        poses[seq] = ps;
    }

    return true;
}

// Provide the first observed ground plane coefficients in camera coordinate
void calculateInitialPose(Eigen::Vector4f &coefficients, tf::Transform &pose)
{
    //
    Eigen::Vector4f n4(coefficients[0],coefficients[1],coefficients[2], 0);
    Eigen::Vector3f n(coefficients[0],coefficients[1],coefficients[2]);

    // Project origin to plane
    Eigen::Vector4f p (0, 0, 0, 1);
    float distance_to_plane = coefficients.dot (p);
    Eigen::Vector4f origin = p - n4 * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe

    // X,Y,Z axis
    Eigen::Vector3f z(0,0,1);
    Eigen::Vector3f zaxis = n;
    Eigen::Vector3f xaxis = zaxis.cross(z);
    Eigen::Vector3f yaxis = zaxis.cross(xaxis);

    // Get Rotation
    tf::Matrix3x3 m33(xaxis[0],xaxis[1], xaxis[2],
            yaxis[0],yaxis[1], yaxis[2],
            zaxis[0],zaxis[1], zaxis[2]);
    tf::Quaternion quaternion;
    m33.getRotation(quaternion);

    // Set transform
    pose.setOrigin(tf::Vector3(origin[0], origin[1], origin[2]));
    pose.setRotation(quaternion);
    pose = pose.inverse();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_slam_node");

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");

    if( argc < 2)
    {
        cerr << endl << "Usage: ./plane_slam_image setting.yaml <paused>" << endl;
        return 1;
    }
    std::string settingfile = argv[1];
    bool is_paused = false;
    if( argc >= 3 )
    {
        std::string bool_str = argv[2];
        is_paused = !bool_str.compare("true");  // paused mode ?
    }

    //
    cv::FileStorage fs(settingfile, cv::FileStorage::READ);
    CameraParameters camera;
    camera.fx = fs["Camera.fx"];
    camera.fy = fs["Camera.fy"];
    camera.cx = fs["Camera.cx"];
    camera.cy = fs["Camera.cy"];
    camera.width = fs["Camera.width"];
    camera.height = fs["Camera.height"];
    camera.scale = fs["Camera.scale"];
    //
    cout << "/**** Camera Parameters ****/" << endl;
    cout << WHITE << "Camera.fx: " << BLUE << camera.fx << RESET << endl;
    cout << WHITE << "Camera.fy: " << BLUE << camera.fy << RESET << endl;
    cout << WHITE << "Camera.cx: " << BLUE << camera.cx << RESET << endl;
    cout << WHITE << "Camera.cy: " << BLUE << camera.cy << RESET << endl;
    cout << WHITE << "Camera.scale: " << BLUE << camera.scale << RESET << endl;
    cout << WHITE << "Camera.width: " << BLUE << camera.width << RESET << endl;
    cout << WHITE << "Camera.height: " << BLUE << camera.height << RESET << endl;
    cout << "/***************************/"<< endl;
    //
    std::string datapath = fs["Data.path"];
    std::string assofile = fs["Data.associations"];
    std::string posefile = fs["Data.trajectory"];
    std::string file_associations = datapath+"/"+assofile;
    std::string file_truepose = datapath + "/"+posefile;

    // Read associations
    std::vector<RgbdAssociation> associations;
    readRgbdAssociations(associations, file_associations);
    cout << GREEN << "Read associations, size = " << associations.size() << RESET << endl;

    // Read true trajectory
    std::map<int,geometry_msgs::PoseStamped> true_poses;
    readTUMTrajectory(file_truepose, true_poses);
    cout << GREEN << "Read true poses, size = " << true_poses.size() << RESET << endl;


    // Plane slam
    plane_slam::KinectListener kl;

    // Set initial pose
    if(true_poses.size() > 0)
    {
        tf::StampedTransform initial_pose;
        geometry_msgs::PoseStamped ps = true_poses.begin()->second;
        tf::poseMsgToTF(ps.pose, initial_pose);
        kl.setInitPose(initial_pose);
        cout << endl << GREEN << "Set initial pose:" << RESET << endl;
        cout << GREEN << " - position: " << ps.pose.position.x
             << " " << ps.pose.position.y << " " << ps.pose.position.z << RESET << endl;
        cout << GREEN << " - orientation: " << ps.pose.orientation.x
             << " " << ps.pose.orientation.y << " " << ps.pose.orientation.z
             << " " << ps.pose.orientation.w << RESET << endl;
    }else{
        Eigen::Vector4f ground(0.0468333, -0.985658, -0.162125, 1.2848);
        tf::Transform initial_pose;
        calculateInitialPose(ground, initial_pose);
//        kl.setInitPose(initial_pose);
//        cout << endl << GREEN << "Set initial pose from ground plane: "
//             << ground[0] << " " << ground[1] << " " << ground[2] << ground[3] << RESET << endl;
//        cout << GREEN << " - position: " << initial_pose.getOrigin().x()
//             << " " << initial_pose.getOrigin().y() << " " << initial_pose.getOrigin().z() << RESET << endl;
//        cout << GREEN << " - orientation: " << initial_pose.getRotation().x()
//             << " " << initial_pose.getRotation().y() << " " << initial_pose.getRotation().z()
//             << " " << initial_pose.getRotation().w() << RESET << endl;
    }

    // Set camera parameters
    kl.setCameraParameters(camera);
    // Camera cloud
    CameraParameters camera_cloud(camera, 4);
//    camera_cloud.fy = fabs(camera_cloud.fy);
    kl.getLineBasedPlaneSegmentor()->segmentor().setCameraInfo(camera_cloud);

    cout << GREEN << "##############################################################" << RESET << endl;
    cout << GREEN << " Press space to pause." << RESET << endl;
    cout << GREEN << "##############################################################" << RESET << endl;


    // Setup terminal
    setupTerminal();

    // Load all messages
    ros::Rate loop_rate(20);

    // pause
    while(ros::ok())
    {
        char cin = readCharFromStdin();
        if(cin == ' ')
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::namedWindow("Depth Image");
    cv::namedWindow("Rgb Image");

    BOOST_FOREACH(RgbdAssociation const asso, associations)
    {
        // exit
        if(!ros::ok())
            break;

        // pause
        if( is_paused || readCharFromStdin() == ' ')
        {
            while(ros::ok())
            {
                char cin = readCharFromStdin();
                if(cin == ' ')
                    break;
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        //
//        cout << GREEN << " - process depth = " << asso.file_depth << ", rgb = " << asso.file_rgb << RESET << endl;
        //
        std_msgs::Header header_depth, header_rgb;
        header_depth.frame_id = "camera_rgb_optical_frame";
        header_depth.seq = asso.seq_depth;
        header_depth.stamp = ros::Time::now();
        header_rgb = header_depth;
        header_rgb.seq = asso.seq_rgb;
        cv::Mat mdepth, mrgb;
        mdepth = cv::imread(asso.file_depth, cv::IMREAD_ANYDEPTH);
        mrgb = cv::imread(asso.file_rgb, cv::IMREAD_UNCHANGED);
//        cout << " depth = " << mdepth.type() << ", chs = " << mdepth.channels() << ", rows = " << mdepth.rows << ", cols = " << mdepth.cols << endl;
//        cout << " rgb   = " << mrgb.type() << ", chs = " << mrgb.channels() << ", rows = " << mrgb.rows << ", cols = " << mrgb.cols << endl;
        // Check depth mat
        cv::Mat mat;
        if(mdepth.type() != CV_32FC1)
        {
            mdepth.convertTo(mat, CV_32FC1, 1/5000.0);
            mdepth = mat;
        }
        cv::imshow("Depth Image", mdepth);
        cv::imshow("Rgb Image", mrgb);

        cv::waitKey(1);
        //
        cv_bridge::CvImagePtr depth_ptr(new cv_bridge::CvImage(header_depth, "32FC1", mdepth));
        cv_bridge::CvImagePtr visual_ptr(new cv_bridge::CvImage(header_rgb, "bgr8", mrgb));
        kl.trackDepthRgbImage(visual_ptr->toImageMsg(), depth_ptr->toImageMsg(), camera);
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

    cv::destroyAllWindows();

    ros::spin();
}

