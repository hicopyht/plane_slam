#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <vector>
#include <cstdio>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

std::string timeToStr()
{
    std::stringstream msg;
    const boost::posix_time::ptime now=
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f=
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}


class GMappingPose
{
public:
    GMappingPose()
        : nh_()
        , private_nh_("~")
        , scan_topic_("/scan")
        , source_frame_("/head_kinect_depth_frame")
        , target_frame_("/map")
        , child_frame_id_("odom_combined")
        , frame_id_("map")
    {
        save_data_ss_ = nh_.advertiseService("save_data", &GMappingPose::saveDataCallback, this );
        tf_sub_ = nh_.subscribe("tf", 100, &GMappingPose::tfCallback, this);
        scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 30);
        scan_sub_->registerCallback(&GMappingPose::scanCallback, this);

        ROS_INFO_STREAM("Subscribe to scan topic: " << scan_sub_->getTopic() << ".");
    }


protected:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        ROS_INFO_STREAM("Scan message " << msg->header.seq << ", id = " << ((int)msg->header.seq-1));

        vector_of_seq_.push_back(msg->header.seq-1);
        unique_lock<mutex> lock(tf_mutex_);
        vector_of_transform_.push_back(transform_);
    }

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    {
        for( size_t i = 0; i < msg->transforms.size(); i++)
        {
            const geometry_msgs::TransformStamped &transform = msg->transforms[i];
            if(transform.header.frame_id == frame_id_ && transform.child_frame_id == child_frame_id_)
            {
                ROS_INFO_STREAM("TF message " << transform.header.seq);
                unique_lock<mutex> lock(tf_mutex_);
                transform_ = transform.transform;
                break;
            }
        }
    }

    bool saveDataCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string filename = "/home/lizhi/map_to_odom"+timeToStr()+".txt";

        FILE* yaml = std::fopen( filename.c_str(), "w" );
        fprintf( yaml, "# %s\n", filename.c_str() );
        fprintf( yaml, "# transform format: seq T(xyz) Q(xyzw)\n" );
//        fprintf( yaml, "# transform format: seq T(xy) Q(yaw)\n" );
        fprintf( yaml, "# size: %d\n", (int)(vector_of_transform_.size()) );
        // Save Data
        for( size_t i = 0; i < vector_of_transform_.size(); i++)
        {
            geometry_msgs::Transform &tr = vector_of_transform_[i];
            fprintf( yaml, "%d %f %f %f %f %f %f %f \n", vector_of_seq_[i], tr.translation.x, tr.translation.y, tr.translation.z,
                     tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w);
        }
//        for( size_t i = 0; i < vector_of_transform_.size(); i++)
//        {
//            geometry_msgs::Transform &tr = vector_of_transform_[i];
//            double yaw = tf::getYaw(tr.rotation);
//            fprintf( yaml, "%d %f %f %f\n", vector_of_seq_[i], tr.translation.x, tr.translation.y, yaw);
//        }

        // close
        fclose(yaml);

        res.message = "Save data to file: " + filename;
        res.success = true;

        cout << "Save data to file: " << filename << endl;
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::ServiceServer save_data_ss_;
    ros::Subscriber tf_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> *scan_sub_;
    //
    std::string scan_topic_;
    std::string source_frame_;
    std::string target_frame_;
    //
    std::string child_frame_id_;
    std::string frame_id_;
    //
    std::mutex tf_mutex_;
    geometry_msgs::Transform transform_;
    //
    std::vector<geometry_msgs::Transform> vector_of_transform_;
    std::vector<int> vector_of_seq_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gmapping_pose");
    GMappingPose gmp;
    ros::spin();
}
