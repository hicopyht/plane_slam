#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <cstdio>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> NoCloudSyncPolicy;

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


class RecordSeqStamp
{
public:
    RecordSeqStamp()
        : nh_()
        , private_nh_("~")
        , it_(nh_)
    {
        save_sequence_ss_ = nh_.advertiseService("save_sequence", &RecordSeqStamp::saveDataCallback, this);
        image_sub_ = it_.subscribe("image", 10, &RecordSeqStamp::imageCallback, this);
        ROS_INFO_STREAM("Subscribe to image topic: " << image_sub_.getTopic() << ".");
    }

protected:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        sequences_.push_back(msg->header.seq);
        stamps_.push_back(msg->header.stamp.toSec());

        ROS_INFO_STREAM(" " << image_sub_.getTopic() << " frame id: " << msg->header.frame_id);
    }

    bool saveDataCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string filename = "/home/lizhi/bags/result/ORB_SLAM2/sequences_"+timeToStr()+".txt";

        FILE* yaml = std::fopen( filename.c_str(), "w" );
        fprintf( yaml, "# %s\n", filename.c_str() );
        fprintf( yaml, "# format: sequence stamp\n" );
        fprintf( yaml, "# size: %d\n", (int)(sequences_.size()) );
        // Save Data
        for( size_t i = 0; i < sequences_.size(); i++)
        {
            fprintf( yaml, "%d %f \n", sequences_[i], stamps_[i]);
        }

        // close
        fclose(yaml);

        res.message = "Save sequence data to file: " + filename;
        res.success = true;

        cout << "Save sequence to file: " << filename << endl;
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::ServiceServer save_sequence_ss_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    //
    std::vector<int> sequences_;
    std::vector<double> stamps_;
};


class RecordRgbdStamp{
public:
    RecordRgbdStamp()
        : nh_()
        , private_nh_("~")
        , topic_image_visual_("/head_kinect/rgb/image_rect_color")
        , topic_image_depth_("/head_kinect/depth_registered/image")
        , subscriber_queue_size_(4)
    {
        save_sequence_ss_ = nh_.advertiseService("save_sequence", &RecordRgbdStamp::saveDataCallback, this);

        //No cloud, use visual image, depth image, camera_info
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        depth_sub_ = new image_sub_type (nh_, topic_image_depth_, subscriber_queue_size_);
        no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *depth_sub_),
        no_cloud_sync_->registerCallback(boost::bind(&RecordRgbdStamp::noCloudCallback, this, _1, _2));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_image_depth_ << ".");
    }

protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg, const sensor_msgs::ImageConstPtr& depth_img_msg)
    {
        rgb_sequences_.push_back(visual_img_msg->header.seq);
        rgb_stamps_.push_back(visual_img_msg->header.stamp.toSec());
        depth_sequences_.push_back(depth_img_msg->header.seq);
        depth_stamps_.push_back(depth_img_msg->header.stamp.toSec());

        ROS_INFO_STREAM(" - RGB-D: " << visual_img_msg->header.seq << " " << visual_img_msg->header.stamp.toSec()
                        << " \t" << depth_img_msg->header.seq << " " << depth_img_msg->header.stamp.toSec() << ".");
    }

    bool saveDataCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        saveData2();
        res.message = "Save sequence data to file.";
        res.success = true;
        cout << "Save sequence to file."<< endl;
        return true;
    }

private:
    void saveData()
    {
        std::string filename = "/home/lizhi/bags/result/ORB_SLAM2/sequences_rgb_depth"+timeToStr()+".txt";

        FILE* yaml = std::fopen( filename.c_str(), "w" );
        fprintf( yaml, "# %s\n", filename.c_str() );
        fprintf( yaml, "# format: rgbSequence rgbStamp depthSequence depthStamp\n" );
        fprintf( yaml, "# size: %d\n", (int)(rgb_sequences_.size()) );
        // Save Data
        for( size_t i = 0; i < rgb_sequences_.size(); i++)
        {
            fprintf( yaml, "%d %f %d %f\n", rgb_sequences_[i], rgb_stamps_[i], depth_sequences_[i], depth_stamps_[i]);
        }

        // close
        fclose(yaml);
    }

    void saveData2()
    {
        std::string filename = "/home/lizhi/bags/result/ORB_SLAM2/depth_sequence_rgb_stamp"+timeToStr()+".txt";

        FILE* yaml = std::fopen( filename.c_str(), "w" );
        fprintf( yaml, "# %s\n", filename.c_str() );
        fprintf( yaml, "# format: depthSequence rgbStamp\n" );
        fprintf( yaml, "# size: %d\n", (int)(depth_sequences_.size()) );
        // Save Data
        for( size_t i = 0; i < rgb_sequences_.size(); i++)
        {
            fprintf( yaml, "%d %f\n", depth_sequences_[i], rgb_stamps_[i]);
        }

        // close
        fclose(yaml);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::ServiceServer save_sequence_ss_;
    //
    std::string topic_image_visual_;
    std::string topic_image_depth_;
    int subscriber_queue_size_;
    //
    message_filters::Subscriber<sensor_msgs::Image> *visual_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    //
    std::vector<int> rgb_sequences_;
    std::vector<int> depth_sequences_;
    std::vector<double> rgb_stamps_;
    std::vector<double> depth_stamps_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgb_seq_stamp");
//    RecordSeqStamp rss;
    RecordRgbdStamp rrs;
    ros::spin();
}
