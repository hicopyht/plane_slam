#include "plane_slam.h"

/*
 * Useage:
 * 1. call function 'initialize' to register first observation.
 * 2. call function 'planeSlam' to process every observation.
 * 3. publishPath() to publish estimate path
 */

PlaneSlam::PlaneSlam() :
    private_nh_("~")
  , isam2_parameters_()
  , graph_()
  , initial_estimate_()
  , first_pose_(true)
  , pose_count_( 0 )
  , landmark_count_( 0 )
  , plane_match_direction_threshold_( 0.1 )
  , plane_match_distance_threshold_( 0.05 )
  , plane_inlier_leaf_size_( 0.02f )
  , plane_hull_alpha_( 0.5 )
{
    isam2_parameters_.relinearizeThreshold = 0.05;
    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    //
    path_publisher_ = nh_.advertise<nav_msgs::Path>("camera_path", 10);
    odom_path_publisher_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void PlaneSlam::initialize(Pose3 &init_pose, std::vector<PlaneType> &planes)
{
    if(planes.size() == 0)
        return;

    // Add a prior factor
    pose_count_ = 0;
    Key x0 = Symbol('x', 0);
    Vector pose_sigmas(6);
    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );
    // Add to estimated pose
    estimated_poses_.push_back( init_pose );
    pose_count_++;

    // Add a prior landmark
    Key l0 = Symbol('l', 0);
    OrientedPlane3 lm0(planes[0].coefficients);
    OrientedPlane3 glm0 = lm0.transform(init_pose.inverse());
    noiseModel::Diagonal::shared_ptr lm_noise = noiseModel::Diagonal::Sigmas( (Vector(2) << planes[0].sigmas[0], planes[0].sigmas[1]).finished() );
    graph_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );

    // Add odom pose to path
    odom_pose_ = init_pose;
    odom_poses_.clear();
    odom_poses_.push_back( odom_pose_ );

    landmark_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', i);

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        OrientedPlane3 glmn = lmn.transform( init_pose.inverse() );
        initial_estimate_.insert<OrientedPlane3>( ln,  glmn );

        // Add to estimated plane
        estimated_planes_.push_back( glmn );

        // Add to landmarks buffer
        PlaneType global_plane = plane;
        global_plane.coefficients = glmn.planeCoefficients();
        Eigen::Matrix4d transform = init_pose.matrix();
        transformPointCloud( *plane.cloud, *global_plane.cloud, transform );
        transformPointCloud( *plane.cloud_boundary, *global_plane.cloud_boundary, transform );
        transformPointCloud( *plane.cloud_hull, *global_plane.cloud_hull, transform );
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *global_plane.cloud_boundary, cen );
        global_plane.centroid.x = cen[0];
        global_plane.centroid.y = cen[1];
        global_plane.centroid.z = cen[2];
        landmarks_.push_back( global_plane );

        //
        landmark_count_ ++;
    }
    isam2_->update( graph_, initial_estimate_ );
    last_estimate_pose_ = init_pose;
    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    //
    first_pose_ = false;

    cout << GREEN << "Initialize plane slam, register first observation." << endl;
    initial_estimate_.print(" - Initial estimate: ");
    cout << RESET << endl;
}

Pose3 PlaneSlam::planeSlam(Pose3 &odom_pose, std::vector<PlaneType> &planes)
{
    if(first_pose_)
    {
        ROS_ERROR("You should call initialize() before doing slam.");
        exit(1);
    }
    // calculate relative pose
    Pose3 rel_pose = odom_pose_.inverse() * odom_pose;

    // convert to gtsam plane type
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        observations.push_back( OrientedPlane3(planes[i].coefficients) );
    }

    // Add odometry factors
    Vector odom_sigmas(6);
    odom_sigmas << rel_pose.translation().vector()*0.2, rel_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', pose_count_);
    Key last_key = Symbol('x', pose_count_-1);
    graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    Pose3 new_pose = last_estimate_pose_ * rel_pose;
    initial_estimate_.insert<Pose3>( pose_key, new_pose );
    pose_count_ ++;

    // Add odom_pose
    odom_pose_ = odom_pose;
    odom_poses_.push_back( odom_pose_ );

    // Transform modeled landmakr to pose frame
    std::vector<OrientedPlane3> predicted_observations;
    predictObservation( estimated_planes_, new_pose, predicted_observations);
//    getPredictedObservation( new_pose, predicted_observations );

    // Match modelled landmark with measurement
    std::vector<PlanePair> pairs;
    matchPlanes( predicted_observations, observations, pairs);

    // Add factor to exist landmark
    Eigen::VectorXd unpairs = Eigen::VectorXd::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        Key ln = Symbol('l', pair.ilm);
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }
    cout << GREEN << " find pairs: " << pairs.size() << RESET << endl;

    // Add new landmark
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            Key ln = Symbol('l', landmark_count_);
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            OrientedPlane3 glmn = lmn.transform( new_pose.inverse() );
            initial_estimate_.insert<OrientedPlane3>( ln, glmn );

            // Add to landmarks buffer
            PlaneType global_plane = obs;
            global_plane.coefficients = glmn.planeCoefficients();
            Eigen::Matrix4d transform = new_pose.matrix();
            transformPointCloud( *obs.cloud, *global_plane.cloud, transform );
            transformPointCloud( *obs.cloud_boundary, *global_plane.cloud_boundary, transform );
            transformPointCloud( *obs.cloud_hull, *global_plane.cloud_hull, transform );
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *global_plane.cloud_boundary, cen );
            global_plane.centroid.x = cen[0];
            global_plane.centroid.y = cen[1];
            global_plane.centroid.z = cen[2];
            landmarks_.push_back( global_plane );

            //
            landmark_count_ ++;
        }
    }

    cout << GREEN << " lm number: " << landmark_count_ << RESET << endl;

    // Update graph
    cout << "update " << endl;
    isam2_->update(graph_, initial_estimate_);
    // isam2->update(); // call additionally

    // Update estimated poses and planes
    Values values = isam2_->calculateBestEstimate();
    estimated_poses_.clear();
    for(int i = 0; i < pose_count_; i++)
    {
        Key xn = Symbol('x', i);
        Pose3 pose = values.at(xn).cast<Pose3>();
        estimated_poses_.push_back( pose );
    }
    estimated_planes_.clear();
    for(int i = 0; i < landmark_count_; i++)
    {
        Key ln = Symbol('l', i);
        OrientedPlane3 predicted = values.at(ln).cast<OrientedPlane3>();
        estimated_planes_.push_back( predicted );
    }

    // Update pose
    cout << pose_count_ << " get current est " << endl;
    Pose3 current_estimate = values.at( pose_key ).cast<Pose3>();
//    Pose3 current_estimate = isam2_->calculateEstimate( pose_key ).cast<Pose3>();

    // Update landmarks
    updateLandmarks( landmarks_, planes, pairs, new_pose, current_estimate, estimated_planes_ );

    // Print estimate pose:
    cout << BLUE;
    current_estimate.print("Current estimate:");
    cout << RESET << endl;

    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    last_estimate_pose_ = current_estimate;

    return current_estimate;
}

// simple euclidian distance
void PlaneSlam::matchPlanes( std::vector<OrientedPlane3> &predicted_observations,
                             std::vector<OrientedPlane3> &observations,
                             std::vector<PlanePair> &pairs)
{
    Eigen::VectorXd paired = Eigen::VectorXd::Zero( predicted_observations.size() );
    for( int i = 0; i < observations.size(); i++)
    {
        OrientedPlane3 &obs = observations[i];
        double min_d = 1e2;
        int min_index = -1;
        for( int l = 0; l < predicted_observations.size(); l++)
        {
            if( paired[l] )
                continue;

            OrientedPlane3 &lm = predicted_observations[l];
//            Vector3 error = obs.errorVector( lm );
            Vector3 error = obs.error( lm );
            double dir_error = acos( cos(error[0])*cos(error[1]));
            double dis_error = fabs( error[2] );
            double d = fabs(dir_error) + dis_error;
            cout << YELLOW << "  - " << i << "*" << l << ": " << dir_error << ", " << dis_error << RESET << endl;
            if( (fabs(dir_error) < plane_match_direction_threshold_)
                    && (dis_error < plane_match_distance_threshold_) && (d < min_d) )
            {
                min_d = d;
                min_index = l;
            }
        }
        if( min_index >= 0 )
        {
            paired[min_index] = 1;
            pairs.push_back( PlanePair(i, min_index) );
        }
    }
}

// get predicted landmarks
void PlaneSlam::getPredictedObservation( Pose3 &pose, std::vector<OrientedPlane3> &predicted_observations )
{
    Values values = isam2_->calculateBestEstimate();

    for(int i = 0; i < landmark_count_; i++)
    {
        Key ln = Symbol('l', i);
        OrientedPlane3 predicted = values.at(ln).cast<OrientedPlane3>();
        predicted_observations.push_back( predicted.transform( pose ) );
    }
}

// get predicted landmarks
void PlaneSlam::predictObservation( std::vector<OrientedPlane3> &landmarks, Pose3 &pose,
                                    std::vector<OrientedPlane3> &predicted_observations)
{
    predicted_observations.clear();
    for(int i = 0; i < landmarks.size(); i++)
    {
        predicted_observations.push_back( landmarks[i].transform( pose ) );
    }
}

void PlaneSlam::updateLandmarks( std::vector<PlaneType> &landmarks,
                                 const std::vector<PlaneType> &observations,
                                 const std::vector<PlanePair> &pairs,
                                 const Pose3 &odom_pose,
                                 const Pose3 &estimated_pose,
                                 const std::vector<OrientedPlane3> &estimated_planes)

{
    if( landmarks.size() != estimated_planes.size() )
    {
        cout << RED << "[Error]: landmark.size() != estimated_planes.size()" << RESET << endl;
        return;
    }

    Eigen::Matrix4d transform = estimated_pose.matrix();

    for( int i = 0; i < landmarks.size(); i++)
    {
        PlaneType &lm = landmarks[i] ;
        lm.coefficients = estimated_planes[i].planeCoefficients();
    }

    for( int i = 0; i < pairs.size(); i++)
    {
        const int iobs = pairs[i].iobs;
        const int ilm = pairs[i].ilm;
        const PlaneType &obs = observations[ iobs ];
        PlaneType &lm = landmarks[ ilm ] ;

        PointCloudTypePtr cloud( new PointCloudType );
        PointCloudTypePtr cloud_boundary( new PointCloudType );
        PointCloudTypePtr cloud_hull( new PointCloudType );

        // transform cloud
        transformPointCloud( *obs.cloud, *cloud, transform );
        transformPointCloud( *obs.cloud_boundary, *cloud_boundary, transform );
        transformPointCloud( *obs.cloud_hull, *cloud_hull, transform );

        // sum
        *lm.cloud += *cloud;
        *lm.cloud_boundary += *cloud_boundary;
        *lm.cloud_hull += *cloud_hull;

        // project
        projectPoints( *lm.cloud, lm.coefficients, *cloud );
//        projectPoints( *lm.cloud_boundary, lm.coefficients, *cloud_boundary );
//        projectPoints( *lm.cloud_hull, lm.coefficients, *cloud_hull );

        // refresh inlier, boundary, hull
        passThoughFilter( cloud, lm.cloud, plane_inlier_leaf_size_ );
//        cloudHull( cloud_boundary, lm.cloud_boundary );
        cloudHull( cloud, lm.cloud_hull );

        // compute new centroid
        Eigen::Vector4d cen;
        pcl::compute3DCentroid( *lm.cloud_hull, cen);
        lm.centroid.x = cen[0];
        lm.centroid.y = cen[1];
        lm.centroid.z = cen[2];
    }
}

void PlaneSlam::publishEstimatedPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < estimated_poses_.size(); i++)
    {
        Pose3 &pose = estimated_poses_[i];
        geometry_msgs::PoseStamped p;
        p.pose.position.x = pose.x();
        p.pose.position.y = pose.y();
        p.pose.position.z = pose.z();
        Eigen::Vector4d quater = pose.rotation().quaternion();
        p.pose.orientation.w = quater[0];
        p.pose.orientation.x = quater[1];
        p.pose.orientation.y = quater[2];
        p.pose.orientation.z = quater[3];
        path.poses.push_back( p );
    }
    path_publisher_.publish( path );
    cout << GREEN << "Publisher path, p = " << estimated_poses_.size() << RESET << endl;
}

void PlaneSlam::publishOdomPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < odom_poses_.size(); i++)
    {
        Pose3 &pose = odom_poses_[i];
        geometry_msgs::PoseStamped p;
        p.pose.position.x = pose.x();
        p.pose.position.y = pose.y();
        p.pose.position.z = pose.z();
        Eigen::Vector4d quater = pose.rotation().quaternion();
        p.pose.orientation.w = quater[0];
        p.pose.orientation.x = quater[1];
        p.pose.orientation.y = quater[2];
        p.pose.orientation.z = quater[3];
        path.poses.push_back( p );
    }
    odom_path_publisher_.publish( path );
    cout << GREEN << "Publisher odom path, p = " << odom_poses_.size() << RESET << endl;
}

void PlaneSlam::passThoughFilter( const PointCloudTypePtr &cloud,
                                  PointCloudTypePtr &cloud_filtered,
                                  float leaf_size)
{
    // Create the filtering object
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
    sor.filter ( *cloud_filtered );
}

void PlaneSlam::extractPlaneHulls(const PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        // projected cloud
        Eigen::Vector4f model_coefficients;
        model_coefficients[0] = plane.coefficients[0];
        model_coefficients[1] = plane.coefficients[1];
        model_coefficients[2] = plane.coefficients[2];
        model_coefficients[3] = plane.coefficients[3];
        projectPoints( *input, plane.inlier, model_coefficients, *(plane.cloud) );
        // hull
        cloudHull( plane.cloud, plane.cloud_hull );
    }
}

void PlaneSlam::projectPoints ( const PointCloudType &input,
                                const Eigen::Vector4d &model_coefficients,
                                PointCloudType &projected_points )
{
    Eigen::Vector4f coefficients;
    coefficients[0] = model_coefficients[0];
    coefficients[1] = model_coefficients[1];
    coefficients[2] = model_coefficients[2];
    coefficients[3] = model_coefficients[3];
    projectPoints( input, coefficients, projected_points );
}

void PlaneSlam::projectPoints ( const PointCloudType &input,
                                const Eigen::Vector4f &model_coefficients,
                                PointCloudType &projected_points )
{
    projected_points.header = input.header;
    projected_points.is_dense = input.is_dense;

    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

    // normalize the vector perpendicular to the plane...
    mc.normalize ();
    // ... and store the resulting normal as a local copy of the model coefficients
    Eigen::Vector4f tmp_mc = model_coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    projected_points.points.resize (input.size ());
    projected_points.width    = static_cast<uint32_t> ( input.size() );
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < input.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input.points[i], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < input.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input.points[i].x,
                            input.points[i].y,
                            input.points[i].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
        pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

void PlaneSlam::projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                                const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points )
{
    Eigen::Vector4f coefficients;
    coefficients[0] = model_coefficients[0];
    coefficients[1] = model_coefficients[1];
    coefficients[2] = model_coefficients[2];
    coefficients[3] = model_coefficients[3];
    projectPoints( input, inlier, coefficients, projected_points );
}

void PlaneSlam::projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                                const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points )
{
    projected_points.header = input.header;
    projected_points.is_dense = input.is_dense;

    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

    // normalize the vector perpendicular to the plane...
    mc.normalize ();
    // ... and store the resulting normal as a local copy of the model coefficients
    Eigen::Vector4f tmp_mc = model_coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    projected_points.points.resize (inlier.size ());
    projected_points.width    = static_cast<uint32_t> (inlier.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inlier.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input.points[inlier[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inlier.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input.points[inlier[i]].x,
                            input.points[inlier[i]].y,
                            input.points[inlier[i]].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
        pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

//void PlaneSlam::projectPoints( const PointCloudTypePtr &input, std::vector<int> &inliers,
//                               Eigen::Vector4d &coeffs, PointCloudTypePtr &output)
//{
//    pcl::ProjectInliers<PointType> proj;
//    proj.setModelType ( pcl::SACMODEL_PLANE );
//    pcl::PointIndicesPtr indices;
//    for(int i = 0; i < inliers.size(); i++)
//    {
//        indices->indices.push_back( inliers[i] );
//    }
//    proj.setIndices ( indices );
//    proj.setInputCloud ( input );
//    pcl::ModelCoefficientsPtr coefficients( new pcl::ModelCoefficients );
//    coefficients->values.push_back( (float)(coeffs[0]) );
//    coefficients->values.push_back( (float)(coeffs[1]) );
//    coefficients->values.push_back( (float)(coeffs[2]) );
//    coefficients->values.push_back( (float)(coeffs[3]) );
//    proj.setModelCoefficients ( coefficients );
//    proj.filter ( *output );
//}

void PlaneSlam::cloudHull( const PointCloudTypePtr &cloud, PointCloudTypePtr &cloud_hull)
{
    pcl::ConcaveHull<PointType> chull;
    chull.setInputCloud ( cloud );
    chull.setAlpha ( plane_hull_alpha_ );
    chull.reconstruct ( *cloud_hull );
}

void PlaneSlam::tfToPose3( tf::Transform &trans, gtsam::Pose3 &pose )
{
    Eigen::Matrix3d m33 = matrixTF2Eigen( trans.getBasis() );
    gtsam::Rot3 rot3(m33);
    gtsam::Point3 point3;
    tf::Vector3 origin = trans.getOrigin();
    point3[0] = origin.getX();
    point3[1] = origin.getY();
    point3[2] = origin.getZ();
    pose = gtsam::Pose3( rot3, point3 );
}

void PlaneSlam::pose3ToTF( gtsam::Pose3 &pose, tf::Transform &trans )
{
    trans.setOrigin( tf::Vector3( pose.x(), pose.y(), pose.z()) );
    tf::Matrix3x3 m33;
    matrixEigen2TF( pose.rotation().matrix(), m33 );
    trans.setBasis( m33 );
}
