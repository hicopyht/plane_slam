#include "gtsam_mapping.h"

namespace plane_slam
{

GTMapping::GTMapping(ros::NodeHandle &nh, Viewer * viewer)
    : nh_(nh)
    , viewer_(viewer)
    , map_frame_("/world")
    , mapping_config_server_( ros::NodeHandle( nh_, "GTMapping" ) )
    , isam2_parameters_()
    , factor_graph_()
    , initial_estimate_()
    , pose_count_( 0 )
    , landmark_max_count_( 0 )
    , use_keyframe_( true )
    , keyframe_linear_threshold_( 0.05f )
    , keyframe_angular_threshold_( 5.0f )
    , isam2_relinearize_threshold_( 0.05f )
    , isam2_relinearize_skip_( 1 )
    , plane_match_direction_threshold_( 10.0*DEG_TO_RAD )   // 10 degree
    , plane_match_distance_threshold_( 0.1 )    // 0.1meter
    , plane_match_check_overlap_( true )
    , plane_match_overlap_alpha_( 0.5 )
    , plane_inlier_leaf_size_( 0.05f )  // 0.05meter
    , plane_hull_alpha_( 0.5 )
    , rng_(12345)
    , next_plane_id_( 0 )
    , next_frame_id_( 0 )
{
    // reconfigure
    mapping_config_callback_ = boost::bind(&GTMapping::gtMappingReconfigCallback, this, _1, _2);
    mapping_config_server_.setCallback(mapping_config_callback_);

    // Pose and Map
    optimized_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("optimized_pose", 10);
    optimized_path_publisher_ = nh_.advertise<nav_msgs::Path>("optimized_path", 10);
    map_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //
    optimize_graph_service_server_ = nh_.advertiseService("optimize_graph", &GTMapping::optimizeGraphCallback, this );
    save_graph_service_server_ = nh_.advertiseService("save_graph", &GTMapping::saveGraphCallback, this );
    save_map_service_server_ = nh_.advertiseService("save_map_pcd", &GTMapping::saveMapPCDCallback, this );
    remove_bad_inlier_service_server_ = nh_.advertiseService("remove_bad_inlier", &GTMapping::removeBadInlierCallback, this );

    // reset
    reset();
}


bool GTMapping::mapping( Frame *frame )
{
    bool success;
    if( !optimized_poses_list_.size() )    // first frame, add prior
    {
        success = addFirstFrame( frame );
    }
    else    // do mapping
    {
        if( use_keyframe_ )
        {
            if( isKeyFrame( frame ))
                success = doMapping( frame );   // do mapping
            else
                success = false;
        }
        else
        {
            success = doMapping( frame );   // do mapping
        }
    }

    // Map visualization
    if( success )
        updateMapViewer();

    cout << GREEN << " GTMapping, success = " << (success?"true":"false") << "." << RESET << endl;

    return success;
}

bool GTMapping::doMapping( Frame *frame )
{
    //
    if( !landmarks_list_.size() )
    {
        ROS_ERROR("You should call mapping() instead of doMapping().");
        exit(1);
    }

    // Set id to frame
    frame->setId( next_frame_id_ );
    next_frame_id_ ++;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 new_pose = tfToPose3( frame->pose_ );
    gtsam::Pose3 rel_pose = last_estimated_pose_.inverse() * new_pose;

    // Convert observations to OrientedPlane3
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        observations.push_back( OrientedPlane3(plane.coefficients) );

        // do voxel grid filter
        voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
    }

    // Get predicted-observation
    std::map<int, gtsam::OrientedPlane3> predicted_observations = getPredictedObservation( new_pose );

    // Match observations with predicted ones
    std::vector<PlanePair> pairs; // <lm, obs>
    matchObservationWithPredicted( predicted_observations, observations, planes, new_pose, pairs);

    // Print pairs info
    cout << GREEN << " find pairs(obs, lm): " << pairs.size() << RESET << endl;
    for( int i = 0; i < pairs.size(); i++)
    {
        const PlanePair &pair = pairs[i];
        cout << "  - " << pair.iobs << ", " << pair.ilm << endl;
    }

//    // check pairs
//    if( pairs.size() < 3 )
//        return new_pose;

    // Add odometry factors
    Vector odom_sigmas(6);
//    odom_sigmas << rel_pose.translation().vector()*0.1, rel_pose.rotation().rpy() * 0.1;
    odom_sigmas << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
//    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', frame->id() );
    Key last_key = Symbol('x', frame->id()-1);
    factor_graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    initial_estimate_.insert<Pose3>( pose_key, new_pose );

    // Add factor to exist landmark
    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        obs.setId( pair.ilm );
        Key ln =  Symbol( 'l', obs.id() );
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }

    // Add new landmark for unpaired observation
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            obs.setId( next_plane_id_ );
            next_plane_id_++;
            Key ln = Symbol('l', obs.id());
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            OrientedPlane3 glmn = lmn.transform( new_pose.inverse() );
            initial_estimate_.insert<OrientedPlane3>( ln, glmn );

            // Insert landmark coefficient to list
            optimized_landmarks_list_[obs.id()] = glmn;
            // Insert global plane to list
            PlaneType *lm( new PlaneType(true) );
            lm->coefficients = glmn.planeCoefficients();
            lm->color.Blue = rng_.uniform(0, 255);
            lm->color.Green = rng_.uniform(0, 255);
            lm->color.Red = rng_.uniform(0, 255);
            lm->color.Alpha = 255;
            landmarks_list_[obs.id()] = lm;
        }
    }

    // Insert optimized pose to list
    optimized_poses_list_[frame->id()] = new_pose;
    // Insert keyframe to list
    frames_list_[ frame->id() ] = frame;


    // Update graph
    isam2_->update(factor_graph_, initial_estimate_);
    isam2_->update(); // call additionally

    // Update optimized poses and planes
    updateOptimizedResult();

    // Update Map
    updateLandmarksInlier();

    // Clear the factor graph and values for the next iteration
    factor_graph_.resize(0);
    initial_estimate_.clear();

    // Refine map
    if( refine_planar_map_ )
    {
        refinePlanarMap();
    }

    // update estimated
    last_estimated_pose_ = optimized_poses_list_[ frame->id() ];
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );
    return true;

}

bool GTMapping::addFirstFrame( Frame *frame )
{
    // check number of planes
    if( frame->segment_planes_.size() == 0 )
        return false;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 init_pose = tfToPose3( frame->pose_ );

    // set ids
    frame->setId( next_frame_id_ );
    next_frame_id_++;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        plane.setId( next_plane_id_ );
        next_plane_id_ ++;

        // do voxel grid filter
        voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
    }


    // Add a prior factor
    Key x0 = Symbol('x', frame->id());
    Vector pose_sigmas(6);
//    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.2;
    pose_sigmas << 0.001, 0.001, 0.001, 0.0001, 0.001, 0.001;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//    noiseModel::Diagonal::shared_ptr poseNoise = //
//        noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    factor_graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );

    // Add a prior landmark
    Key l0 = Symbol('l', planes[0].id() );
    OrientedPlane3 lm0(planes[0].coefficients);
    OrientedPlane3 glm0 = lm0.transform(init_pose.inverse());
    noiseModel::Diagonal::shared_ptr lm_noise = noiseModel::Diagonal::Sigmas( (Vector(2) << planes[0].sigmas[0], planes[0].sigmas[1]).finished() );
    factor_graph_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );

    // Add new landmark
    landmark_max_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', plane.id() );

        cout << YELLOW << " lm" << i << " coefficents: " << plane.coefficients[0]
             << ", " << plane.coefficients[1] << ", " << plane.coefficients[2]
             << ", " << plane.coefficients[3] << ", centroid: " << plane.centroid.x
             << ", " << plane.centroid.y << ", " << plane.centroid.z << RESET << endl;

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
//        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        OrientedPlane3 glmn = lmn.transform( init_pose.inverse() );
        initial_estimate_.insert<OrientedPlane3>( ln,  glmn );

        // Insert landmarks coefficients to list
        optimized_landmarks_list_[plane.id()] = glmn;
        // Insert empty global plane to list
        PlaneType *lm( new PlaneType(true) );
        lm->coefficients = glmn.planeCoefficients();
        lm->color.Blue = rng_.uniform(0, 255);
        lm->color.Green = rng_.uniform(0, 255);
        lm->color.Red = rng_.uniform(0, 255);
        lm->color.Alpha = 255;
        landmarks_list_[plane.id()] = lm;
    }


    // Insert first pose to list
    optimized_poses_list_[ frame->id() ] = init_pose;
    // Insert first frame to list
    frames_list_[ frame->id() ] = frame;  // add frame to list

    // Optimize factor graph
//    isam2_->update( graph_, initial_estimate_ );
//    updateOptimizedResult();

//    // Clear the factor graph and values for the next iteration
//    graph_.resize(0);
//    initial_estimate_.clear();
//    //
    updateLandmarksInlier();

    last_estimated_pose_ = init_pose;
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );

    cout << GREEN << " Register first frame in the map." << endl;
//    initial_estimate_.print(" - Init pose: ");
    cout << " - Initial pose: " << endl;
    printTransform( init_pose.matrix() );
    cout << RESET << endl;

    return true;
}

bool GTMapping::isKeyFrame( Frame *frame )
{
    tf::Transform rel_tf = last_estimated_pose_tf_.inverse() * frame->pose_;
    double rad, distance;
    calAngleAndDistance( rel_tf, rad, distance );

    if( distance > keyframe_linear_threshold_ || rad > keyframe_angular_threshold_ )
        return true;
    else
        return false;
}

// get predicted landmarks
std::map<int, gtsam::OrientedPlane3> GTMapping::getPredictedObservation( const Pose3 &pose )
{
    std::map<int, gtsam::OrientedPlane3> predicted_observations;
    for( std::map<int, gtsam::OrientedPlane3>::iterator it = optimized_landmarks_list_.begin();
         it != optimized_landmarks_list_.end(); it++)
    {
        OrientedPlane3 &plane = it->second;
        predicted_observations[ it->first ] = plane.transform( pose );
    }
    return predicted_observations;
}

// simple euclidian distance
void GTMapping::matchObservationWithPredicted( std::map<int, gtsam::OrientedPlane3> &predicted_observations,
                                            const std::vector<OrientedPlane3> &observations,
                                            const std::vector<PlaneType> &observed_planes,
                                            const Pose3 pose,
                                            std::vector<PlanePair> &pairs)
{
//    Eigen::VectorXd paired = Eigen::VectorXd::Zero( predicted_observations.size() );
    for( int i = 0; i < observations.size(); i++)
    {
        const PlaneType &observed = observed_planes[i];
        const OrientedPlane3 &obs = observations[i];
        // plane coordinate
        Point3 point( observed.centroid.x, observed.centroid.y, observed.centroid.z );
        Point3 col3 = obs.normal().point3();
        Matrix32 basis = obs.normal().basis();
        Point3 col1( basis(0,0), basis(1,0), basis(2,0) );
        Point3 col2( basis(0,1), basis(1,1), basis(2,1) );
        Rot3 rot3( col1, col2, col3 );
        Pose3 local( rot3, point);
//        local.print("local coord: ");
        // Transform observation to local frame
        const OrientedPlane3 &lobs = obs.transform( local );
        int min_index = -1;
        int max_size = 0;

        for( std::map<int, gtsam::OrientedPlane3>::iterator it = predicted_observations.begin();
             it != predicted_observations.end(); it++)
        {
            const int id = it->first;
            OrientedPlane3 &plm = it->second;
            PlaneType *glm = landmarks_list_[id];

//            // check if alive
//            if( !glm.valid )
//                continue;

            // Transform landmark to local frame
            const OrientedPlane3 &llm = plm.transform( local );
            double cs = lobs.normal().dot( llm.normal() );
            double dr_error = acos( cs );
            double ds_error = fabs( lobs.distance() - llm.distance() );
//            cout << CYAN << "  - " << i << "*" << l << ": " << dr_error << "("<< plane_match_direction_threshold_ << "), "
//                 << ds_error << "(" << plane_match_distance_threshold_ << ")" << RESET << endl;

//            double dir_error = acos( obs.normal().dot( plm.normal() ));
//            double dis_error = fabs( obs.distance() - plm.distance() );
//            cout << YELLOW << "  - " << i << "*" << l << ": " << dir_error << "("<< plane_match_direction_threshold_ << "), "
//                 << dis_error << "(" << plane_match_distance_threshold_ << ")" << RESET << endl;
            if( (fabs(dr_error) < plane_match_direction_threshold_)
                    && (ds_error < plane_match_distance_threshold_) )
            {
                if( glm->cloud_voxel->size() > max_size )
                {
                    if( plane_match_check_overlap_ && !checkOverlap( glm->cloud_voxel, glm->coefficients, observed.cloud_voxel, pose ) )
                        continue;
    //                min_d = d;
                    min_index = id;
                    max_size = glm->cloud_voxel->size();

                }
            }
        }
        if( min_index >= 0 )
        {
//            paired[min_index] = 1;
            pairs.push_back( PlanePair(i, min_index) );
        }
    }
}

bool GTMapping::checkOverlap( const PointCloudTypePtr &landmark_cloud,
                            const OrientedPlane3 &landmark,
                            const PointCloudTypePtr &observation,
                            const Pose3 &pose)
{
    // transform and project pointcloud
    PointCloudTypePtr cloud( new PointCloudType ), cloud_projected( new PointCloudType );
    transformPointCloud( *observation, *cloud, pose.matrix() );
    projectPoints( *cloud, landmark.planeCoefficients(), *cloud_projected );

    // build octree
    float resolution = plane_inlier_leaf_size_;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( landmark_cloud );
    octreeD.addPointsFromInputCloud();

    // check if occupied
    int collision = 0;
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
            collision ++;
    }

    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

    double alpha = ((float)collision) / (float)cloud_projected->size();
    if( alpha < plane_match_overlap_alpha_ )
        return false;
    else
        return true;

    return true;
}

// indices of lm1 must bigger than that of lm2
bool GTMapping::checkLandmarksOverlap( const PlaneType &lm1, const PlaneType &lm2)
{
    // project lm2 inlier to lm1 plane
    PointCloudTypePtr cloud_projected( new PointCloudType );
    projectPoints( *lm2.cloud_voxel, lm1.coefficients, *cloud_projected );

    // build octree from lm1
    float resolution = plane_inlier_leaf_size_;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( lm1.cloud_voxel );
    octreeD.addPointsFromInputCloud();

    // check if occupied
    int collision = 0;
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
        {
            collision ++;
            if(collision > 20)
                return true;
        }
    }

//    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

//    double alpha = ((float)collision) / (float)cloud_projected->size();
//    if( alpha < plane_match_overlap_alpha_ )
//        return false;
//    else
//        return true;

    return false;
}

// just do project and voxel filter
void GTMapping::mergeLandmarkInlier( PlaneType &from, PlaneType &to)
{
    PointCloudTypePtr cloud( new PointCloudType );
    projectPoints( *from.cloud, to.coefficients, *cloud);
    *cloud += *to.cloud;
    voxelGridFilter( cloud, to.cloud, plane_inlier_leaf_size_ );
}

// return true if being refined.
bool GTMapping::refinePlanarMap()
{
//    cout << RED << ", lm size: " << landmarks_.size()
//         << ", pl size: " << estimated_planes_.size() << endl;

    // find co-planar landmark pair
    bool find_coplanar = false;
    const double direction_threshold = planar_merge_direction_threshold_;
    const double distance_threshold = planar_merge_distance_threshold_;
    std::map<int, int> remove_list;  // removed landmarks, <from, to>
    for( std::map<int, PlaneType*>::iterator it1 = landmarks_list_.begin();
         it1 != landmarks_list_.end(); it1++)
    {
        const int idx1 = it1->first;
        PlaneType &p1 = *(it1->second);
        OrientedPlane3 &lm1 = optimized_landmarks_list_[idx1];

        // check if will be removed
        if( !p1.valid )
            continue;

        // plane coordinate
        Point3 point( p1.centroid.x, p1.centroid.y, p1.centroid.z );
        Point3 col3 = lm1.normal().point3();
        Matrix32 basis = lm1.normal().basis();
        Point3 col1( basis(0,0), basis(1,0), basis(2,0) );
        Point3 col2( basis(0,1), basis(1,1), basis(2,1) );
        Rot3 rot3( col1, col2, col3 );
        Pose3 local( rot3, point);
        // Transform observation to local frame
        const OrientedPlane3 &llm1 = lm1.transform( local );

        for( std::map<int, PlaneType*>::iterator it2 = landmarks_list_.begin();
             it2 != landmarks_list_.end(); it2++)
        {
            const int idx2 = it2->first;
            PlaneType &p2 = *(it2->second);
            OrientedPlane3 &lm2 = optimized_landmarks_list_[idx2];

            // check if will be removed
            if( !p2.valid )
                continue;

            if( idx1 == idx2)
                continue;

            /// Match two plane
            // Transform landmark to local frame
            const OrientedPlane3 &llm2 = lm2.transform( local );
            double dr_error = acos( llm1.normal().dot( llm2.normal() ));
            double ds_error = fabs( llm1.distance() - llm2.distance() );
//            cout << CYAN << "  - " << idx1 << "*" << idx2 << ": " << dr_error << "("<< direction_threshold << "), "
//                 << ds_error << "(" << distance_threshold << ")" << RESET << endl;
            if( (fabs(dr_error) < direction_threshold)
                    && (ds_error < distance_threshold) )
            {
                // check if overlap
                bool overlap = false;
                if( p1.cloud_voxel->size() < p2.cloud_voxel->size() )
                    overlap = checkLandmarksOverlap( p2, p1);
                else
                    overlap = checkLandmarksOverlap( p1, p2);

                if( overlap )
                {
                    // merge, make smaller one invalid
                    if( p1.cloud_voxel->size() < p2.cloud_voxel->size() )
                    {
                        p1.valid = false;
                        remove_list[idx1] = idx2;
                        cout << RED << "  -- find co-planar: " << idx1 << " to " << idx2 << RESET << endl;
                        break;
                    }
                    else
                    {
                        p2.valid = false;
                        remove_list[idx2] = idx1;
                        cout << RED << "  -- find co-planar: " << idx2 << " to " << idx1 << RESET << endl;
                    }
                    find_coplanar = true;
                } // end of if overlap

            }

        } // end of for( int j = i+1; j < num; j++)

    } // end of for( int i = 0; i < (num - 1 ); i++)

    cout << YELLOW << " Find co-planar pairs: " << remove_list.size() << RESET << endl;

    std::map<int, std::set<int> > merge_list;   // merge list <to, set<from>>
    for( std::map<int,int>::iterator it = remove_list.begin(); it != remove_list.end(); it++)
    {
        int from = it->first;
        int to = it->second;

        // if to in tos
        bool to_in_tos = false;
        std::map<int, std::set<int> >::iterator ttitem = merge_list.find( to );
        if( ttitem != merge_list.end() )
        {
            ttitem->second.insert( from );
            to_in_tos = true;
        }

        // if from in tos
        bool from_in_tos = false;
        std::map<int, std::set<int> >::iterator ftitem = merge_list.find( from );
        if( ftitem != merge_list.end() )
        {
            std::set<int> fset = ftitem->second;
            fset.insert( from );
            merge_list.erase( ftitem );
            merge_list[to] = fset;
            from_in_tos = true;
        }

        // if to in froms
        bool to_is_removed = false;
        for( std::map<int, std::set<int> >::iterator rit = merge_list.begin();
             rit != merge_list.end(); rit++)
        {
            std::set<int>::iterator rset_item = rit->second.find( to );
            if( rset_item != rit->second.end() ) // already be removed
            {
                rit->second.insert( from );
                to_is_removed = true;
                break;
            }
        }

        if( to_in_tos || from_in_tos || to_is_removed )
            continue;


        // default, put into merge list
        // construct empty set, add
        std::set<int> from_set;
        from_set.insert( from );
        merge_list[to] = from_set;
    }

    // print merge list
    cout << YELLOW << " Merge list: " << endl;
    for( std::map<int, std::set<int> >::iterator it = merge_list.begin(); it != merge_list.end(); it++)
    {
        std::set<int> from_set = it->second;
        cout << "  - from:";
        for( std::set<int>::iterator its = from_set.begin(); its!= from_set.end(); its++)
            cout << " " << (*its);
        cout << " to: " << it->first << endl;
    }
    cout << RESET << endl;

    // reset for test
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        it->second->valid = true;
    }

    return find_coplanar;
}

bool GTMapping::removeBadInlier()
{
    float radius = 0.1;
    int min_neighbors = M_PI * radius *radius
            / (plane_inlier_leaf_size_ * plane_inlier_leaf_size_) *  planar_bad_inlier_alpha_;
    cout << GREEN << " Remove bad inlier, radius = " << radius
         << ", minimum neighbours = " << min_neighbors << RESET << endl;
    for( int i = 0; i < landmarks_.size(); i++)
    {
        PlaneType &lm = landmarks_[i];
        if( !lm.valid )
            continue;
        // build the filter
        pcl::RadiusOutlierRemoval<PointType> ror;
        ror.setInputCloud( lm.cloud_voxel );
        ror.setRadiusSearch( radius );
        ror.setMinNeighborsInRadius ( min_neighbors );
        // apply filter
        PointCloudTypePtr cloud_filtered( new PointCloudType );
        ror.filter( *cloud_filtered );
        // swap
        lm.cloud->swap( *cloud_filtered );
    }
}

void GTMapping::updateOptimizedResult()
{
    //
    Values values = isam2_->calculateBestEstimate();
    for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
            it != optimized_poses_list_.end(); it++)
    {
        Pose3 pose3 = values.at( Symbol('x', it->first) ).cast<Pose3>();
        it->second = pose3;
        frames_list_[it->first]->pose_ = pose3ToTF( pose3 );
    }
    for( std::map<int, gtsam::OrientedPlane3>::iterator it = optimized_landmarks_list_.begin();
            it != optimized_landmarks_list_.end(); it++)
    {
        OrientedPlane3 plane = values.at( Symbol('l', it->first) ).cast<OrientedPlane3>();
        it->second = plane;
        landmarks_list_[it->first]->coefficients = plane.planeCoefficients();
    }
}

void GTMapping::updateLandmarksInlier()
{
    // Update landmark poses, plane coefficients
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        lm->cloud_voxel->clear();  // clear inlier
    }
    // Sum
    for( std::map<int, Frame*>::iterator it = frames_list_.begin();
         it != frames_list_.end(); it++)
    {
        Frame *frame = it->second;
        Eigen::Matrix4d trans = transformTFToMatrix4d( frame->pose_ );
        for( int idx = 0; idx < frame->segment_planes_.size(); idx++)
        {
            PlaneType &obs = frame->segment_planes_[idx];
            PlaneType *lm = landmarks_list_[ obs.id() ];
            *(lm->cloud_voxel) += transformPointCloud( *(obs.cloud_voxel), trans );
        }
    }

    // Project and Downsample
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        PointCloudTypePtr cloud_projected( new PointCloudType );
        projectPoints( *(lm->cloud_voxel), lm->coefficients, *cloud_projected );
        voxelGridFilter( cloud_projected, lm->cloud_voxel, plane_inlier_leaf_size_ );
        setPointCloudColor( *(lm->cloud_voxel), lm->color );
        cloud_projected->clear();

        // compute 3d centroid
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *(lm->cloud_voxel), cen );
        lm->centroid.x = cen[0];
        lm->centroid.y = cen[1];
        lm->centroid.z = cen[2];
        lm->centroid.rgb = lm->color.float_value;
    }
}

void GTMapping::optimizeGraph( int n )
{
    while( n > 0 )
    {
        isam2_->update();
        n--;
    }
//    isam2_->print( "Map Graph");
}

void GTMapping::publishOptimizedPose()
{
    geometry_msgs::PoseStamped msg = pose3ToGeometryPose( last_estimated_pose_ );
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    optimized_pose_publisher_.publish( msg );
}

void GTMapping::publishOptimizedPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
            it != optimized_poses_list_.end(); it++)
    {
        path.poses.push_back( pose3ToGeometryPose( it->second ) );
    }

    optimized_path_publisher_.publish( path );
    cout << GREEN << " Publisher optimized path, p = " << path.poses.size() << RESET << endl;
}

void GTMapping::publishMapCloud()
{
    PointCloudTypePtr cloud ( new PointCloudType );

    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        *cloud += *(lm->cloud_voxel);
    }

//    for( int i = 0; i < landmarks_.size(); i++)
//    {
//        const PlaneType &lm = landmarks_[i];
//        if( !lm.valid )
//            continue;
//       *cloud += *lm.cloud;
//    }

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2);
    cloud2.header.frame_id = map_frame_;
    cloud2.header.stamp = ros::Time::now();
    cloud2.is_dense = false;

    map_cloud_publisher_.publish( cloud2 );

    cout << GREEN << " Publisher map as sensor_msgs/PointCloud2." << RESET << endl;
}

void GTMapping::updateMapViewer()
{
    // Pose, Path, MapCloud
    publishOptimizedPose();
    publishOptimizedPath();
    publishMapCloud();
    // Map in pcl visualization
    viewer_->removeMap();
    viewer_->displayMapLandmarks( landmarks_list_, "Map" );
    viewer_->spinMapOnce();
}

void GTMapping::reset()
{
    // Delete old
    if( isam2_ )
        delete isam2_;

    // Construct new
    isam2_parameters_.relinearizeThreshold = isam2_relinearize_threshold_; // 0.1
    isam2_parameters_.relinearizeSkip = isam2_relinearize_skip_; // 1
    isam2_parameters_.factorization = ISAM2Params::Factorization::QR;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    // clear
    pose_count_ = 0;
    landmark_max_count_ = 0;
    initial_estimate_.clear();
    estimated_poses_.clear();
    landmarks_.clear();
    estimated_planes_.clear();

    //
    next_plane_id_ = 0;
    next_frame_id_ = 0;
    for( std::map<int, Frame*>::iterator it = frames_list_.begin(); it != frames_list_.end(); it++)
    {
        delete (it->second);
    }
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin(); it!= landmarks_list_.end(); it++)
    {
        delete (it->second);
    }
    frames_list_.clear();
    landmarks_list_.clear();
    landmark_ids_.clear();
    key_frame_ids_.clear();
    landmarks_related_frames_list_.clear();
    optimized_poses_list_.clear();
    optimized_landmarks_list_.clear();

}

void GTMapping::saveMapPCD( const std::string &filename )
{
    PointCloudTypePtr cloud ( new PointCloudType );
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
//        if( !lm->valid )
//            continue;
        // Add color
        setPointCloudColor( *(lm->cloud_voxel), lm->color );
        *cloud += *(lm->cloud_voxel);
    }

    pcl::io::savePCDFileASCII ( filename, *cloud);
}

std::vector<geometry_msgs::PoseStamped> GTMapping::getOptimizedPath()
{
    std::vector<geometry_msgs::PoseStamped> poses;
    for( int i = 0; i < estimated_poses_.size(); i++)
    {
        poses.push_back( pose3ToGeometryPose( estimated_poses_[i] ) );
    }
    return poses;
}


void GTMapping::gtMappingReconfigCallback(plane_slam::GTMappingConfig &config, uint32_t level)
{
    //
    use_keyframe_ = config.use_keyframe;
    keyframe_linear_threshold_ = config.keyframe_linear_threshold;
    keyframe_angular_threshold_ = config.keyframe_angular_threshold * DEG_TO_RAD;
    //
    isam2_relinearize_threshold_ = config.isam2_relinearize_threshold;
    isam2_relinearize_skip_ = config.isam2_relinearize_skip;
    //
    plane_match_direction_threshold_ = config.plane_match_direction_threshold * DEG_TO_RAD;
    plane_match_distance_threshold_ = config.plane_match_distance_threshold;
    plane_match_check_overlap_ = config.plane_match_check_overlap;
    plane_match_overlap_alpha_ = config.plane_match_overlap_alpha;
    plane_inlier_leaf_size_ = config.plane_inlier_leaf_size;
    plane_hull_alpha_ = config.plane_hull_alpha;
    //
    refine_planar_map_ = config.refine_planar_map;
    planar_merge_direction_threshold_ = config.planar_merge_direction_threshold * DEG_TO_RAD;
    planar_merge_distance_threshold_ = config.planar_merge_distance_threshold;
    planar_bad_inlier_alpha_ = config.planar_bad_inlier_alpha;
    //
    publish_optimized_path_ = config.publish_optimized_path;

    cout << GREEN <<" GTSAM Mapping Config." << RESET << endl;
}

bool GTMapping::optimizeGraphCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( isam2_->empty() )
    {
        res.success = false;
        res.message = " Failed, graph is empty.";
    }
    else
    {
        optimizeGraph();    // graph optimizing
        updateMapViewer();  // update map visualization
        res.success = true;
        res.message = " Optimize graph for 10 times, and update map viewer.";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::saveGraphCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( isam2_->empty() )
    {
        res.success = false;
        res.message = " Failed, graph is empty.";
    }
    else
    {
        std::string filename = "/home/lizhi/bags/result/"+timeToStr()+"_graph.dot";
        saveGraphDot( filename );
        res.success = true;
        res.message = " Save isam2 graph as dot file: " + filename + ".";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::saveMapPCDCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( !landmarks_.size() )
    {
        res.success = false;
        res.message = " Failed, map is empty.";
    }
    else
    {
        std::string filename = "/home/lizhi/bags/result/" + timeToStr() + "_map.pcd";
        saveMapPCD( filename );
        res.success = true;
        res.message = " Save map as pcd file: " + filename + ".";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::removeBadInlierCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( !landmarks_.size() )
    {
        res.success = false;
        res.message = " Failed, map is empty.";
    }
    else
    {
        removeBadInlier();  // remove bad inlier in landmarks
        updateMapViewer();  // update map visualization
        //
        res.success = true;
        res.message = " Remove landmark bad inlier.";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

} // end of namespace plane_slam
