#include "tracking.h"

namespace plane_slam
{

Tracking::Tracking( ros::NodeHandle &nh, Viewer * viewer )
    : nh_(nh),
      viewer_(viewer),
      tracking_config_server_( ros::NodeHandle(nh_, "Tracking") )
{
    tracking_config_callback_ = boost::bind(&Tracking::trackingReconfigCallback, this, _1, _2);
    tracking_config_server_.setCallback(tracking_config_callback_);
}

bool Tracking::trackPlanes(const Frame &source, const Frame &target,
                     RESULT_OF_MOTION &motion, const Eigen::Matrix4d estimated_transform)
{
    ros::Time start_time = ros::Time::now();
    double pairs_dura, m_e_dura;

    // Set invalid at beginning
    motion.valid = false;
    motion.rmse = 1e9;
    motion.inlier = 0;

    // Find plane correspondences
    const std::vector<PlaneType> &planes = target.segment_planes_;
    const std::vector<PlaneType> &last_planes = source.segment_planes_;
    std::vector<PlanePair> pairs;
    if( planes.size() > 0 && last_planes.size() > 0 )
    {
        ITree::euclidianPlaneCorrespondences( planes, last_planes, pairs, estimated_transform);
        std::sort( pairs.begin(), pairs.end() );
    }
    const int pairs_num = pairs.size();
    cout << GREEN << " Plane pairs = " << pairs_num << RESET << endl;
    if( pairs_num < 3 )
        return false;

    //
    pairs_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // Estimate transform
    // Estimate motion using plane correspondences
    RESULT_OF_MOTION best_transform;
    std::vector<PlanePair> best_inlier;
    best_transform.rmse = 1e6; // no inlier
    best_transform.valid = false;   // no result
    best_transform.valid = solveRelativeTransformPlanes( source, target, pairs, best_transform, best_inlier );
    if( best_transform.valid && validRelativeTransform(best_transform) )
        motion = best_transform;
    //
    m_e_dura = (ros::Time::now() - start_time).toSec() * 1000;

    double total_time = pairs_dura + m_e_dura;
    cout << GREEN << " Tracking total time: " << total_time << endl;
    cout << " - Time:"
         << " pairing: " << pairs_dura
         << ", motion_estimate: " << m_e_dura
         << RESET << endl;

    return motion.valid;
}

bool Tracking::track(const Frame &source, Frame &target,
                     RESULT_OF_MOTION &motion, const Eigen::Matrix4d estimated_transform)
{
    ros::Time start_time = ros::Time::now();
    double pairs_dura, match_f_dura, m_e_dura, display_dura;

    // Find plane correspondences
    const std::vector<PlaneType> &planes = target.segment_planes_;
    const std::vector<PlaneType> &last_planes = source.segment_planes_;
    std::vector<PlanePair> pairs;
    if( planes.size() > 0 && last_planes.size() > 0 )
    {
        ITree::euclidianPlaneCorrespondences( planes, last_planes, pairs, estimated_transform);
        std::sort( pairs.begin(), pairs.end() );
    }
    const int pairs_num = pairs.size();
    cout << GREEN << " Plane pairs = " << pairs_num << RESET << endl;
    //
    pairs_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // matches keypoint features
    std::vector<cv::DMatch> good_matches;
    matchImageFeatures( source, target, good_matches,
                        feature_good_match_threshold_, feature_min_good_match_size_ );
    // Assign good matches to Frame
    target.good_matches_ = good_matches;
    //
    cout << GREEN << " Matches features, good_matches = " << good_matches.size() << RESET << endl;
    //
    match_f_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // Lookfor keypoint matches on the plane



    // Estimate transform
    std::vector<cv::DMatch> kp_inlier;
    std::vector<PlanePair> pl_inlier;
    bool valid = solveRelativeTransform( source, target, pairs, good_matches,
                                         motion, pl_inlier, kp_inlier );
    // Assign keypoint inlier to Frame
    target.kp_inlier_ = kp_inlier;

    m_e_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // Display
    viewer_->displayKeypointMatches( source.visual_image_, source.feature_locations_2d_,
                                     target.visual_image_, target.feature_locations_2d_,
                                     good_matches );

    display_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    double total_time = pairs_dura + match_f_dura + m_e_dura + display_dura;
    cout << GREEN << " Tracking total time: " << total_time << endl;
    cout << " - Time:"
         << " pairing: " << pairs_dura
         << ", kp_matching: " << match_f_dura
         << ", motion_estimate: " << m_e_dura
         << ", matches display: " << display_dura
         << RESET << endl;

    return valid;

}


bool Tracking::solveRelativeTransformPlanes( const Frame &source,
                                             const Frame &target,
                                             const std::vector<PlanePair> &pairs,
                                             RESULT_OF_MOTION &result,
                                             std::vector<PlanePair> &return_inlier )
{
    const std::vector<PlaneType> &planes = target.segment_planes_;
    const std::vector<PlaneType> &last_planes = source.segment_planes_;

    if( planes.size() < 3 || last_planes.size() < 3 || pairs.size() < 3)
        return false;

    return_inlier.clear();

    const unsigned int pairs_num = pairs.size();

    // Estimate transformation using all the plane correspondences
    RESULT_OF_MOTION best_transform;
    std::vector<PlanePair> best_inlier;
    best_transform.rmse = 1e9; // no inlier
    best_transform.valid = false;
    unsigned int real_iterations = 0;
    unsigned int valid_iterations = 0;
    for( int x1 = 0; x1 < pairs_num-2; x1++)
    {
        const PlanePair &p1 = pairs[x1];
        for( int x2 = x1+1; x2 < pairs_num-1; x2++ )
        {
            const PlanePair &p2 = pairs[x2];
            for( int x3 = x2+1; x3 < pairs_num; x3++)
            {
                real_iterations ++;
                const PlanePair &p3 = pairs[x3];
                std::vector<PlaneCoefficients> currents;
                std::vector<PlaneCoefficients> lasts;
                currents.push_back( planes[p1.iobs].coefficients );
                currents.push_back( planes[p2.iobs].coefficients );
                currents.push_back( planes[p3.iobs].coefficients );
                lasts.push_back( last_planes[p1.ilm].coefficients );
                lasts.push_back( last_planes[p2.ilm].coefficients );
                lasts.push_back( last_planes[p3.ilm].coefficients );
                //
//                cout << YELLOW << " solve motion: (" << x1 << "/" << x2 << "/" << x3 << ")" << RESET << endl;
                RESULT_OF_MOTION motion;
                motion.valid = solveRtPlanes( lasts, currents, motion);

                if( motion.valid )
                {
                    valid_iterations++;
                    // check if better
                    std::vector<PlanePair> inlier;
                    computePairInliersAndError( motion.transform4d(), pairs, last_planes, planes,
                                                inlier, motion.rmse, 5.0*DEG_TO_RAD, 0.05);
                    if( inlier.size() > best_inlier.size() )
                    {
                        best_transform = motion;
                        best_inlier = inlier;
                    }
                    else if( inlier.size() == best_inlier.size() && motion.rmse < best_transform.rmse )
                    {
                        best_transform = motion;
                        best_inlier = inlier;
                    }
                }
            }
        }
    }

    cout << GREEN << " Plane RANSAC iterations = " << real_iterations
         << ", valid iterations = " << valid_iterations << RESET << endl;

//    Eigen::umeyama
    result = best_transform;
    return_inlier = best_inlier;
    return best_transform.valid;
}


bool Tracking::solveRelativeTransformPointsRansac( const Frame &source,
                                                   const Frame &target,
                                                   const std::vector<cv::DMatch> &good_matches,
                                                   RESULT_OF_MOTION &result,
                                                   std::vector<cv::DMatch> &matches)
{
//    // match feature
//    std::vector<cv::DMatch> good_matches;
//    matchImageFeatures( source, frame, good_matches, feature_good_match_threshold_, feature_min_good_match_size_);
//
//    // sort
//    std::sort(good_matches.begin(), good_matches.end()); //sort by distance, which is the nn_ratio

    int min_inlier_threshold = ransac_min_inlier_;
    if( min_inlier_threshold > 0.6*good_matches.size() )
        min_inlier_threshold = 0.6*good_matches.size();

    matches.clear();

//    std::sort( good_matches.begin(), good_matches.end() );

    //
    Eigen::Matrix4f resulting_transformation;
    double rmse = 1e6;
    //
    matches.clear();
    const unsigned int sample_size = ransac_sample_size_;
    unsigned int valid_iterations = 0;
    const unsigned int max_iterations = ransac_iterations_;
    int real_iterations = 0;
    double inlier_error;
    double max_dist_m = ransac_inlier_max_mahal_distance_;
    bool valid_tf;
    for( int n = 0; n < max_iterations && good_matches.size() >= sample_size; n++)
    {
        double refined_error = 1e6;
        std::vector<cv::DMatch> refined_matches;
        std::vector<cv::DMatch> inlier = randomChooseMatchesPreferGood( sample_size, good_matches); //initialization with random samples
//        std::vector<cv::DMatch> inlier = randomChooseMatches( sample_size, good_matches); //initialization with random samples
        Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

        real_iterations++;
//        cout << "Iteration = " << real_iterations << endl;
        for( int refine = 0; refine < 20; refine ++)
        {
            Eigen::Matrix4f transformation = solveRtPcl( source.feature_locations_3d_,
                                                         target.feature_locations_3d_,
                                                         inlier, valid_tf );
            if( !valid_tf || transformation != transformation )
            {
//                cout << BLUE << "- valid = " << (valid_tf?"true":"false") << ", equal = " << (transformation == transformation) << RESET << endl;
                break;
            }

            computeCorrespondenceInliersAndError( good_matches, transformation, source.feature_locations_3d_, target.feature_locations_3d_,
                                                  min_inlier_threshold, inlier, inlier_error, max_dist_m );

            if( inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
                break;

//            cout << BOLDBLUE << " - refine = " << refine << ", relative transform: " << RESET << endl;
//            printTransform( transformation );
//            cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
//            cout << endl;

            if( inlier.size() > refined_matches.size() && inlier_error < refined_error )
            {
                unsigned int prev_num_inliers = refined_matches.size();
                assert( inlier_error>=0 );
                refined_transformation = transformation;
                refined_matches = inlier;
                refined_error = inlier_error;
                if( inlier.size() == prev_num_inliers )
                    break; //only error improved -> no change would happen next iteration
            }
            else
                break;
        }
        // Success
        if( refined_matches.size() > 0 )
        {
            valid_iterations++;

            //Acceptable && superior to previous iterations?
            if (refined_error <= rmse &&
                refined_matches.size() >= matches.size() &&
                refined_matches.size() >= min_inlier_threshold)
            {
                rmse = refined_error;
                resulting_transformation = refined_transformation;
                matches.assign(refined_matches.begin(), refined_matches.end());
                //Performance hacks:
                if ( refined_matches.size() > good_matches.size()*0.5 ) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
                if ( refined_matches.size() > good_matches.size()*0.75 ) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
                if ( refined_matches.size() > good_matches.size()*0.8 ) break; ///Can this get better anyhow?
            }
        }
    }

    if( valid_iterations == 0 ) // maybe no depth. Try identity?
    {
        cout << BOLDRED << "No valid iteration, try identity." << RESET << endl;
        //ransac iteration with identity
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();//hypothesis
        std::vector<cv::DMatch> inlier; //result
        double refined_error = 1e6;
        std::vector<cv::DMatch> refined_matches;
        Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

        //test which samples are inliers
        computeCorrespondenceInliersAndError( good_matches, Eigen::Matrix4f::Identity(), source.feature_locations_3d_, target.feature_locations_3d_,
                                            min_inlier_threshold, inlier, inlier_error, max_dist_m );

        cout << BOLDRED << "inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
        if( inlier.size() > sample_size && inlier_error < refined_error )
        {
            refined_matches = inlier;
            refined_error = inlier_error;
        }

        // refine
        for( int refine = 0; refine < 20; refine ++)
        {
            if( inlier.size() < sample_size )
                break;

            Eigen::Matrix4f transformation = solveRtPcl( source.feature_locations_3d_,
                                                         target.feature_locations_3d_,
                                                         inlier, valid_tf );
            if( !valid_tf || transformation != transformation )
            {
//                cout << BLUE << "- valid = " << (valid_tf?"true":"false") << ", equal = " << (transformation == transformation) << RESET << endl;
                break;
            }

            computeCorrespondenceInliersAndError( good_matches, transformation, source.feature_locations_3d_, target.feature_locations_3d_,
                                                  min_inlier_threshold, inlier, inlier_error, max_dist_m );

            if( inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
                break;

            cout << BOLDBLUE << " - refine = " << refine << ", relative transform: " << RESET << endl;
            printTransform( transformation );
            cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
            cout << endl;

            if( inlier.size() > refined_matches.size() && inlier_error < refined_error )
            {
                unsigned int prev_num_inliers = refined_matches.size();
                assert( inlier_error>=0 );
                refined_transformation = transformation;
                refined_matches = inlier;
                refined_error = inlier_error;
                if( inlier.size() == prev_num_inliers )
                    break; //only error improved -> no change would happen next iteration
            }
            else
                break;
        }

        // Success
        if( refined_matches.size() > 0 )
        {
            if (refined_error <= rmse &&
                refined_matches.size() >= matches.size() &&
                refined_matches.size() >= min_inlier_threshold)
            {
                rmse = refined_error;
                resulting_transformation = refined_transformation;
                matches.assign(refined_matches.begin(), refined_matches.end());
            }
        }
    }

    cout << BLUE << " Point RANSAC real iterations = " << real_iterations
         << ", valid iterations = " << valid_iterations << RESET << endl;

    result.setTransform4f( resulting_transformation );    // save result
    result.rmse = rmse;
    result.inlier = matches.size();
    result.valid = matches.size() >= min_inlier_threshold;
    return ( matches.size() >= min_inlier_threshold );
}


bool Tracking::solveRelativeTransformPlanesPointsRansac( const Frame &source,
                                                         const Frame &target,
                                                         const std::vector<PlanePair> &pairs,
                                                         const std::vector<cv::DMatch> &good_matches,
                                                         RESULT_OF_MOTION &result,
                                                         std::vector<cv::DMatch> &matches )
{
    if( !pairs.size() )
        return false;

    matches.clear();

    const int pairs_num = pairs.size();
    const std::vector<PlaneType> &planes = target.segment_planes_;
    const std::vector<PlaneType> &last_planes = source.segment_planes_;

    // using all plane correspondences
    std::vector< std::vector<PlanePair> > sample_plane_pairs;
    for( int i = 0; i < pairs_num-1; i++)
    {
        for( int j = i+1; j < pairs_num; j++)
        {
            // check co-planar for planes
            if( ITree::checkCoPlanar(pairs[i].iobs, pairs[j].iobs, 15.0) )
                continue;
            if( ITree::checkCoPlanar(pairs[i].ilm, pairs[j].ilm, 15.0) )
                continue;

            std::vector<PlanePair> ps;
            ps.push_back( pairs[i] );
            ps.push_back( pairs[j] );
            sample_plane_pairs.push_back( ps ); // 2 planes
        }
    }
    for( int i = 0; i < pairs_num; i++)
    {
        std::vector<PlanePair> ps;
        ps.push_back( pairs[i] );
        sample_plane_pairs.push_back( ps ); // 1 planes
    }

    // Will be the final result
    matches.clear();
    double resulting_error = 1e9;
    Eigen::Matrix4f resulting_transformation = Eigen::Matrix4f::Identity();
    //
    const unsigned int sample_size = 3;
    unsigned int valid_iterations = 0;
    int real_iterations = 0;
    double inlier_error;
    unsigned int min_inlier_threshold = good_matches.size() * 0.6;  // minimum point inlier
    double max_dist_m = ransac_inlier_max_mahal_distance_;
    bool valid_tf;
//    cout << BLUE << " min_inlier = " << min_inlier_threshold << ", iterations = " << ransac_iterations_ << RESET << endl;
    //
    unsigned int max_iterations = ransac_iterations_; // Size of sample plane/point size dependent or not?
    for( int n = 0; n < max_iterations; n++)
    {
        real_iterations ++;
        // Compute transformation
        std::vector<PlanePair> ps = randomChoosePlanePairsPreferGood( sample_plane_pairs );
        const unsigned int sample_points_size = sample_size - ps.size(); // number of sample points
        std::vector<cv::DMatch> inlier = randomChooseMatchesPreferGood( sample_points_size, good_matches ); //initialization with random samples
        Eigen::Matrix4f transformation = solveRtPlanesPoints( last_planes, planes, ps, source.feature_locations_3d_,
                                                              target.feature_locations_3d_, inlier, valid_tf );
//            cout << " - valid_tf = " << (valid_tf?"true":"false") << endl;
        if( !valid_tf || transformation != transformation )
            continue;
        if( valid_tf )  // valid transformation
        {
            computeCorrespondenceInliersAndError( good_matches, transformation, source.feature_locations_3d_, target.feature_locations_3d_,
                                                  min_inlier_threshold, inlier, inlier_error, max_dist_m );

            //
            if( inlier.size() > min_inlier_threshold && inlier_error < max_dist_m )
            {
                valid_iterations ++;

                //
                if( inlier.size() > matches.size() && inlier_error < resulting_error )
                {
                    matches = inlier;
                    resulting_error = inlier_error;
                    resulting_transformation = transformation;
                    //
//                    cout << BOLDBLUE << " - refine = " << n << ", relative transform: " << RESET << endl;
//                    printTransform( transformation );
//                    cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;

                    //
                    //Performance hacks:
                    if ( inlier.size() > good_matches.size()*0.5 )
                        n += max_iterations*0.1;
                    if ( inlier.size() > good_matches.size()*0.7 )
                        n += max_iterations*0.1;
                    if ( inlier.size() > good_matches.size()*0.8 )
                        n += max_iterations*0.1;
                    if ( inlier.size() > good_matches.size()*0.9 )
                        n += max_iterations*0.2;
                    if ( inlier.size() > good_matches.size()*0.95 )
                        n += max_iterations*0.3;
                }
            }
        }

    }

    //
    cout << BLUE << " Plane/Point RANSAC real iterations = " << real_iterations
         << ", valid iterations = " << valid_iterations << RESET << endl;

    // check if success
    if( matches.size() > min_inlier_threshold && resulting_error < max_dist_m)
    {
        result.setTransform4f( resulting_transformation );
        result.inlier = matches.size();
        result.rmse = resulting_error;
        result.valid = true;
        return true;
    }
    else
        return false;
}


bool Tracking::solveRelativeTransformIcp( const Frame &source,
                                          const Frame &target,
                                          RESULT_OF_MOTION &result)
{
    PointCloudXYZPtr cloud_icp( new PointCloudXYZ );
    result.valid = solveRtIcp( target.feature_cloud_, source.feature_cloud_, cloud_icp, result );
    return result.valid;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
bool Tracking::solveRelativeTransformPnP( const Frame& source,
                                          const Frame& target,
                                          const std::vector<cv::DMatch> &good_matches,
                                          const CameraParameters& camera,
                                          RESULT_OF_MOTION &result)
{
    int min_inlier_threshold = pnp_min_inlier_;
    if( min_inlier_threshold > 0.75*good_matches.size() )
        min_inlier_threshold = 0.75*good_matches.size();

    // 3d points
    vector<cv::Point3f> pts_obj;
    // 2d points
    vector< cv::Point2f > pts_img;

    //
    for (size_t i=0; i<good_matches.size(); i++)
    {
        int query_index = good_matches[i].queryIdx;
        int train_index = good_matches[i].trainIdx;

        // train
        // 3d point
        Eigen::Vector4f pc = target.feature_locations_3d_[train_index];
        cv::Point3f pd( pc[0], pc[1], pc[2] );
        pts_obj.push_back( pd );

        // query
        // 2d point
        pts_img.push_back( cv::Point2f( source.feature_locations_2d_[query_index].pt ) );
    }

    // Construct camera intrinsic matrix
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inlier;
    // Solve pnp
//    cout << "solving pnp" << endl;
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false,
                        pnp_iterations_, pnp_repreject_error_, min_inlier_threshold, inlier );

    //
    result.inlier = inlier.rows;
    result.translation = Eigen::Vector3d( tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2) );
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    cvToEigen( R, result.rotation );

//    cout << " Trans: " << endl << result.translation << endl;
//    cout << " Rot: " << endl << result.rotation << endl;

    return ( result.inlier >= min_inlier_threshold );
}


bool Tracking::solveRelativeTransform( const Frame &source,
                                       const Frame &target,
                                       const std::vector<PlanePair> &pairs,
                                       const std::vector<cv::DMatch> &good_matches,
                                       RESULT_OF_MOTION &result,
                                       std::vector<PlanePair> &pl_inlier ,
                                       std::vector<cv::DMatch> &kp_inlier)
{
    result.valid = false;
    result.rmse = 1e9;
    result.inlier = 0;

    ros::Time start_time = ros::Time::now();
    double planes_dura, points_planes_dura,
            points_dura, icp_dura, pnp_dura;

    /// case 1: Estimate motion using plane correspondences
    RESULT_OF_MOTION best_transform;
    std::vector<PlanePair> best_inlier;
    best_transform.rmse = 1e6; // no inlier
    best_transform.valid = false;   // no result
    if( pairs.size() >= 3 )
    {
        best_transform.valid = solveRelativeTransformPlanes( source, target, pairs, best_transform, best_inlier );
        if( best_transform.valid && validRelativeTransform(best_transform) )
        {
            result = best_transform;
            pl_inlier = best_inlier;
            return true;
        }
    }

    //
    planes_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

//    // print info
    cout << GREEN << " Transformation from plane correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
//    printTransform( best_transform.transform4d() );

//    best_transform.valid = false;   // for test
    /// case 2: Estimate motion using plane and point correspondences
    if( !best_transform.valid && pairs.size() > 0 && good_matches.size() >= 3)
    {
        best_transform.valid = solveRelativeTransformPlanesPointsRansac( source, target, pairs, good_matches, best_transform, kp_inlier );
    }
    if( best_transform.valid && validRelativeTransform(best_transform) )
    {
        result = best_transform;
        return true;
    }
    //
    points_planes_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

//    // print info
    cout << GREEN << " Transformation from plane/point correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << best_transform.rmse << ", inlier = " << best_transform.inlier << RESET << endl;
//    printTransform( best_transform.transform4d() );

//    best_transform.valid = false; // for test
    /// case 3: Estimate motion using point correspondences
    if( !best_transform.valid && good_matches.size() >= 3 )
    {
        kp_inlier.clear();
        best_transform.valid = solveRelativeTransformPointsRansac( source, target, good_matches, best_transform, kp_inlier );
    }

    if( best_transform.valid && validRelativeTransform(best_transform) )
    {
        result = best_transform;
        return true;
    }

    points_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

//    // print info
    cout << GREEN << " Transformation from point correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << best_transform.rmse << ", inlier = " << best_transform.inlier << RESET << endl;
//    printTransform( best_transform.transform4d() );

//    best_transform.valid = false; // for test
    /// case 4: Using ICP
    if( !best_transform.valid && good_matches.size() >= 20 )
    {
        best_transform.valid = solveRelativeTransformIcp( source, target, best_transform );
    }

    if( best_transform.valid && validRelativeTransform(best_transform) )
    {
        result = best_transform;
        return true;
    }
    //
    icp_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

//    // print info
    cout << GREEN << " Transformation from ICP: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
//    printTransform( best_transform.transform4d() );

//    best_transform.valid = false; // for test
    /// case 5: using PnP
    if( !best_transform.valid && good_matches.size() >= 20 )
    {
        best_transform.valid = solveRelativeTransformPnP( source, target, good_matches, target.camera_params_, best_transform );
    }

    if( best_transform.valid && validRelativeTransform(best_transform) )
    {
        result = best_transform;
        return true;
    }
    //
    pnp_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

//    // print info
    cout << GREEN << " Transformation from PnP: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
//    printTransform( best_transform.transform4d() );

    /// Print info
    double total_time = planes_dura + points_planes_dura + points_dura + icp_dura + pnp_dura;
    cout << GREEN << " Transformation total time: " << total_time << endl;
    cout << " -Time:"
         << " planes: " << planes_dura
         << ", planes/points: " << points_planes_dura
         << ", points: " << points_dura
         << ", icp: " << icp_dura
         << ", pnp: " << pnp_dura
         << RESET << endl;

    return false;
}

bool Tracking::solveRtIcp( const PointCloudXYZPtr &source,
                           const PointCloudXYZPtr &target,
                           PointCloudXYZPtr &cloud_icp,
                           RESULT_OF_MOTION &result )
{
    Eigen::Matrix4d Tm = Eigen::Matrix4d::Identity();
    const double icp_score_threshold = icp_score_threshold_;
    double score = 1.0;
    bool is_converged = false;

    // icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    icp->setMaxCorrespondenceDistance(icp_max_distance_);
    icp->setMaximumIterations (icp_iterations_);
    icp->setTransformationEpsilon( icp_tf_epsilon_ );
    icp->setInputSource( source );
    icp->setInputTarget( target );
    icp->align ( *cloud_icp );
    score = icp->getFitnessScore();
    is_converged = icp->hasConverged();

    // save result
    Tm = icp->getFinalTransformation().cast<double>();
    result.setTransform4d( Tm );
    result.inlier = cloud_icp->size();
    result.score = score;

    if(is_converged && score <= icp_score_threshold )
    {
        result.valid = true;
        return true;
    }
    else
    {
        result.valid = false;
        return false;
    }
}

Eigen::Matrix4f Tracking::solveRtPcl( const std_vector_of_eigen_vector4f &query_points,
                                      const std_vector_of_eigen_vector4f &train_points,
                                      const std::vector<cv::DMatch> &matches,
                                      bool &valid)
{
    pcl::TransformationFromCorrespondences tfc;
    valid = true;
    float weight = 1.0;

    BOOST_FOREACH(const cv::DMatch& m, matches)
    {
//        Eigen::Vector3f from = query_points[m.queryIdx].head<3>();
//        Eigen::Vector3f to = train_points[m.trainIdx].head<3>();
        Eigen::Vector3f to = query_points[m.queryIdx].head<3>();
        Eigen::Vector3f from = train_points[m.trainIdx].head<3>();
        if( std::isnan(from(2)) || std::isnan(to(2)) )
            continue;
        weight = 1.0/(from(2) * to(2));
        tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
    }

    // get relative movement from samples
    if( tfc.getNoOfSamples() < 3)
    {
        valid = false;
        return Eigen::Matrix4f();
    }
    else
        return tfc.getTransformation().matrix();
}

void Tracking::solveRt( const std::vector<PlaneCoefficients> &before,
                              const std::vector<PlaneCoefficients> &after,
                              const std::vector<Eigen::Vector3d>& from_points,
                              const std::vector<Eigen::Vector3d>& to_points,
                              RESULT_OF_MOTION &result)
{
    const int num_points = from_points.size();
    const int num_planes = before.size();

//    cout << " solveRT, planes = " << num_planes << ", points = " << num_points << RESET << endl;

    ROS_ASSERT( before.size() == after.size() );
    ROS_ASSERT( from_points.size() == to_points.size() );
    ROS_ASSERT( num_planes + num_points == 3 );

    // Rotation
    Eigen::MatrixXd froms(3, num_points), tos(3, num_points);
    Eigen::MatrixXd src(3, num_planes), dst(3, num_planes);
    for( int i = 0; i < num_points; i++)
    {
        tos.col(i) = from_points[i];
        froms.col(i) = to_points[i];
    }
    for( int i = 0; i < num_planes; i++)
    {
        src.col(i) = after[i].head<3>();
        dst.col(i) = before[i].head<3>();
    }
    const double wi = 1.0;// / num_planes;
    const double one_over_n = num_points > 0 ? 1.0 / num_points : 0;
    /// 1: For point
    // mean
    const Eigen::VectorXd froms_mean = froms.rowwise().sum() * one_over_n;
    const Eigen::VectorXd tos_mean = tos.rowwise().sum() * one_over_n;
    // demeaning
    const Eigen::MatrixXd froms_demean = froms.colwise() - froms_mean;
    const Eigen::MatrixXd tos_demean = tos.colwise() - tos_mean;
    // Eq. (38)
    const Eigen::MatrixXd sigma_points = one_over_n * tos_demean * froms_demean.transpose();

    /// 2: For plane
    const Eigen::MatrixXd sigma_planes = wi * dst * src.transpose();

    /// 3: sigma
    const Eigen::MatrixXd sigma = sigma_points + sigma_planes;

    /// 4: Umeyama solution
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3,3);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);
    // Eq. (39)
    Eigen::VectorXd S = Eigen::VectorXd::Ones( 3 );
//    cout << "   det(sigma) = " << sigma.determinant() << endl;
    if( sigma.determinant() < 0 )
        S( 2 ) = -1;
    // Eq. (40) and (43)
    const Eigen::VectorXd& vs = svd.singularValues();
    int rank = 0;
    for (int i=0; i<3; ++i)
        if (!Eigen::internal::isMuchSmallerThan(vs.coeff(i),vs.coeff(0)))
            ++rank;
//    cout << "   D: " << endl;
//    cout << vs << endl;
//    cout << "   rank(sigma) = " << rank << endl;
    if ( rank == 2 )
    {
//        cout << "   det(U)*det(V) = " << svd.matrixU().determinant() * svd.matrixV().determinant() << endl;
        if ( svd.matrixU().determinant() * svd.matrixV().determinant() > 0 )
        {
            R = svd.matrixU()*svd.matrixV().transpose();
        }
        else
        {
            const double s = S(2);
            S(2) = -1;
            R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
            S(2) = s;
        }
    }
    else
    {
        R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    }
    // Translation
    Eigen::VectorXd T = Eigen::Vector3d::Zero(3);
    /// 1: For points
    Eigen::MatrixXd I3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd A1 = num_points * I3;
    Eigen::VectorXd b1 = num_points * ( tos_mean - R * froms_mean );
    /// 2: For planes
    Eigen::MatrixXd A2( num_planes, 3 );
    Eigen::VectorXd b2( num_planes );
    for( int i = 0; i < num_planes; i++)
    {
        A2.row(i) = wi * before[i].head<3>().transpose();
        b2(i) = after[i](3) - before[i](3);
    }
    /// 3:  ( A1 A2 )^T * t = ( b1 b2)^T
    Eigen::MatrixXd AA;( 3 + num_planes, 3 );
    Eigen::VectorXd bb;( 3 + num_planes );
    if( num_points != 0 )
    {
        AA = Eigen::MatrixXd( 3 + num_planes, 3);
        bb = Eigen::VectorXd( 3 + num_planes );
        AA << A1, A2;
        bb << b1, b2;
    }
    else
    {
        AA = Eigen::MatrixXd( num_planes, 3);
        bb = Eigen::VectorXd( num_planes );
        AA << A2;
        bb << b2;
    }


    /// 4: t = (A.transpose()*A).inverse()*A.transpose()*b;
    Eigen::MatrixXd AAT = AA.transpose();
    T = (AAT*AA).inverse()*AAT*bb;

    result.rotation = R;
    result.translation = T;
    result.valid = true;
}

void Tracking::solveRt( const std::vector<Eigen::Vector3d>& from_points,
                              const std::vector<Eigen::Vector3d>& to_points,
                              RESULT_OF_MOTION &result)
{
    ROS_ASSERT( from_points.size() >= 3 && to_points.size() >=3 );

    Eigen::MatrixXd src(3,3), dst(3,3);
    for( int i = 0; i < 3; i++)
    {
        src.col(i) = from_points[i];
        dst.col(i) = to_points[i];
    }
    Eigen::Matrix4d transform = Eigen::umeyama(src, dst, false);
    result.rotation = transform.topLeftCorner(3,3);
    result.translation = transform.col(3).head<3>();
}

void Tracking::solveRt( const std::vector<PlaneCoefficients> &before,
                              const std::vector<PlaneCoefficients> &after,
                              RESULT_OF_MOTION &result)
{
    ROS_ASSERT( after.size() >= 3 && before.size() >=3 );

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3,3);
    Eigen::VectorXd T = Eigen::Vector3d::Zero(3);
    // algorithm: "Least-squares estimation of transformation parameters between two point patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
    Eigen::MatrixXd A(3, 3), B(3, 3); // A = RB, A === dst, B === src
    Eigen::VectorXd distance( 3 );
    for(int i = 0; i < 3; i++ )
    {
        const Eigen::Vector3d to = before[i].head<3>();
        const Eigen::Vector3d from = after[i].head<3>();
        const double d2 = before[i](3);
        const double d1 = after[i](3);
        B.col(i) = from;
        A.col(i) = to;
        distance(i) = d1 - d2;   // d_src - d_dst
    }
    // Rotation
    // n_dst = R * n_src
    const Eigen::MatrixXd sigma = A * B.transpose();    // Eq. ABt
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);

    // Eq. (39)
    Eigen::VectorXd S = Eigen::VectorXd::Ones( 3 );
//    cout << "   det(sigma) = " << sigma.determinant() << endl;
    if( sigma.determinant() < 0 )
        S( 2 ) = -1;

    // Eq. (40) and (43)
    const Eigen::VectorXd& vs = svd.singularValues();
    int rank = 0;
    for (int i=0; i<3; ++i)
        if (!Eigen::internal::isMuchSmallerThan(vs.coeff(i),vs.coeff(0)))
            ++rank;
//    cout << "   D: " << endl;
//    cout << vs << endl;
//    cout << "   rank(sigma) = " << rank << endl;
    if ( rank == 2 )
    {
//        cout << "   det(U)*det(V) = " << svd.matrixU().determinant() * svd.matrixV().determinant() << endl;
        if ( svd.matrixU().determinant() * svd.matrixV().determinant() > 0 )
        {
            R = svd.matrixU()*svd.matrixV().transpose();
        }
        else
        {
            const double s = S(2);
            S(2) = -1;
            R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
            S(2) = s;
        }
    }
    else
    {
        R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    }
    // Translation
    // n_dst^T * t = d_dst - d_src
    Eigen::JacobiSVD<MatrixXd> svdA(A.transpose(), ComputeFullU | ComputeFullV);
    T = svdA.solve(distance);

    result.rotation = R;
    result.translation = T;
}

bool Tracking::solveRtPlanes( const std::vector<PlaneCoefficients> &last_planes,
                              const std::vector<PlaneCoefficients> &planes,
                              RESULT_OF_MOTION &result)
{
    result.rotation = Eigen::Matrix3d::Identity();
    result.translation = Eigen::Vector3d::Zero();

    /// 1: Check number of planes
    if( planes.size() != 3 || last_planes.size() != 3)
        return false;

    /// 2: Check co-planar
    double dis1, dis2, dis3;
    double dir1, dir2, dir3;
    const double dir_thresh = 15.0 * DEG_TO_RAD;
    // check co-planar
    ITree::euclidianDistance( planes[0], planes[1], dir1, dis1 );
    ITree::euclidianDistance( planes[0], planes[2], dir2, dis2 );
    ITree::euclidianDistance( planes[1], planes[2], dir3, dis3 );
//    cout << BOLDBLUE << " dir " << dir_thresh << " : " << dir1 << " " << dir2 << " " << dir3 << RESET << endl;
    if( dir1 < dir_thresh || dir2 < dir_thresh || dir3 < dir_thresh )
        return false;
    // check co-planar
    ITree::euclidianDistance( last_planes[0], last_planes[1], dir1, dis1 );
    ITree::euclidianDistance( last_planes[0], last_planes[2], dir2, dis2 );
    ITree::euclidianDistance( last_planes[1], last_planes[2], dir3, dis3 );
//    cout << BOLDBLUE << " dir " << dir_thresh << " : " << dir1 << " " << dir2 << " " << dir3 << RESET << endl;
    if( dir1 < dir_thresh || dir2 < dir_thresh || dir3 < dir_thresh )
        return false;

    /// 3: compute Rt
    solveRt( last_planes, planes, result );

    return true;
}


Eigen::Matrix4f Tracking::solveRtPlanesPoints( const std::vector<PlaneType> &last_planes,
                                               const std::vector<PlaneType> &planes,
                                               const std::vector<PlanePair> &pairs,
                                               const std_vector_of_eigen_vector4f &last_feature_3d,
                                               const std_vector_of_eigen_vector4f &feature_3d,
                                               const std::vector<cv::DMatch> &matches,
                                               bool &valid )
{
    if( !pairs.size() || ! matches.size() || (pairs.size() + matches.size()) != 3)
    {
        valid = false;
        ROS_ERROR("Invalid number of pairs and matches to solve RT, pairs = %d, matches = %d ",
                  pairs.size(), matches.size() );
        return Eigen::Matrix4f::Identity();
    }

    std::vector<PlaneCoefficients> before;
    std::vector<PlaneCoefficients> after;
    std::vector<Eigen::Vector3d> from_points;
    std::vector<Eigen::Vector3d> to_points;

    // solve RT
    for( int i = 0; i < pairs.size(); i++)
    {
        before.push_back( last_planes[pairs[i].ilm].coefficients );
        after.push_back( planes[pairs[i].iobs].coefficients );
    }
    for( int i = 0; i < matches.size(); i++)
    {
        from_points.push_back( last_feature_3d[matches[i].queryIdx].head<3>().cast<double>() );
        to_points.push_back( feature_3d[matches[i].trainIdx].head<3>().cast<double>() );
    }

    // check geometric constrains
    // 2 planes and 1 point
    valid = false;
    const double dir_threshold = 8.0 * DEG_TO_RAD;
    const double dis_threshold = 0.1;
    if( pairs.size() == 2)
    {
        double dis1, dis2;
        double dir1, dir2;
        // angle of 2 planes
        ITree::euclidianDistance( last_planes[pairs[0].ilm], last_planes[pairs[1].ilm], dir1, dis1 );
        ITree::euclidianDistance( planes[pairs[0].iobs], planes[pairs[1].iobs], dir2, dis2 );
        if( fabs( dir1 - dir2 ) > dir_threshold )
            return Eigen::Matrix4f::Identity();

        // distance of point to plane1
        dis1 = ITree::distancePointToPlane( last_feature_3d[matches[0].queryIdx], last_planes[pairs[0].ilm].coefficients );
        dis2 = ITree::distancePointToPlane( feature_3d[matches[0].trainIdx], planes[pairs[0].iobs].coefficients );
        if( fabs(dis1 - dis2) > dis_threshold )
            return Eigen::Matrix4f::Identity();

        // distance of point to plane2
        dis1 = ITree::distancePointToPlane( last_feature_3d[matches[0].queryIdx], last_planes[pairs[1].ilm].coefficients );
        dis2 = ITree::distancePointToPlane( feature_3d[matches[0].trainIdx], planes[pairs[1].iobs].coefficients );
        if( fabs(dis1 - dis2) > dis_threshold )
            return Eigen::Matrix4f::Identity();
    }
    // 1 plane and 2 points
    else if( pairs.size() == 1)
    {
        double dis1, dis2;
        // distance of 2 points
        dis1 = (before[0] - before[1]).norm();
        dis2 = (after[0] - after[1]).norm();
        if(  dis1 < 0.2 || dis2 < 0.2
                || fabs(dis1 - dis2) > dis_threshold )
            return Eigen::Matrix4f::Identity();

        // distance of point1 to plane
        dis1 = ITree::distancePointToPlane( last_feature_3d[matches[0].queryIdx], last_planes[pairs[0].ilm].coefficients );
        dis2 = ITree::distancePointToPlane( feature_3d[matches[0].trainIdx], planes[pairs[0].iobs].coefficients );
        if( fabs(dis1 - dis2) > dis_threshold )
            return Eigen::Matrix4f::Identity();

        // distance of point2 to plane
        dis1 = ITree::distancePointToPlane( last_feature_3d[matches[1].queryIdx], last_planes[pairs[0].ilm].coefficients );
        dis2 = ITree::distancePointToPlane( feature_3d[matches[1].trainIdx], planes[pairs[0].iobs].coefficients );
        if( fabs(dis1 - dis2) > dis_threshold )
            return Eigen::Matrix4f::Identity();
    }
    else
    {
        return Eigen::Matrix4f::Identity();
    }


    RESULT_OF_MOTION motion;
    solveRt( before, after, from_points, to_points, motion );
    valid = true;

    return motion.transform4f();
}



void Tracking::matchImageFeatures( const Frame& source,
                                   const Frame& target,
                                   vector< cv::DMatch > &good_matches,
                                   double good_match_threshold,
                                   int min_match_size)
{
    vector< cv::DMatch > matches;

    uint64_t* query_value =  reinterpret_cast<uint64_t*>(source.feature_descriptors_.data);
    uint64_t* search_array = reinterpret_cast<uint64_t*>(target.feature_descriptors_.data);
    for(unsigned int i = 0; i < source.feature_locations_2d_.size(); ++i, query_value += 4)
    {   //ORB feature = 32*8bit = 4*64bit
        int result_index = -1;
        int hd = bruteForceSearchORB(query_value, search_array, target.feature_locations_2d_.size(), result_index);
        if(hd >= 128)
            continue;//not more than half of the bits matching: Random
        cv::DMatch match(i, result_index, hd /256.0 + (float)rand()/(1000.0*RAND_MAX));
        matches.push_back(match);
    }

//    cout << GREEN << "Kp matches = " << matches.size() << RESET << endl;

    // Sort
    std::sort( matches.begin(), matches.end() );

    // Get good matches, fixed size
    if( min_match_size != 0)
    {
        int add = 0;
        BOOST_FOREACH(const cv::DMatch& m, matches)
        {
            if( add >= min_match_size )
                break;

            if( source.feature_locations_3d_[m.queryIdx](2) != 0
                    && target.feature_locations_3d_[m.trainIdx](2) != 0)
            {
                good_matches.push_back( m );
                add ++;
            }
        }
    }
    else
    {
        const double minDis = matches[0].distance;

        BOOST_FOREACH(const cv::DMatch& m, matches)
        {
            if( m.distance >= good_match_threshold * minDis )
                break;

            if( source.feature_locations_3d_[m.queryIdx](2) != 0
                    && target.feature_locations_3d_[m.trainIdx](2) != 0)
            {
                good_matches.push_back( m );
            }
        }

    }

//    cout << "good matches: " << good_matches.size() << endl;
}

// from: https://github.com/felixendres/rgbdslam_v2/src/node.cpp
void Tracking::computeCorrespondenceInliersAndError( const std::vector<cv::DMatch> & matches,
                                  const Eigen::Matrix4f& transform4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& query_points,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& train_points,
                                  unsigned int min_inlier,
                                  std::vector<cv::DMatch>& inlier, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  double squared_max_distance) const
{
    inlier.clear();
    assert(matches.size() > 0);
    inlier.reserve(matches.size());
    //errors.clear();
    const size_t all_matches_size = matches.size();
    double mean_error = 0.0;
    Eigen::Matrix4d transform4d = transform4f.cast<double>();

    //parallelization is detrimental here
    //#pragma omp parallel for reduction (+: mean_error)
    for(int i=0; i < all_matches_size; ++i)
    //BOOST_FOREACH(const cv::DMatch& m, all_matches)
    {
        const cv::DMatch& m = matches[i];
//        const Eigen::Vector4f& from = froms[m.queryIdx];
//        const Eigen::Vector4f& to = tos[m.trainIdx];
        const Eigen::Vector4f& to = query_points[m.queryIdx];
        const Eigen::Vector4f& from = train_points[m.trainIdx];

        if( std::isnan(from(2)) || std::isnan(to(2)) )
        { //does NOT trigger on NaN
            continue;
        }
        double mahal_dist = errorFunction2(from, to, transform4d);
//        double mahal_dist = errorFunction2(to, from, transform4d);
//        cout << " (" << from[0] << "," << from[1] << "," << from[2]
//             << ")->(" << to[0] << "," << to[1] << "," << to[2] << ") = "<< mahal_dist;
        if(mahal_dist > squared_max_distance)
        {
            continue; //ignore outliers
        }
        if(!(mahal_dist >= 0.0))
        {
            ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
            ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transform4d << "Matches: " << i);
            continue;
        }
        mean_error += mahal_dist;
        //#pragma omp critical
        inlier.push_back(m); //include inlier
    }


    if ( inlier.size()<3 )
    {
        //at least the samples should be inliers
        ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches_size); // only warn if this checks for all initial matches
        return_mean_error = 1e9;
    }
    else
    {
        mean_error /= inlier.size();
        return_mean_error = sqrt(mean_error);
    }
}

void Tracking::computePairInliersAndError( const Eigen::Matrix4d &transform,
                                           const std::vector<PlanePair>& pairs,
                                           const std::vector<PlaneType>& last_planes,
                                           const std::vector<PlaneType>& planes,
                                           std::vector<PlanePair> &inlier,
                                           double &return_mean_error,
                                           const double max_direction_error,
                                           const double max_distance_error )
{
    inlier.clear();

    double direction_squared_sum = 0;
    double distance_squared_sum = 0;
    for( int i = 0; i < pairs.size(); i++)
    {
        const PlaneType &plane = planes[ pairs[i].iobs ];
        const PlaneType &last_plane = last_planes[ pairs[i].ilm ];
        PlaneType transformed_plane;
        transformPlane( last_plane.coefficients, transform, transformed_plane.coefficients );
//        cout << GREEN << " - tr p: " << pairs[i].iobs << "/" << pairs[i].ilm << ": " << endl;
//        cout << " current: " << plane.coefficients[0] << ", " << plane.coefficients[1]
//             << ", " << plane.coefficients[2] << ", " << plane.coefficients[3] << endl;
//        cout << " transformed: " << transformed_plane.coefficients[0] << ", " << transformed_plane.coefficients[1]
//             << ", " << transformed_plane.coefficients[2] << ", " << transformed_plane.coefficients[3]
//             << RESET << endl;
        double direction, distance;
        ITree::euclidianDistance( plane, transformed_plane, direction, distance );

        // check inlier
        if( (direction < max_direction_error) && (distance < max_distance_error) )
        {
            inlier.push_back( pairs[i] );
            direction_squared_sum += direction * direction;
            distance_squared_sum += distance * distance;
        }
    }

    return_mean_error = sqrt( direction_squared_sum / inlier.size() )
            + sqrt( distance_squared_sum / inlier.size() );
}

std::vector<PlanePair> Tracking::randomChoosePlanePairsPreferGood( const std::vector< std::vector<PlanePair> > &sample_pairs )
{
    if( sample_pairs.size() > 0 )
    {
        int id1 = rand() % sample_pairs.size();
        int id2 = rand() % sample_pairs.size();
        if(id1 > id2)
            id1 = id2; //use smaller one => increases chance for lower id
        return sample_pairs[id1];
    }
    else
        return std::vector<PlanePair>();
}

std::vector<PlanePair> Tracking::randomChoosePlanePairs( const std::vector< std::vector<PlanePair> > &sample_pairs )
{
    if( sample_pairs.size() > 0 )
    {
        int id = rand() % sample_pairs.size();
        return sample_pairs[id];
    }
    else
        return std::vector<PlanePair>();
}

std::vector<cv::DMatch> Tracking::randomChooseMatchesPreferGood( const unsigned int sample_size,
                                                                const vector< cv::DMatch > &matches_with_depth )
{
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size)
    {
        int id1 = rand() % matches_with_depth.size();
        int id2 = rand() % matches_with_depth.size();
        if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
            sampled_ids.insert(id1);
        if(++safety_net > 2000)
        {
            ROS_ERROR("Infinite Sampling");
            break;
        }
    }

    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve( sampled_ids.size() );
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids)
    {
        sampled_matches.push_back(matches_with_depth[id]);
    }
    return sampled_matches;
}

std::vector<cv::DMatch> Tracking::randomChooseMatches( const unsigned int sample_size,
                                                    const vector< cv::DMatch > &matches )
{
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches.size() >= sample_size)
    {
        int id = rand() % matches.size();
        sampled_ids.insert(id);
        if(++safety_net > 2000)
        {
            ROS_ERROR("Infinite Sampling");
            break;
        }
    }

    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve( sampled_ids.size() );
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids)
    {
        sampled_matches.push_back(matches[id]);
    }
    return sampled_matches;
}

void Tracking::trackingReconfigCallback(plane_slam::TrackingConfig &config, uint32_t level)
{
    feature_good_match_threshold_ = config.feature_good_match_threshold;
    feature_min_good_match_size_ = config.feature_min_good_match_size;
    ransac_sample_size_ = config.ransac_sample_size;
    ransac_iterations_ = config.ransac_iterations;
    ransac_min_inlier_ = config.ransac_min_inlier;
    ransac_inlier_max_mahal_distance_ = config.ransac_inlier_max_mahal_distance;
    //
    icp_max_distance_ = config.icp_max_distance;
    icp_iterations_ = config.icp_iterations;
    icp_tf_epsilon_ = config.icp_tf_epsilon;
    icp_min_indices_ = config.icp_min_indices;
    icp_score_threshold_ = config.icp_score_threshold;
    //
    pnp_iterations_ = config.pnp_iterations;
    pnp_min_inlier_ = config.pnp_min_inlier;
    pnp_repreject_error_ = config.pnp_repreject_error;
    //

    cout << GREEN <<" Tracking Config." << RESET << endl;
}

} // end of namespace plane_slam
