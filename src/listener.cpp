#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "pcl/point_types.h"
#include "tf2_eigen/tf2_eigen.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/passthrough.h"
#include "time.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/centroid.h"

ros::Publisher pub_pointcloud;
ros::Publisher pub_marker;

// This function checks a point for NaN value. If Point value is NaN the function returns 1, otherwise 0.
bool checkPointNaN(pcl::PointXYZ& input)
{
    if (input.x == input.x)
    {
        return false;
    }
    return true;
}

// Calculates quaternion from RANSAC model cofficients.
inline Eigen::Quaternionf obtainCylinderOrientationFromModel(pcl::ModelCoefficients& coefficients)
{
    Eigen::Vector3f axis_vector(coefficients.values.at(3), coefficients.values.at(4), coefficients.values.at(5));
    Eigen::Vector3f up_vector(0.0, 0.0, -1.0);
    Eigen::Vector3f right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();

    Eigen::Quaternionf q(Eigen::AngleAxisf(-1.0 * std::acos(axis_vector.dot(up_vector)), right_vector));
    q.normalize();

    return q;
}

// This function returns the point with the highest z value from a pointcloud.
pcl::PointXYZ getPointMaxZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::PointXYZ point_z_max;
    pcl::PointXYZ point_temp;

    if (input->isOrganized())
    {
        int input_row = input->height;
        int input_colum = input->width;

        for (int r = 0; r < input_row; r++)
        {
            for (int c = 0; c < input_colum; c++)
            {            
                point_temp = input->at(c, r);

                if (!checkPointNaN(point_temp))
                {                
                    if (point_z_max.z < point_temp.z)
                    {
                        point_z_max = point_temp;
                    }
                }
            }
        }
    }
    else
    {
        int input_length = input->size();

        for (int i = 0; i < input_length; i++)
        {            
            point_temp = input->at(i);

            if (!checkPointNaN(point_temp))
            {                
                if (point_z_max.z < point_temp.z)
                {
                    point_z_max = point_temp;
                }
            }
        }
    }

    return point_z_max;
}
pcl::PointXYZ getPointMaxZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointIndices::Ptr& indices_input)
{
    pcl::PointXYZ point_z_max;
    pcl::PointXYZ point_temp;

    int indices_input_length = indices_input->indices.size();

    for (int i = 0; i < indices_input_length; i++)
    {            
        point_temp = input->at(indices_input->indices.at(i));

        if (!checkPointNaN(point_temp))
        {                
            if (point_z_max.z < point_temp.z)
            {
                point_z_max = point_temp;
            }
        }
    }
    return point_z_max;
}

// Returns point with lowest z value from a pointcloud.
pcl::PointXYZ getPointMinZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::PointXYZ point_z_min;
    pcl::PointXYZ point_temp;

    if (input->isOrganized())
    {
        int input_row = input->height;
        int input_colum = input->width;

        for (int r = 0; r < input_row; r++)
        {
            for (int c = 0; c < input_colum; c++)
            {            
                point_temp = input->at(c, r);

                if (!checkPointNaN(point_temp) or (point_temp.z != 0.0))
                {                
                    if (point_z_min.z > point_temp.z)
                    {
                        point_z_min = point_temp;
                    }
                }
            }
        }
    }
    else
    {
        int input_length = input->size();

        for (int i = 0; i < input_length; i++)
        {            
            point_temp = input->at(i);

            if (!checkPointNaN(point_temp) or (point_temp.z != 0.0))
            {                
                if (point_z_min.z > point_temp.z)
                {
                    point_z_min = point_temp;
                }
            }
        }
    }

    return point_z_min;
}

// Adds highest negative z value of the pointcloud, to each point of a pointcloud, so minimum z value will be 0.0.
void moveUp(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::PointXYZ point_z_min = getPointMinZ(input);

    if (input->isOrganized())
    {
        int input_row = input->height;
        int input_colum = input->width;

        for (int r = 0; r < input_row; r++)
        {
            for (int c = 0; c < input_colum; c++)
            {
                if (!checkPointNaN(input->at(c,r)))
                {                
                    input->at(c,r).z = input->at(c,r).z + fabsf(point_z_min.z);
                }
            }
        }
    }
    else
    {
        int input_length = input->size();

        for (int i = 0; i < input_length; i++)
        {
            if (!checkPointNaN(input->at(i)))
            {
                input->at(i).z = input->at(i).z + fabsf(point_z_min.z);
            }
        }
    }
}

// This function subtracts the lowest z value of the pointcloud, from each point of a pointcloud, so minimum z value will be 0.0.
void moveDown(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::PointXYZ point_z_min = getPointMinZ(input);

    if (input->isOrganized())
    {
        int input_row = input->height;
        int input_colum = input->width;

        for (int r = 0; r < input_row; r++)
        {
            for (int c = 0; c < input_colum; c++)
            {
                if (!checkPointNaN(input->at(c,r)))
                {                
                    input->at(c,r).z = input->at(c,r).z - fabsf(point_z_min.z);
                }
            }
        }
    }
    else
    {
        int input_length = input->size();

        for (int i = 0; i < input_length; i++)
        {
            if (!checkPointNaN(input->at(i)))
            {
                input->at(i).z = input->at(i).z - fabsf(point_z_min.z);
                
            }
        }
    }
}

// Mirrors the pointcloud in z direction.
void mirrorPointcloudZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    int input_length = input->size();
    pcl::PointXYZ point_temp;

    for (int i = 0; i < input_length; i++)
    {
        point_temp = input->at(i);
        point_temp.z = point_temp.z * -1;
        input->push_back(point_temp);
    }
}

// Rotates the given pointcloud.
void rectifyTilt(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translation() << input->at(100, 100).x, input->at(100, 100).y, input->at(100, 100).z;
    transform.rotate(Eigen::AngleAxisf((-187*M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*input, *input, transform);
}

// This function uses RANSAC to remove all points representing the groundplane form a pointcloud.
void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    // Ransac
    pcl::ModelCoefficients::Ptr coefficients_ptr (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.004);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients_ptr);
    
    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(true);
    extr_inliers_filter.filter(*input);
    */

    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud (input);
    pass_filter.setFilterFieldName ("z");
    pass_filter.setFilterLimits (getPointMaxZ(input, inliers).z, 1.0);
    pass_filter.setNegative (false);
    pass_filter.filter (*input);
}

// This function uses RANSAC to fit a cylinder in to the given pointcloud.
void findCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::ModelCoefficients::Ptr& output_coefficients, pcl::PointIndices::Ptr& output_point_indicies)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    //Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);
    ne.setNumberOfThreads(8);
    ne.setKSearch(30);
    ne.setInputCloud(input);
    ne.setSearchMethod(tree);
    ne.compute(*cloud_normals_ptr);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg (true);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.2);
    seg.setMaxIterations (30000);
    seg.setDistanceThreshold(0.009);
    seg.setRadiusLimits(0.001, 0.015);
    //seg.setAxis(axis);
    //seg.setEpsAngle(10.0f * (M_PI/180.0f));
    seg.setInputCloud(input);
    seg.setInputNormals(cloud_normals_ptr);
    seg.setNumberOfThreads(8);
    seg.segment(*output_point_indicies, *output_coefficients);

    ROS_INFO_STREAM("Cylinder coefficients (x, y, z, ax, ay, az): ");
    for (int i = 0; i < 7; i++)
    {
        ROS_INFO_STREAM(i+1 << " = " << output_coefficients->values.at(i));
    }
    
    ROS_INFO_STREAM("Cylinder inliers: " << output_point_indicies->indices.size());

    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(false);
    extr_inliers_filter.filter(*input);
    */
}

// This function uses RANSAC to fit a circle in to the given pointcloud.
void findCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::ModelCoefficients::Ptr& output_coefficients, pcl::PointIndices::Ptr& output_point_indicies)
{
    Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentation<pcl::PointXYZ> seg (true);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (50000);
    seg.setDistanceThreshold(0.009);
    seg.setRadiusLimits(0.001, 0.015);
    seg.setAxis(axis);
    seg.setEpsAngle(10.0f * (M_PI/180.0f));
    seg.setInputCloud(input);
    seg.setNumberOfThreads(8);
    seg.segment(*output_point_indicies, *output_coefficients);

    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(false);
    extr_inliers_filter.filter(*input);
    */
}

// This function uses RANSAC to fit a line in to the given pointcloud.
void findLine(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::ModelCoefficients::Ptr& output_coefficients)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (100000);
    seg.setDistanceThreshold(0.001);
    //seg.setRadiusLimits(0.002, 0.003);
    //seg.setAxis(axis);
    //seg.setEpsAngle(10.0f * (M_PI/180.0f));
    seg.setInputCloud(input);
    seg.setNumberOfThreads(8);
    seg.segment(*inliers, *output_coefficients);

    ROS_INFO_STREAM("Line coefficients (x, y, z, ax, ay, az): ");
    for (int i = 0; i < 6; i++)
    {
        ROS_INFO_STREAM(i+1 << " = " << output_coefficients->values.at(i));
    }
    
    //ROS_INFO_STREAM("Cylinder inliers: " << inliers->indices.size());

    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(false);
    extr_inliers_filter.filter(*input);
    */
}

// This function sets a sphere marker to the given point.
void setSphereMarker(visualization_msgs::MarkerPtr& input_marker, pcl::PointXYZ input_point, std::string input_namespace, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
{
    input_marker->header.frame_id = "depth_camera_link";
    input_marker->header.stamp = ros::Time::now();
    input_marker->type = visualization_msgs::Marker::SPHERE;
    input_marker->ns = input_namespace;
    input_marker->id = 0;
    input_marker->action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker
    input_marker->pose.position.x = input_point.x;
    input_marker->pose.position.y = input_point.y;
    input_marker->pose.position.z = input_point.z;
    input_marker->pose.orientation.x = 0.0;
    input_marker->pose.orientation.y = 0.0;
    input_marker->pose.orientation.z = 0.0;
    input_marker->pose.orientation.w = 1.0;
  
    // Set the scale of the marker
    input_marker->scale.x = 0.01;
    input_marker->scale.y = 0.01;
    input_marker->scale.z = 0.01;
  
    // Set the color of the marker
    input_marker->color.r = input_color_r;
    input_marker->color.g = input_color_g;
    input_marker->color.b = input_color_b;
    input_marker->color.a = input_color_a;

    input_marker->lifetime = ros::Duration();
}

// This function sets a cylinder marker to the given coefficients.
void setCylinderMarker(visualization_msgs::MarkerPtr& input_marker, pcl::ModelCoefficients& input_coefficients)
{    
    Eigen::Quaternionf quaternion_from_coefficients;

    quaternion_from_coefficients = obtainCylinderOrientationFromModel(input_coefficients);

    input_marker->header.frame_id = "depth_camera_link";
    input_marker->header.stamp = ros::Time::now();
    input_marker->type = visualization_msgs::Marker::CYLINDER;
    input_marker->ns = "cylinder";
    input_marker->id = 0;
    input_marker->action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker
    input_marker->pose.position.x = input_coefficients.values.at(0);
    input_marker->pose.position.y = input_coefficients.values.at(1);
    input_marker->pose.position.z = input_coefficients.values.at(2);
    input_marker->pose.orientation.x = quaternion_from_coefficients.x();
    input_marker->pose.orientation.y = quaternion_from_coefficients.y();
    input_marker->pose.orientation.z = quaternion_from_coefficients.z();
    input_marker->pose.orientation.w = quaternion_from_coefficients.w();
  
    // Set the scale of the marker
    input_marker->scale.x = input_coefficients.values.at(6)*2;
    input_marker->scale.y = input_coefficients.values.at(6)*2;
    input_marker->scale.z = 20.0;
  
    // Set the color of the marker
    input_marker->color.r = 1.0f;
    input_marker->color.g = 1.0f;
    input_marker->color.b = 1.0f;
    input_marker->color.a = 0.6;

    input_marker->lifetime = ros::Duration();
}

// This function sets a line marker to the given coefficients.
void setLineMarker(visualization_msgs::MarkerPtr& input_marker, pcl::ModelCoefficients& input_coefficients)
{    
    // Convert axis vector to quarternion format
    double axis_roll = atan2(input_coefficients.values[5],input_coefficients.values[4]);
    double axis_pitch = -1.0 * atan2(input_coefficients.values[5],input_coefficients.values[3]);
    double axis_yaw = atan2(input_coefficients.values[4],input_coefficients.values[3]);

    tf2::Quaternion axis_quarternion;
    axis_quarternion.setRPY( 0.0, -0.5*M_PI + axis_pitch, axis_yaw);
    //axis_quarternion.setRPY( axis_roll, axis_pitch, axis_yaw);
    axis_quarternion.normalize();

    input_marker->header.frame_id = "depth_camera_link";
    input_marker->header.stamp = ros::Time::now();
    input_marker->type = visualization_msgs::Marker::CYLINDER;
    input_marker->ns = "cylinder";
    input_marker->id = 0;
    input_marker->action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker
    input_marker->pose.position.x = input_coefficients.values.at(0);
    input_marker->pose.position.y = input_coefficients.values.at(1);
    input_marker->pose.position.z = input_coefficients.values.at(2);
    input_marker->pose.orientation.x = axis_quarternion.getX();
    input_marker->pose.orientation.y = axis_quarternion.getY();
    input_marker->pose.orientation.z = axis_quarternion.getZ();
    input_marker->pose.orientation.w = axis_quarternion.getW();
  
    // Set the scale of the marker
    input_marker->scale.x = 0.01;
    input_marker->scale.y = 0.01;
    input_marker->scale.z = 10.0;
  
    // Set the color of the marker
    input_marker->color.r = 1.0f;
    input_marker->color.g = 1.0f;
    input_marker->color.b = 1.0f;
    input_marker->color.a = 0.8;

    input_marker->lifetime = ros::Duration();
}

// This function sets a points marker to the given point indices.
void setPointListMarker(visualization_msgs::MarkerPtr& input_marker, pcl::PointIndicesPtr& input_point_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, std::string input_namespace, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
{    
    input_marker->header.frame_id = "depth_camera_link";
    input_marker->header.stamp = ros::Time::now();
    input_marker->type = visualization_msgs::Marker::POINTS;
    input_marker->ns = input_namespace;
    input_marker->id = 0;
    input_marker->action = visualization_msgs::Marker::ADD;

    int input_point_indices_size = input_point_indices->indices.size();

    for (int i = 0; i < input_point_indices->indices.size(); i++)
    {
        geometry_msgs::Point p;

        p.x = input_pointcloud->at(input_point_indices->indices.at(i)).x;
        p.y = input_pointcloud->at(input_point_indices->indices.at(i)).y;
        p.z = input_pointcloud->at(input_point_indices->indices.at(i)).z;

        input_marker->points.push_back(p);
    }
  
    // Set the scale of the marker
    input_marker->scale.x = 0.003;
    input_marker->scale.y = 0.003;
    input_marker->scale.z = 0.003;
  
    // Set the color of the marker
    input_marker->color.r = input_color_r;
    input_marker->color.g = input_color_g;
    input_marker->color.b = input_color_b;
    input_marker->color.a = input_color_a;

    input_marker->lifetime = ros::Duration();
}

pcl::PointIndices::Ptr removeNotDirectPointNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, pcl::PointXYZ searchPoint)
{
    pcl::PointIndices::Ptr point_neighbors_indices_ptr (new pcl::PointIndices);

    if (input_pointcloud->size() > 0)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(input_pointcloud);
        
        std::vector<int> point_indices_radius_search;
        std::vector<float> point_radius_squared_distance;
        
        //float radius = 0.0001f * rand () / (RAND_MAX + 1.0f);
        float radius = 0.0000001f;

        if (kdtree.radiusSearch (searchPoint, radius, point_indices_radius_search, point_radius_squared_distance) > 0)
        {
            for (int i = 0; i < point_indices_radius_search.size(); i++)
            {
                point_neighbors_indices_ptr->indices.push_back(point_indices_radius_search[i]);
            }
        }
    }

    return point_neighbors_indices_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr getNewPcFromIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pointcloud, pcl::PointIndices::Ptr &input_indices)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extr_indices_filter;
    extr_indices_filter.setInputCloud(input_pointcloud);
    extr_indices_filter.setIndices(input_indices);
    extr_indices_filter.setNegative(false);
    extr_indices_filter.filter(*filtered_pointcloud);

    return filtered_pointcloud;
}

pcl::PointXYZ getCentroidPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pointcloud)
{
    // Create and accumulate points
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    for (int i = 0; i < input_pointcloud->size(); i++)
    {
        centroid.add(input_pointcloud->at(i));
    }

    pcl::PointXYZ point_centroid;
    centroid.get(point_centroid);

    return point_centroid;
}
pcl::PointXYZ getCentroidPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pointcloud, pcl::PointIndices::Ptr &input_indices)
{
    // Create and accumulate points
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    for (int i = 0; i < input_indices->indices.size(); i++)
    {
        centroid.add(input_pointcloud->at(input_indices->indices.at(i)));
    }

    pcl::PointXYZ point_centroid;
    centroid.get(point_centroid);

    return point_centroid;
}

void dpCallback(const sensor_msgs::PointCloud2& sen_msg_pc2)
{
    /* // start timer
    //struct timespec begin, end;
    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &begin);
    */

    // create pcl pointcloud and convert sensor_msgs pointcloud2 to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_filtered_ptr;
    pcl::fromROSMsg(sen_msg_pc2, *pcl_pc_ptr);

    pcl::PointIndices::Ptr cylinder_inliers_ptr (new pcl::PointIndices);
    pcl::PointIndices::Ptr circle_inliers_ptr (new pcl::PointIndices);
    pcl::PointIndices::Ptr cylinder_max_z_neighbors_ptr (new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr cylinder_coefficients_ptr (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circle_coefficients_ptr (new pcl::ModelCoefficients);

    // create a marker and pointer
    visualization_msgs::MarkerPtr sphere_marker_ptr(new visualization_msgs::Marker);
    visualization_msgs::MarkerPtr cylinder_marker_ptr(new visualization_msgs::Marker);
    visualization_msgs::MarkerPtr points_marker_ptr(new visualization_msgs::Marker);

    ROS_INFO_STREAM("Seq. Nr.: " << pcl_pc_ptr->header.seq);

    // fix tilted pointcloud
    rectifyTilt(pcl_pc_ptr);

    moveUp(pcl_pc_ptr);

    // remove every point from the pointcloud representing the groundplane
    removeGroundPlane(pcl_pc_ptr);

    //moveDown(pcl_pc_ptr);

    // mirror pointcloud
    //mirrorPointcloudZ(pcl_pc_ptr);

    // publish pointcloud to "pclEdit" topic
    pub_pointcloud.publish(*pcl_pc_ptr);

    // publish sphere marker at overall z max, color: purple
    setSphereMarker(sphere_marker_ptr, getPointMaxZ(pcl_pc_ptr), "z_max_all", 0.5, 0.0, 0.5, 1.0);
    pub_marker.publish(sphere_marker_ptr);

    // find cylinder and publish markers
    findCylinder(pcl_pc_ptr, cylinder_coefficients_ptr, cylinder_inliers_ptr);
    if (!cylinder_inliers_ptr->indices.empty())
    {
        // publish cylinder marker
        setCylinderMarker(cylinder_marker_ptr, *cylinder_coefficients_ptr);
        pub_marker.publish(cylinder_marker_ptr);

        // publish sphere marker at cylinder inliers z max, color: blue
        setSphereMarker(sphere_marker_ptr, getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr), "z_max_cylinder", 0.0, 0.0, 1.0, 1.0);
        pub_marker.publish(sphere_marker_ptr);

        // publish points marker at cylinder inliers, color: blue
        setPointListMarker(points_marker_ptr, cylinder_inliers_ptr, pcl_pc_ptr, "cylinder_inliers", 0.0, 0.0, 1.0, 1.0);
        pub_marker.publish(points_marker_ptr);
    }

    // publish points marker at centroid overall points, color: green
    setSphereMarker(sphere_marker_ptr, getCentroidPoint(pcl_pc_ptr), "centroid_all", 0.0, 1.0, 0.0, 1.0);
    pub_marker.publish(sphere_marker_ptr);

    //pcl_pc_filtered_ptr = getNewPcFromIndices(pcl_pc_ptr, cylinder_inliers_ptr);

    //cylinder_max_z_neighbors_ptr = removeNotDirectPointNeighbors(pcl_pc_filtered_ptr, getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr));

    // publish sphere marker at cylinder z max neighbors centroid, color: red
    //setSphereMarker(sphere_marker_ptr, getCentroidPoint(pcl_pc_filtered_ptr, cylinder_max_z_neighbors_ptr), "centroid_cylinder_neighbors", 1.0, 0.0, 0.0, 1.0);
    //pub_marker.publish(sphere_marker_ptr);

    // publish points marker at cylinder z max neighbors, color: red
    //setPointListMarker(points_marker_ptr, cylinder_max_z_neighbors_ptr, pcl_pc_filtered_ptr, "cylinder_neighbors", 1.0, 0.0, 0.0, 1.0);
    //pub_marker.publish(points_marker_ptr);

    /* // stop timer and get result
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);

    long seconds = end.tv_sec - begin.tv_sec;
    long nanoseconds = end.tv_nsec - begin.tv_nsec;
    double elapsed = seconds + nanoseconds*1e-9;
    
    ROS_INFO_STREAM("Time measured: " << elapsed);
    */
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting node");
    
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("pclEdit", 1);
    pub_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub = n.subscribe("depth/points", 10, dpCallback);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}