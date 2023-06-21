#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "visualization_msgs/Marker.h"
#include "pcl/features/normal_3d.h"
#include "time.h"
#include "limits"
#include "iostream"
#include "math.h"
#include "float.h"

ros::Publisher pub_pointcloud;
ros::Publisher pub_marker;
typedef pcl::PointXYZ PointT;

// This function checks a point for NaN value. If Point value is NaN the function returns 1, otherwise 0.
bool checkPointNaN(pcl::PointXYZ& input)
{
    if (input.x == input.x)
    {
        return false;
    }
    return true;
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

    ROS_INFO_STREAM(point_z_max.z);
    return point_z_max;
}

// This function returns the point with the highest z value from a pointcloud.
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

                if (!checkPointNaN(point_temp))
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

            if (!checkPointNaN(point_temp))
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

// This function adds the highest negative z value of the pointcloud, to each point of a pointcloud, so minimum z value will be 0.0.
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

float getAngle(pcl::PointXYZ& a, pcl::PointXYZ& b)
{
    float ak;
    float gk;
    float r;

    if (a.x == b.x)
    {
        // calculate ak
        if (a.y < b.y)
        {
            ak = b.y - a.y;
        }
        else
        {
            ak = a.y - b.y;
        }

        // calculate gk
        if (a.z < b.z)
        {
            gk = b.z - a.z;
        }
        else
        {
            gk = a.z - b.z;
        }

        //calculate tan
        r = atan(gk/ak);
        //return (r * 180.0) / M_PI;
        return 1.0;
    }
    else if (a.y == b.y)
    {
        // calculate ak
        if (a.x < b.x)
        {
            ak = b.x - a.x;
        }
        else
        {
            ak = a.x - b.x;
        }

        // calculate gk
        if (a.z < b.z)
        {
            gk = b.z - a.z;
        }
        else
        {
            gk = a.z - b.z;
        }

        //calculate tan
        r = atan(gk/ak);
        ROS_INFO_STREAM("r: " << r << ", ak: " << ak << ", gk: " << gk);
        ROS_INFO_STREAM("a.X: " << a.x << ", b.X: " << b.x);
        ROS_INFO_STREAM("a.Y: " << a.y << ", b.Y: " << b.y);
        ROS_INFO_STREAM("a.Z: " << a.z << ", b.Z: " << b.z);
        return (r * 180.0) / M_PI;
        //return 2.0;
    }
    else
    {
        //ROS_INFO_STREAM("a.X: " << a.x << ", b.X: " << b.x << ", a.Y: " << a.y << ", b.Y: " << b.y);
        return 0.0;
        
    }
}

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

    // created RandomSampleConsensus object and compute the appropriated model
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

void findCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    // Ransac
    pcl::ModelCoefficients::Ptr coefficients_ptr (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    
    ne.setKSearch(2);

    ROS_INFO_STREAM("Compute normals");

    ne.compute(*cloud_normals_ptr);

    ROS_INFO_STREAM("Start SEC");

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.05);
    seg.setMaxIterations (100000);
    seg.setDistanceThreshold(0.9);
    seg.setRadiusLimits(0.01, 0.9);
    seg.setInputCloud(input);
    seg.setInputNormals (cloud_normals_ptr);
    seg.setNumberOfThreads(8);
    seg.segment(*inliers_ptr, *coefficients_ptr);

    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(false);
    extr_inliers_filter.filter(*input);
    
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud (input);
    pass_filter.setFilterFieldName ("z");
    pass_filter.setFilterLimits (input->at(inliers->indices.at(100)).z,1.0);
    pass_filter.setNegative (false);
    pass_filter.filter (*input);
    */
}

// This function sets the marker position to the given point.
void setMarker(visualization_msgs::MarkerPtr& input_marker, pcl::PointXYZ input_point)
{
    input_marker->header.frame_id = "depth_camera_link";
    input_marker->header.stamp = ros::Time::now();
    input_marker->type = visualization_msgs::Marker::SPHERE;
    input_marker->ns = "z_max";
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
    input_marker->color.r = 1.0f;
    input_marker->color.g = 0.0f;
    input_marker->color.b = 0.0f;
    input_marker->color.a = 1.0;

    input_marker->lifetime = ros::Duration();
}

void dpCallback(const sensor_msgs::PointCloud2& sen_msg_pc2)
{
    /* // start timer
    //struct timespec begin, end;
    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &begin);
    */

    // create pcl pointcloud and convert sensor_msgs pointcloud2 to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(sen_msg_pc2, *pcl_pc_ptr);

    // create a marker and pointer
    visualization_msgs::MarkerPtr marker_ptr(new visualization_msgs::Marker);

    // fix tilted pointcloud
    rectifyTilt(pcl_pc_ptr);

    moveUp(pcl_pc_ptr);

    // remove every point from the pointcloud representing the groundplane
    removeGroundPlane(pcl_pc_ptr);

    // setup marker
    setMarker(marker_ptr, getPointMaxZ(pcl_pc_ptr));

    // find cylinder
    findCylinder(pcl_pc_ptr);

    // publish marker to "visualization_marker" topic
    pub_marker.publish(marker_ptr);

    // publish pointcloud to "pclEdit" topic
    pub_pointcloud.publish(*pcl_pc_ptr);

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