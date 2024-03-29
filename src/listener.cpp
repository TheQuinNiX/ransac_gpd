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
#include "pcl/filters/crop_box.h"
#include "dynamic_reconfigure/server.h"
#include "ransac_gpd/parametersConfig.h"

ros::Publisher publisher_pointcloud;
ros::Publisher publisher_pointcloud_normals;
ros::Publisher publisher_pointcloud_debug;
ros::Publisher publisher_vis_marker;
tf2_ros::Buffer tf_buffer;
int int_setKSearch;
double double_setNormalDistanceWeight;
double double_setDistanceThreshold;

void callback(ransac_gpd::parametersConfig &config, u_int32_t level)
{
    ROS_INFO("Parameters changed!");
    int_setKSearch = config.int_setKSearch;
    double_setNormalDistanceWeight = config.double_setNormalDistanceWeight;
    double_setDistanceThreshold = config.double_setDistanceThreshold;
}

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
    pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.004);
    seg.setInputCloud(input);
    seg.segment(*inliers_ptr, *coefficients_ptr);
    
    /*
    pcl::ExtractIndices<pcl::PointXYZ> extr_inliers_filter;
    extr_inliers_filter.setInputCloud(input);
    extr_inliers_filter.setIndices(inliers);
    extr_inliers_filter.setNegative(true);
    extr_inliers_filter.filter(*input);
    */
    
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(input);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(getPointMaxZ(input, inliers_ptr).z, 1.0);
    pass_filter.setNegative(false);
    pass_filter.filter(*input);
}

void addCropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(input);
    crop_box_filter.setNegative(false);
    crop_box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop_box_filter.filter(*input);
}

// This function uses RANSAC to fit a cylinder in to the given pointcloud.
void findCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::ModelCoefficients::Ptr& output_coefficients, pcl::PointIndices::Ptr& output_point_indicies)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    //Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);
    ne.setNumberOfThreads(8);
    ne.setKSearch(int_setKSearch);
    ne.setInputCloud(input);
    ne.setSearchMethod(tree);
    ROS_INFO_STREAM("Calculate normals...");
    ne.compute(*cloud_normals_ptr);

    // publish pointcloud to "pclEditNormals" topic
    publisher_pointcloud_normals.publish(*cloud_normals_ptr);

    // create RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg (true);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (double_setNormalDistanceWeight);
    seg.setMaxIterations (30000);
    seg.setDistanceThreshold(double_setDistanceThreshold);
    seg.setRadiusLimits(0.002, 0.012);
    //seg.setRadiusLimits(0.0002, 0.0008);
    //seg.setRadiusLimits(0.02, 0.04);
    //seg.setAxis(axis);
    //seg.setEpsAngle(10.0f * (M_PI/180.0f));
    seg.setInputCloud(input);
    seg.setInputNormals(cloud_normals_ptr);
    seg.setNumberOfThreads(8);
    ROS_INFO_STREAM("Calculate cylinder...");
    seg.segment(*output_point_indicies, *output_coefficients);

    ROS_INFO_STREAM("Cylinder coefficients (x, y, z, ax, ay, az): ");
    for (int i = 0; i < 7; i++)
    {
        ROS_INFO_STREAM(i+1 << " = " << output_coefficients->values.at(i));
    }
    
    ROS_INFO_STREAM("Cylinder inliers: " << output_point_indicies->indices.size());
}

// This function sets a sphere marker to the given point.
void setSphereMarker(pcl::PointXYZ input_point, std::string input_namespace, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
{
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::SPHERE;
    marker_ptr->ns = input_namespace;
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker
    marker_ptr->pose.position.x = input_point.x;
    marker_ptr->pose.position.y = input_point.y;
    marker_ptr->pose.position.z = input_point.z;
    marker_ptr->pose.orientation.x = 0.0;
    marker_ptr->pose.orientation.y = 0.0;
    marker_ptr->pose.orientation.z = 0.0;
    marker_ptr->pose.orientation.w = 1.0;
  
    // Set the scale of the marker
    marker_ptr->scale.x = 0.005;
    marker_ptr->scale.y = 0.005;
    marker_ptr->scale.z = 0.005;
  
    // Set the color of the marker
    marker_ptr->color.r = input_color_r;
    marker_ptr->color.g = input_color_g;
    marker_ptr->color.b = input_color_b;
    marker_ptr->color.a = input_color_a;

    marker_ptr->lifetime = ros::Duration();

    publisher_vis_marker.publish(marker_ptr);
}

// This function removes the sphere marker.
void removeSphereMarker(std::string input_namespace)
{
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::SPHERE;
    marker_ptr->ns = input_namespace;
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::DELETEALL;

    publisher_vis_marker.publish(marker_ptr);
}

// This function sets a cylinder marker to the given coefficients.
void setCylinderMarker(pcl::ModelCoefficients::Ptr& input_coefficients)
{    
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    Eigen::Quaternionf quaternion_from_coefficients;

    quaternion_from_coefficients = obtainCylinderOrientationFromModel(*input_coefficients);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::CYLINDER;
    marker_ptr->ns = "cylinder";
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::ADD;

    // Set the position and orientation of the marker
    marker_ptr->pose.position.x = input_coefficients->values.at(0);
    marker_ptr->pose.position.y = input_coefficients->values.at(1);
    marker_ptr->pose.position.z = input_coefficients->values.at(2);
    marker_ptr->pose.orientation.x = quaternion_from_coefficients.x();
    marker_ptr->pose.orientation.y = quaternion_from_coefficients.y();
    marker_ptr->pose.orientation.z = quaternion_from_coefficients.z();
    marker_ptr->pose.orientation.w = quaternion_from_coefficients.w();
  
    // Set the scale of the marker
    marker_ptr->scale.x = input_coefficients->values.at(6)*2;
    marker_ptr->scale.y = input_coefficients->values.at(6)*2;
    marker_ptr->scale.z = 20.0;
  
    // Set the color of the marker
    marker_ptr->color.r = 1.0f;
    marker_ptr->color.g = 1.0f;
    marker_ptr->color.b = 1.0f;
    marker_ptr->color.a = 0.6;

    marker_ptr->lifetime = ros::Duration();

    publisher_vis_marker.publish(marker_ptr);
}

// This function removes the existing cylinder marker.
void removeCylinderMarker()
{
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::CYLINDER;
    marker_ptr->ns = "cylinder";
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::DELETEALL;

    publisher_vis_marker.publish(marker_ptr);
}

// This function sets a points marker to the given point indices.
void setPointListMarker(pcl::PointIndices::Ptr& input_point_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, std::string input_namespace, float input_color_r, float input_color_g, float input_color_b, float input_color_a)
{    
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::POINTS;
    marker_ptr->ns = input_namespace;
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::ADD;

    int input_point_indices_size = input_point_indices->indices.size();

    for (int i = 0; i < input_point_indices_size; i++)
    {
        geometry_msgs::Point p;

        p.x = input_pointcloud->at(input_point_indices->indices.at(i)).x;
        p.y = input_pointcloud->at(input_point_indices->indices.at(i)).y;
        p.z = input_pointcloud->at(input_point_indices->indices.at(i)).z;
        //ROS_INFO_STREAM("Inlier: " << i+1 << " - " << "x: " << p.x << " ,y: " << p.y << " ,z: " << p.z);
        marker_ptr->points.push_back(p);
    }
  
    // Set the scale of the marker
    marker_ptr->scale.x = 0.003;
    marker_ptr->scale.y = 0.003;
    marker_ptr->scale.z = 0.003;
  
    // Set the color of the marker
    marker_ptr->color.r = input_color_r;
    marker_ptr->color.g = input_color_g;
    marker_ptr->color.b = input_color_b;
    marker_ptr->color.a = input_color_a;

    marker_ptr->lifetime = ros::Duration();

    publisher_vis_marker.publish(marker_ptr);
}

// This function removes the existing points marker.
void removePointListMarker(std::string input_namespace)
{    
    visualization_msgs::Marker::Ptr marker_ptr (new visualization_msgs::Marker);

    marker_ptr->header.frame_id = "mount_plate_link";
    marker_ptr->header.stamp = ros::Time::now();
    marker_ptr->type = visualization_msgs::Marker::POINTS;
    marker_ptr->ns = input_namespace;
    marker_ptr->id = 0;
    marker_ptr->action = visualization_msgs::Marker::DELETEALL;

    publisher_vis_marker.publish(marker_ptr);
}

void removeAllMarkers()
{
    removeCylinderMarker();
    removePointListMarker("cylinder_inliers");
    removeSphereMarker("z_max_all");
    removeSphereMarker("z_max_cylinder");
    removeSphereMarker("centroid_all");
    removeSphereMarker("centroid_cylinder_neighbors");
    removeSphereMarker("centroid_cylinder_inliers");
    removePointListMarker("cylinder_neighbors");
}

pcl::PointIndices::Ptr removeNotDirectPointNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pointcloud, pcl::PointXYZ search_point)
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

        if (kdtree.radiusSearch (search_point, radius, point_indices_radius_search, point_radius_squared_distance) > 0)
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

    sensor_msgs::PointCloud2 sen_msg_pc2_tf;

    //pcl_ros::transformPointCloud("mount_plate_link", sen_msg_pc2, sen_msg_pc2_tf, tfBuffer);

    if (sen_msg_pc2_tf.data.size() < 1)
    {
        //return;
    }

    // create pcl pointcloud and convert sensor_msgs pointcloud2 to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_filtered_ptr;
    pcl::fromROSMsg(sen_msg_pc2, *pcl_pc_ptr);

    //DEBUG publish pointcloud to "pclEdit" topic
    publisher_pointcloud_debug.publish(*pcl_pc_ptr);

    pcl::PointIndices::Ptr cylinder_inliers_ptr (new pcl::PointIndices);
    pcl::PointIndices::Ptr cylinder_max_z_neighbors_ptr (new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr cylinder_coefficients_ptr (new pcl::ModelCoefficients);

    //removeAllMarkers();

    ROS_INFO_STREAM("Seq. Nr.: " << pcl_pc_ptr->header.seq);

    // fix tilted pointcloud
    rectifyTilt(pcl_pc_ptr);

    moveUp(pcl_pc_ptr);

    // remove every point from the pointcloud representing the groundplane
    removeGroundPlane(pcl_pc_ptr);

    //addCropBox(pcl_pc_ptr, 0.25, 0.05, -1.0, 0.65, 0.4, 10.0);

    // publish pointcloud to "pclEdit" topic
    publisher_pointcloud.publish(*pcl_pc_ptr);

    // publish sphere marker at overall z max, color: purple
    setSphereMarker(getPointMaxZ(pcl_pc_ptr), "z_max_all", 0.5, 0.0, 0.5, 1.0);

    // publish points marker at centroid overall points, color: green
    setSphereMarker(getCentroidPoint(pcl_pc_ptr), "centroid_all", 0.0, 1.0, 0.0, 1.0);

    // find cylinder and publish markers
    findCylinder(pcl_pc_ptr, cylinder_coefficients_ptr, cylinder_inliers_ptr);
    
    if (cylinder_inliers_ptr->indices.size() > 0)
    {
        // publish cylinder marker
        setCylinderMarker(cylinder_coefficients_ptr);

        // publish points marker at cylinder inliers, color: blue
        setPointListMarker(cylinder_inliers_ptr, pcl_pc_ptr, "cylinder_inliers", 0.0, 0.0, 1.0, 1.0);

        // publish sphere marker at cylinder inliers z max, color: blue
        setSphereMarker(getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr), "z_max_cylinder", 0.0, 0.0, 1.0, 1.0);

        // publish sphere marker at cylinder inliers centroid color: orange
        setSphereMarker(getCentroidPoint(pcl_pc_ptr, cylinder_inliers_ptr), "centroid_cylinder_inliers", 1.0, 0.0, 0.0, 1.0);
    }

    //pcl_pc_filtered_ptr = getNewPcFromIndices(pcl_pc_ptr, cylinder_inliers_ptr);

    //cylinder_max_z_neighbors_ptr = removeNotDirectPointNeighbors(pcl_pc_filtered_ptr, getPointMaxZ(pcl_pc_ptr, cylinder_inliers_ptr));

    // publish sphere marker at cylinder z max neighbors centroid, color: red
    //setSphereMarker(getCentroidPoint(pcl_pc_filtered_ptr, cylinder_max_z_neighbors_ptr), "centroid_cylinder_neighbors", 1.0, 0.0, 0.0, 1.0);

    // publish points marker at cylinder z max neighbors, color: red
    //setPointListMarker(cylinder_max_z_neighbors_ptr, pcl_pc_filtered_ptr, "cylinder_neighbors", 1.0, 0.0, 0.0, 1.0);

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
    ROS_INFO("Starting listener node...");
    
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<ransac_gpd::parametersConfig> server;
    dynamic_reconfigure::Server<ransac_gpd::parametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    tf2_ros::TransformListener tfListener(tf_buffer);

    publisher_pointcloud = n.advertise<sensor_msgs::PointCloud2>("pointcloud_edited", 1);
    publisher_pointcloud_normals = n.advertise<sensor_msgs::PointCloud2>("pointcloud_edited_normals", 1);
    publisher_pointcloud_debug = n.advertise<sensor_msgs::PointCloud2>("pointcloud_debug", 1);
    publisher_vis_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber subscriber_pointcloud = n.subscribe("depth/points", 10, dpCallback);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}