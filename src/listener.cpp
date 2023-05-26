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
#include "time.h"
#include "limits"
#include "iostream"
#include "math.h"
#include "float.h"

ros::Publisher pub;

/*
float findMaxHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
    float maxZ = 0.0;

    for (int x = 1; x < input->height; x++)
    {
        for (int y = 1; y < input->width; y++)
        {
            if (pcl::isFinite(input->at(x,y).z))
            {
                float tmpZ = input->at(x,y).z;
                if (maxZ < tmpZ)
                {
                    maxZ = tmpZ;
                }
                
            }
            
        }
    }

    return maxZ;
}
*/

template<typename T>
bool is_nan( const T &value )
{
    // True if NAN
    return value != value;
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
    transform.rotate(Eigen::AngleAxisf((-186*M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf((0*M_PI) / 180, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*input, *input, transform);
}

void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    // Ransac
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.004);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    
    pcl::ExtractIndices<pcl::PointXYZ> extrInliers;
    extrInliers.setInputCloud(input);
    extrInliers.setIndices(inliers);
    extrInliers.setNegative(true);
    extrInliers.filter(*input);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (input->at(inliers->indices.at(100)).z,1.0);
    pass.setNegative (false);
    pass.filter (*input);
}

void dpCallback(const sensor_msgs::PointCloud2& senMsgPc2)
{
    // start timer
    struct timespec begin, end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &begin);

    // create pcl pointcloud and convert sensor_msgs pointcloud2 to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPcPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(senMsgPc2, *pclPcPtr);

    // remove nan from pcl pointcloud
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*pclPcPtr,*pclPcPtr, indices);


    // fix tilted pointcloud
    rectifyTilt(pclPcPtr);

    /*
    if (pclPcPtr->at(0,0).x != pclPcPtr->at(0,0).x)
    {
        int c = 0;
        int r = 50;

        while ((pclPcPtr->at(c,r).x != pclPcPtr->at(c,r).x))
        {
            c = c + 1;
        }
        
        //ROS_INFO_STREAM("X("<< c <<",0): " << pclPcPtr->at(c,r).x << " Y("<< c <<",0): " << pclPcPtr->at(c,r).y << " Z("<< c <<",0): " << pclPcPtr->at(c,r).z);
        //ROS_INFO_STREAM("X("<< c+10 <<",0): " << pclPcPtr->at(c+10,r).x << " Y("<< c+10 <<",0): " << pclPcPtr->at(c+10,r).y << " Z("<< c+10 <<",0): " << pclPcPtr->at(c+10,r).z);
        ROS_INFO_STREAM("angle: " << getAngle(pclPcPtr->at(c,r),pclPcPtr->at(c+10,r)));
    }
    else
    {
        ROS_INFO_STREAM("X(0,0): " << pclPcPtr->at(0,0).x << " Y(0,0): " << pclPcPtr->at(0,0).x << " Z(0,0): " << pclPcPtr->at(0,0).x);
    }
    */



    // remove every point from the pointcloud representing the groundplane
    removeGroundPlane(pclPcPtr);

    // publish pointcloud to pclEdit topic
    pub.publish(*pclPcPtr);

    // stop timer and get result
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);

    long seconds = end.tv_sec - begin.tv_sec;
    long nanoseconds = end.tv_nsec - begin.tv_nsec;
    double elapsed = seconds + nanoseconds*1e-9;
    
    //ROS_INFO_STREAM("Time measured: " << elapsed);

}

int main(int argc, char **argv)
{
    ROS_INFO("Starting node");
    
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::PointCloud2>("pclEdit", 10);
    ros::Subscriber sub = n.subscribe("depth/points", 10, dpCallback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}