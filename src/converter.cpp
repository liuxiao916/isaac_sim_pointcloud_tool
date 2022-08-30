#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::string output_type;
std::string lidar_topic = "/point_cloud"; 

// VLP-16 
int N_SCAN = 16;
int Horizon_SCAN = 1800;    

static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

ros::Subscriber subPC;
ros::Publisher pubPC;

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::PointCloud2 &old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg.header;
    pc_new_msg.header.frame_id = "velodyne";
    pubPC.publish(pc_new_msg);
}

void lidar_handle(sensor_msgs::PointCloud2 pc_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {

        PointXYZIRT new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = 0; 

        //16 ring. The range of index is 0~15. Up to Down.
        float ang_bottom = 15.0+0.1;
        float ang_res_y = 2;
        float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180 / M_PI;
        float rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        
        new_point.ring = int(rowIdn);
        new_point.time = (point_id / N_SCAN)*0.1/Horizon_SCAN ;

        pc_new->points.push_back(new_point);
    }
    publish_points(pc_new, pc_msg);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rs_converter");
    ros::NodeHandle nh;

    subPC = nh.subscribe(lidar_topic, 1, lidar_handle);
    
    pubPC = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);

    ROS_INFO("Listening to lidar topic ......");
    ros::spin();
    return 0;
}