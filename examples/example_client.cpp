#include <zmq_pointcloud_transport/zmq_pointcloud_transport.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc, char *argv[])
{
    ZPT::ZMQPointCloudTransport zpt;
    zpt.init(ZPT::RECEIVER);
    usleep(100000);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    pcl::PCLPointCloud2::Ptr pc(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    while (true)
    {
        zpt.request();
        zpt.receive(pc);
        pcl::fromPCLPointCloud2(*pc, *cloud);
        viewer.showCloud(cloud);
        usleep(10000);
    }
    return 0;
}
