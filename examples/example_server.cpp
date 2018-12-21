#include <zmq_pointcloud_transport/zmq_pointcloud_transport.h>

int main(int argc, char *argv[])
{
    ZPT::ZMQPointCloudTransport zpt;
    zpt.init(ZPT::SENDER);
    usleep(100000);

    pcl::PointCloud<ZPT::PointT>::Ptr cloud (new pcl::PointCloud<ZPT::PointT>);

    cloud->width = 100000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].r = 255;
        cloud->points[i].g = 0;
        cloud->points[i].b = 255;
    }

    pcl::PCLPointCloud2::Ptr pc(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *pc);

    while (true)
    {
        while (!zpt.checkRequests())
        {
            usleep(10000);
        }
        zpt.reply(pc);
        usleep(10000);
    }

    return 0;
}
