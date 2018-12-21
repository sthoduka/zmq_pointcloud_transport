/**
 * zmq_pointcloud_transport.h
 * Copyright (c) Santosh Thoduka, 2018
 * License: MPLv2.0 https://www.mozilla.org/en-US/MPL/2.0/
 */

#ifndef ZMQ_POINTCLOUD_TRANSPORT_H
#define ZMQ_POINTCLOUD_TRANSPORT_H

#include <zmq.hpp>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace ZPT
{
    typedef pcl::PointXYZRGB PointT;
    enum Type
    {
        SENDER,
        RECEIVER,
        SENDER_AND_RECEIVER
    };

    class ZMQPointCloudTransport
    {
    private:
        zmq::context_t context;
        std::shared_ptr<zmq::socket_t> publisher;
        std::shared_ptr<zmq::socket_t> subscriber;
        std::shared_ptr<zmq::socket_t> server;
        std::shared_ptr<zmq::socket_t> client;
        Type type;
        bool requested;

        struct PointCloudHeader{
            int width;
            int height;
            bool is_dense;
            bool is_bigendian;
            int point_step;
            int row_step;
        };

        bool sendPointCloud(const pcl::PCLPointCloud2::ConstPtr &pc, std::shared_ptr<zmq::socket_t> &socket)
        {
            PointCloudHeader pc_header;
            pc_header.width = pc->width;
            pc_header.height = pc->height;
            pc_header.is_dense = pc->is_dense;
            pc_header.is_bigendian = pc->is_bigendian;
            pc_header.point_step = pc->point_step;
            pc_header.row_step = pc->row_step;

            zmq::message_t header_msg(sizeof pc_header);
            std::memcpy(header_msg.data(), (const void *)&(pc_header), header_msg.size());
            bool rc = socket->send(header_msg, ZMQ_SNDMORE);

            if (pc->data.size() > 0)
            {
                zmq::message_t data_msg(pc->data.size() * sizeof(pc->data[0]));
                std::memcpy(data_msg.data(), (const void *)&(pc->data[0]), data_msg.size());
                rc = socket->send(data_msg) && rc;
            }
            else
            {
                zmq::message_t data_msg;
                rc = socket->send(data_msg) && rc;
            }
            return rc;
        }

        bool receivePointCloud(pcl::PCLPointCloud2::Ptr &pc, std::shared_ptr<zmq::socket_t> &socket)
        {
            zmq::message_t header_msg;
            zmq::message_t data_msg;

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            PointCloudHeader pc_header;

            bool rc = socket->recv(&header_msg);
            if (rc)
            {
                std::memcpy(&pc_header, header_msg.data(), header_msg.size());
            }
            int64_t more;
            size_t more_size = sizeof(more);
            socket->getsockopt(ZMQ_RCVMORE, &more, &more_size);
            if (more)
            {
                rc = socket->recv(&data_msg) && rc;
            }
            cloud->width = pc_header.width;
            cloud->height = pc_header.height;
            cloud->points.resize(cloud->width * cloud->height);

            pcl::toPCLPointCloud2(*cloud, *pc);
            pc->is_dense = pc_header.is_dense;
            pc->is_bigendian = pc_header.is_bigendian;
            pc->point_step = pc_header.point_step;
            pc->row_step = pc_header.row_step;
            std::memcpy(&(pc->data[0]), data_msg.data(), data_msg.size());
            return rc;
        }

        void sendRequest(std::shared_ptr<zmq::socket_t> &socket)
        {
            zmq::message_t msg;
            socket->send(msg);
        }

    public:
        ZMQPointCloudTransport() : context(1), requested(false)
        {
        }
        virtual ~ZMQPointCloudTransport()
        {
        }
        void init(Type type)
        {
            this->type = type;
            if (type == SENDER || type == SENDER_AND_RECEIVER)
            {
                publisher.reset(new zmq::socket_t(context, ZMQ_PUB));
                publisher->bind("ipc://pointcloud_pub.ipc");
                server.reset(new zmq::socket_t(context, ZMQ_REP));
                server->bind("ipc://pointcloud_srv.ipc");
            }
            if (type == RECEIVER || type == SENDER_AND_RECEIVER)
            {
                subscriber.reset(new zmq::socket_t(context, ZMQ_SUB));
                subscriber->connect("ipc://pointcloud_pub.ipc");
                subscriber->setsockopt(ZMQ_SUBSCRIBE, "", 0);
                client.reset(new zmq::socket_t(context, ZMQ_REQ));
                client->connect("ipc://pointcloud_srv.ipc");
            }
        }

        // server functions
        void publish(const pcl::PCLPointCloud2::ConstPtr &pc)
        {
            sendPointCloud(pc, publisher);
        }
        void reply(const pcl::PCLPointCloud2::ConstPtr &pc)
        {
            sendPointCloud(pc, server);
        }

        bool checkRequests()
        {
            zmq::message_t msg;
            return server->recv(&msg, ZMQ_DONTWAIT);
        }

        // client functions
        void receive(pcl::PCLPointCloud2::Ptr &pc)
        {
            if (requested)
            {
                receivePointCloud(pc, client);
            }
            else
            {
                receivePointCloud(pc, subscriber);
            }
            requested = false;
        }
        void request()
        {
            requested = true;
            sendRequest(client);
        }
    };
}

#endif /* ZMQ_POINTCLOUD_TRANSPORT_H */
