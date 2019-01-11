/**
 * zmq_pointcloud_transport.h
 * Copyright (c) Santosh Thoduka, 2018
 * License: MPLv2.0 https://www.mozilla.org/en-US/MPL/2.0/
 */

#ifndef ZMQ_POINTCLOUD_TRANSPORT_H
#define ZMQ_POINTCLOUD_TRANSPORT_H

#include <zmq.hpp>
#include <memory>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace ZPT
{
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

        struct PointField{
            int offset;
            int datatype;
            int count;
        };

        struct PointCloudHeader{
            int width;
            int height;
            bool is_dense;
            bool is_bigendian;
            int point_step;
            int row_step;
            int num_fields;
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
            pc_header.num_fields = pc->fields.size();
            PointField *fields = new PointField[pc_header.num_fields];
            for (int i = 0; i < pc_header.num_fields; i++)
            {
                fields[i].offset = pc->fields[i].offset;
                fields[i].datatype = pc->fields[i].datatype;
                fields[i].count = pc->fields[i].count;
            }

            zmq::message_t header_msg(sizeof pc_header);
            std::memcpy(header_msg.data(), (const void *)&(pc_header), header_msg.size());
            bool rc = socket->send(header_msg, ZMQ_SNDMORE);

            for (int i = 0; i < pc_header.num_fields; i++)
            {
                zmq::message_t field_msg(sizeof (struct PointField));
                std::memcpy(field_msg.data(), (const void*)&(fields[i]), field_msg.size());
                // send PointField
                rc = socket->send(field_msg, ZMQ_SNDMORE) && rc;
                zmq::message_t name(pc->fields[i].name.size());
                std::memcpy(name.data(), pc->fields[i].name.data(), pc->fields[i].name.size());
                // send name of PointField
                rc = socket-> send(name, ZMQ_SNDMORE) && rc;
            }

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
            delete fields;
            return rc;
        }

        bool receivePointCloud(pcl::PCLPointCloud2::Ptr &pc, std::shared_ptr<zmq::socket_t> &socket)
        {
            zmq::message_t header_msg;
            zmq::message_t data_msg;

            PointCloudHeader pc_header;

            bool rc = socket->recv(&header_msg);
            if (rc)
            {
                std::memcpy(&pc_header, header_msg.data(), header_msg.size());
            }
            int num_fields = pc_header.num_fields;
            for (int i = 0; i < num_fields; i++)
            {
                zmq::message_t field_msg;
                zmq::message_t name_msg;
                int64_t more;
                size_t more_size = sizeof(more);
                socket->getsockopt(ZMQ_RCVMORE, &more, &more_size);
                if (!more)
                {
                    std::cerr << "malformed PCL message received" << std::endl;
                    return false;
                }
                // receive PointField message
                rc = socket->recv(&field_msg) && rc;

                PointField field;
                std::memcpy(&field, field_msg.data(), field_msg.size());
                pcl::PCLPointField pcl_field;
                pcl_field.offset = field.offset;
                pcl_field.datatype = field.datatype;
                pcl_field.count = field.count;
                socket->getsockopt(ZMQ_RCVMORE, &more, &more_size);
                if (!more)
                {
                    std::cerr << "malformed PCL message received" << std::endl;
                    return false;
                }
                // receive name of PointField
                rc = socket->recv(&name_msg) && rc;
                pcl_field.name.assign(static_cast<char *>(name_msg.data()), name_msg.size());
                pc->fields.push_back(pcl_field);
            }
            int64_t more;
            size_t more_size = sizeof(more);
            socket->getsockopt(ZMQ_RCVMORE, &more, &more_size);
            if (more)
            {
                rc = socket->recv(&data_msg) && rc;
            }

            pc->width = pc_header.width;
            pc->height = pc_header.height;
            pc->is_dense = pc_header.is_dense;
            pc->is_bigendian = pc_header.is_bigendian;
            pc->point_step = pc_header.point_step;
            pc->row_step = pc_header.row_step;
            pc->data.resize(data_msg.size());
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
                publisher->bind("ipc:///tmp/pointcloud_pub.ipc");
                server.reset(new zmq::socket_t(context, ZMQ_REP));
                server->bind("ipc:///tmp/pointcloud_srv.ipc");
            }
            if (type == RECEIVER || type == SENDER_AND_RECEIVER)
            {
                subscriber.reset(new zmq::socket_t(context, ZMQ_SUB));
                subscriber->connect("ipc:///tmp/pointcloud_pub.ipc");
                subscriber->setsockopt(ZMQ_SUBSCRIBE, "", 0);
                client.reset(new zmq::socket_t(context, ZMQ_REQ));
                client->connect("ipc:///tmp/pointcloud_srv.ipc");
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
