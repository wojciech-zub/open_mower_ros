//
// Created by Clemens Elflein on 15.03.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/Status.h"
#include "sensor_msgs/Imu.h"
#include "boost/version.hpp"

using boost::asio::ip::udp;
class client
{
public:
    client(const udp::endpoint& listen_endpoint)
            : socket_(io_context_, listen_endpoint)
    {
    }

    std::size_t receive(const boost::asio::mutable_buffer& buffer,
                        std::chrono::steady_clock::duration timeout,
                        udp::endpoint &sender,
                        boost::system::error_code& error)
    {
        // Start the asynchronous operation. The handle_receive function used as a
        // callback will update the error and length variables.
        std::size_t length = 0;
        socket_.async_receive_from(boost::asio::buffer(buffer), sender,
                              [capture0 = &error, capture1 = &length](auto && PH1, auto && PH2) { return client::handle_receive(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), capture0, capture1); });

        // Run the operation until it completes, or until the timeout.
        run(timeout);

        return length;
    }

private:
    void run(std::chrono::steady_clock::duration timeout)
    {
        // Restart the io_context, as it may have been left in the "stopped" state
        // by a previous operation.
        io_context_.restart();

        // Block until the asynchronous operation has completed, or timed out. If
        // the pending asynchronous operation is a composed operation, the deadline
        // applies to the entire operation, rather than individual operations on
        // the socket.
        io_context_.run_for(timeout);

        // If the asynchronous operation completed successfully then the io_context
        // would have been stopped due to running out of work. If it was not
        // stopped, then the io_context::run_for call must have timed out.
        if (!io_context_.stopped())
        {
            // Cancel the outstanding asynchronous operation.
            socket_.cancel();

            // Run the io_context again until the operation completes.
            io_context_.run();
        }
    }

    static void handle_receive(
            const boost::system::error_code& error, std::size_t length,
            boost::system::error_code* out_error, std::size_t* out_length)
    {
        *out_error = error;
        *out_length = length;
    }

private:
    boost::asio::io_context io_context_;
    udp::socket socket_;
};


// UDP Comms
boost::asio::io_service io_service;
client c( udp::endpoint(udp::v4(), 4242));
udp::socket status_socket(io_service);
std::array<uint8_t, 1024> bufferArray;

std::mutex mutex;
udp::endpoint status_endpoint;
bool has_messages = false;

// ROS Interface
ros::ServiceClient mowerMotorClient, emergencyClient;
geometry_msgs::Twist twist;

bool mowerEnabled = false;

sensor_msgs::Imu last_imu;

#pragma pack(push, 1)
struct control_struct {
    float speed;
    float rotation;
    uint8_t mower_enabled;
    uint8_t reset_emergency;
}__attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct status_struct {
    uint32_t stamp;
    uint8_t emergency;
    float v_charge;
    float v_battery;
    float charge_current;
    uint32_t ticks_left;
    uint32_t ticks_right;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
}__attribute__((packed));
#pragma pack(pop)

void ioThread() {
    ROS_INFO_STREAM("IO thread started");
    while (ros::ok()) {
        io_service.run_one();
    }
    ROS_INFO_STREAM("shutting down socket");


    ROS_INFO_STREAM("IO thread stopped");
}

void setEmergencyMode(bool emergency) {
    mower_msgs::EmergencyStopSrv emergencyStop;
    emergencyStop.request.emergency = emergency;
    emergencyClient.call(emergencyStop);
}

bool setMowerEnabled(bool enabled) {

    if (enabled == mowerEnabled) {
        return true;
    }

    mowerEnabled = enabled;

    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = enabled;
    mowerMotorClient.call(mow_srv);

    return true;
}

void imuReceived(const sensor_msgs::Imu::ConstPtr &imu) {
    last_imu = *imu;
}

void statusReceived(const mower_msgs::Status::ConstPtr &msg) {
    mutex.lock();
    if (has_messages) {
        struct status_struct status = {
                .stamp = static_cast<uint32_t>(msg->stamp.toNSec()/1000000),
                .emergency = msg->emergency,
                .v_charge = msg->v_charge,
                .v_battery = msg->v_battery,
                .charge_current = msg->charge_current,
                .ticks_left = msg->left_esc_status.tacho,
                .ticks_right = msg->right_esc_status.tacho,
                .gx = static_cast<float>(last_imu.angular_velocity.x),
                .gy = static_cast<float>(last_imu.angular_velocity.y),
                .gz = static_cast<float>(last_imu.angular_velocity.z),
                .ax = static_cast<float>(last_imu.linear_acceleration.x),
                .ay = static_cast<float>(last_imu.linear_acceleration.y),
                .az = static_cast<float>(last_imu.linear_acceleration.z),
        };
        try {
            status_socket.send_to(boost::asio::buffer(&status, sizeof(status)), udp::endpoint(status_endpoint.address(), 4243));
        } catch (boost::exception &e) {
            ROS_ERROR_STREAM("Got boost exception during status send:" << boost::diagnostic_information(e));
            has_messages = false;
        }

    }
    mutex.unlock();
}


int main(int argc, char **argv) {
    status_socket.open(udp::v4());

    ros::init(argc, argv, "mower_udp_bridge");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");


    twist.linear.x = 0;
    twist.angular.z = 0;
    has_messages = false;


    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    mowerMotorClient = n.serviceClient<mower_msgs::MowerControlSrv>(
            "mower_service/mow_enabled");
    emergencyClient = n.serviceClient<mower_msgs::EmergencyStopSrv>(
            "mower_service/emergency");

    ros::Subscriber status_sub = n.subscribe("/mower/status", 0, statusReceived,
                                              ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber imu_sub = n.subscribe("/imu/data_raw", 0, imuReceived,
                                              ros::TransportHints().tcpNoDelay(true));


    if (paramNh.param("wait_for_services", true)) {
        ROS_INFO("Waiting for mower service");
        if (!mowerMotorClient.waitForExistence(ros::Duration(60.0, 0.0))) {
            ROS_ERROR("Mower service not found.");
            return 1;
        }

        ROS_INFO("Waiting for emergency service");
        if (!emergencyClient.waitForExistence(ros::Duration(60.0, 0.0))) {
            ROS_ERROR("Emergency server not found.");
            return 1;
        }
    }

    boost::thread io_thread(ioThread);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    setMowerEnabled(false);
    setEmergencyMode(true);

    udp::endpoint current_sender;

//    s.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 200 });

    while (ros::ok()) {
        try {
            udp::endpoint sender;
            boost::system::error_code error;
            size_t size = c.receive(boost::asio::buffer(bufferArray),
                      std::chrono::seconds(1), sender,error);

            mutex.lock();
            status_endpoint = sender;
            has_messages = true;
            mutex.unlock();

            if (size == sizeof(struct control_struct)) {
                auto *control = (struct control_struct *) bufferArray.data();
                // send the twist to ROS
                twist.linear.x = control->speed;
                twist.angular.z = control->rotation;
                cmd_vel_pub.publish(twist);

                setMowerEnabled(control->mower_enabled);

                if (control->reset_emergency) {
                    ROS_WARN_STREAM("Resetting Emergency Mode");
                    setEmergencyMode(false);
                }

            } else {
                if(size != 0) {
                    ROS_INFO_STREAM("Got packet of invalid size:" << size);
                }
            }
        } catch (boost::exception &e) {
            ROS_ERROR_STREAM("Got boost exception:" << boost::diagnostic_information(e));
        }
    }


    spinner.stop();


    return 0;
}
