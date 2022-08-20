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

using boost::asio::ip::udp;

// UDP Comms
boost::asio::io_service io_service;
udp::socket s(io_service, udp::endpoint(udp::v4(), 4242));
udp::socket status_socket(io_service);
std::array<uint8_t, 1024> bufferArray;

std::mutex mutex;
udp::endpoint status_endpoint;
bool has_messages = false;

// ROS Interface
ros::ServiceClient mowerMotorClient, emergencyClient;
geometry_msgs::Twist twist;

bool mowerEnabled = false;

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
    uint8_t emergency;
    float v_charge;
    float v_battery;
    float charge_current;
}__attribute__((packed));
#pragma pack(pop)

void ioThread() {
    ROS_INFO_STREAM("IO thread started");
    while (ros::ok()) {
        io_service.run_one();
    }
    ROS_INFO_STREAM("shutting down socket");

    // Shut down socket in order to cancel the loop below if no messages arrive
    try {
        s.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    } catch (boost::exception &e) {
        // NOP
    }
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

void statusReceived(const mower_msgs::Status::ConstPtr &msg) {
    mutex.lock();
    if (has_messages) {
        struct status_struct status = {
                .emergency = msg->emergency,
                .v_charge = msg->v_charge,
                .v_battery = msg->v_battery,
                .charge_current = msg->charge_current
        };
        status_socket.send_to(boost::asio::buffer(&status, sizeof(status)), udp::endpoint(status_endpoint.address(), 4243));
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

    udp::endpoint current_sender;

    while (ros::ok()) {
        try {
            udp::endpoint sender;
            size_t size = s.receive_from(boost::asio::buffer(bufferArray), sender);

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
                }

            } else {
                ROS_INFO_STREAM("Got packet of invalid size:" << size);
            }
        } catch (boost::exception &e) {
            ROS_ERROR_STREAM("Got boost exception:" << boost::diagnostic_information(e));
        }
    }


    spinner.stop();


    return 0;
}
