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

#include <boost/asio.hpp>
#include <boost/thread.hpp>


using boost::asio::ip::udp;


boost::asio::io_service io_service;
udp::socket s(io_service, udp::endpoint(udp::v4(), 4242));
std::array<uint8_t, 1024> bufferArray;


#pragma pack(push, 1)
struct control_struct {
    float power_left; float power_right; float power_mower;
}__attribute__((packed));
#pragma pack(pop)

void ioThread() {
    ROS_INFO_STREAM("IO thread started");
    while(ros::ok()) {
        io_service.run_one();
    }
    ROS_INFO_STREAM("shutting down socket");

    // Shut down socket in order to cancel the loop below if no messages arrive
    try {
        s.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    } catch(boost::exception &e) {

    }
    ROS_INFO_STREAM("IO thread stopped");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_udp_bridge");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    boost::thread io_thread(ioThread);



    // don't change, we need to wait for arduino to boot before actually sending stuff
    ros::Duration retryDelay(5, 0);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        try {
            size_t size = s.receive(boost::asio::buffer(bufferArray));
            if(size == sizeof(struct control_struct)) {
                struct control_struct* control = (struct control_struct*)bufferArray.data();
                ROS_INFO_STREAM_THROTTLE(0.1, "left value: " << control->power_left);
            } else {
                ROS_INFO_STREAM("Got packet of invalid size:" << size);
            }
        } catch (boost::exception &e) {
            ROS_ERROR_STREAM("Got boost exception:" << diagnostic_information_what(e));
        }
    }


    spinner.stop();


    return 0;
}
