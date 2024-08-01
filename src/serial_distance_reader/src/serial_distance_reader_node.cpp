#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace boost::asio;
using namespace std;

class SerialPortReader {
public:
    SerialPortReader(io_service& io, const string& port, unsigned int baud_rate, const string& topic)
        : serial_(io, port), topic_(topic) {
        serial_.set_option(serial_port_base::baud_rate(baud_rate));
        serial_.set_option(serial_port_base::character_size(8));
        serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        ros::NodeHandle nh;
        publisher_ = nh.advertise<std_msgs::Float32>(topic_, 1000);

        read_start();
    }

private:
    void read_start() {
        serial_.async_read_some(buffer(data_, max_length),
            boost::bind(&SerialPortReader::read_complete, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

    void read_complete(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!error) {
            string data(data_, bytes_transferred);
            stringstream ss(data);
            float distance;
            while (ss >> distance) {
                std_msgs::Float32 msg;
                msg.data = distance;
                publisher_.publish(msg);
            }
            read_start();
        } else {
            cerr << "Error: " << error.message() << endl;
        }
    }

    serial_port serial_;
    string topic_;
    ros::Publisher publisher_;
    enum { max_length = 1024 };
    char data_[max_length];
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_distance_reader_node");
    ros::NodeHandle nh("~");

    string port;
    int baud_rate;
    string topic;

    nh.param("port", port, string("/dev/ttyUSB1"));
    nh.param("baud_rate", baud_rate, 115200);
    nh.param("topic", topic, string("/distance_data"));

    try {
        io_service io;
        SerialPortReader reader(io, port, baud_rate, topic);
        io.run();
    } catch (std::exception& e) {
        cerr << "Exception: " << e.what() << endl;
    }

    return 0;
}
