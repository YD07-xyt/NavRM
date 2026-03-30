#include "serial_driver.hpp"
#include <boost/asio/buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include"../../../tool/rm_log/include/glog.hpp"
#include <boost/asio/write.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <source_location>
#include <vector>

namespace io::serial {
  SerialDriver::SerialDriver():receive_buffer(1024){
    auto io_service_  =std::make_shared<boost::asio::io_service>();
    auto serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);
    get_param();  
    open();
  }
  SerialDriver::~SerialDriver(){
    serial_port_->close();
  }
  void SerialDriver::get_param() {
  // try {
  //   auto yaml_path = tools::load("io/serial_driver/config/serial.yaml");
  //   serialconfig.device_name =
  //       tools::read<std::string>(yaml_path, "device_name");
  //   serialconfig.baud_rate = tools::read<int>(yaml_path, "baud_rate");
  //   serialconfig.flow_control =
  //       tools::read<std::string>(yaml_path, "flow_control");
  //   serialconfig.parity = tools::read<std::string>(yaml_path, "parity");
  //   serialconfig.stop_bits =
  //       tools::read<std::string>(yaml_path, "stop_bits");
  // } catch (...) {
  //   rmlog::error("serial driver read yaml failed");
  // }
}

  void SerialDriver::open(){
    try {
      serial_port_->open(serialconfig.device_name);
      serial_port_->set_option(boost::asio::serial_port::baud_rate(serialconfig.baud_rate));
      serial_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
      serial_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
      serial_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
      //serial_port.set_option(boost::asio::serial_port::character_size(8));
    }catch(...){
      LOG(ERROR)<< RED << "serial port open failed"<< RESET;

    }
  }

  void SerialDriver::close(){
    if(serial_port_->is_open()){
        serial_port_->close();
        LOG(INFO)<< GREEN << "serial port is close"<< RESET;

    }
  }

  void SerialDriver::reopen_serial(){
    if(serial_port_->is_open()){
      LOG(INFO)<< BLUE << "serial_port not need reopen"<< RESET;
      return;
    }
    open();
  }

  size_t SerialDriver::send_data(const std::vector<uint8_t>& send_data){
      try{
        //serial_port_->write_some(circular_buffer);
        if ( send_data.size()==0) {
            return 0;
        }
        size_t bytes_written = boost::asio::write(
            *serial_port_,
            boost::asio::buffer(send_data.data(), send_data.size())
        );
        return bytes_written;
      }catch(...){
        LOG(ERROR)<< RED << "serial port send data is failed"<< RESET;
      }
      
  }

  std::vector<uint8_t> SerialDriver::receive_data(){
    if (!serial_port_->is_open()) {
      open();
      //return buffer;
      LOG(ERROR)<< RED << "serial port receive data is failed"<< RESET;
      //throw std::runtime_error("Serial port not open");
    }
    try{
      size_t bytes_read = boost::asio::read(
            *serial_port_,
            boost::asio::buffer(receive_buffer)
        );
      
      
    }catch(const boost::system::system_error& e){
      LOG(ERROR)<< RED << "serial port receive data is failed"<< RESET;
    }
    return receive_buffer;
  }
}; // namespace io