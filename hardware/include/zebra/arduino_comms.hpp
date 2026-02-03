#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <iomanip>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  bool read_encoder_values(double &val_1, double &val_2)
  {
    if (!connected()) return false;

    // Check if data is available to avoid blocking forever
    if (serial_conn_.IsDataAvailable()) 
    {
        try 
        {
            std::string response;
            // Read until newline
            serial_conn_.ReadLine(response, '\r\n', timeout_ms_);

            // Check if the line starts with "ENC"
            // We use rfind to check from index 0
            if (response.rfind("ENC", 0) == 0)
            {
                // Remove "ENC " prefix (4 chars)
                std::string data_part = response.substr(4);
                std::stringstream ss(data_part);
                
                // Parse the two double values
                if (ss >> val_1 >> val_2) {
                    return true;}
            }
        } 
        catch (const LibSerial::ReadTimeout&) 
        {
            // It's okay if we don't get data every single cycle
        }
        catch (...)
        {
            // Ignore parsing errors
        }
    }
    return false;
  }
  void set_motor_values(double val_1, double val_2)
  {
    if (!connected()) return;

    std::stringstream ss;
    // Send standard CMD format
    ss << "CMD " << std::fixed << std::setprecision(3) << val_1 << " " << val_2 << "\r\n";
    
    try {
        serial_conn_.Write(ss.str());
    } catch (...) {
        std::cerr << "Failed to write to serial port!" << std::endl;
    }
  }

  // void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  // {
  //   std::stringstream ss;
  //   ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
  //   send_msg(ss.str());
  // }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP