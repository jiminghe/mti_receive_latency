#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <array>
#include <vector>
#include <iomanip> // For std::setw and std::setfill
#include <fstream> // For std::ofstream
#include <ctime>   // For time formatting
#include <chrono>  // For the timer
#include <thread>  // For threading
#include <linux/serial.h>
#include <termios.h>
#include <sys/ioctl.h>

using namespace boost::asio;

// Function to set low latency mode on the serial port
void setLowLatencyMode(serial_port& serial) {
    int fd = serial.native_handle();
    struct serial_struct serial_info;
    if (ioctl(fd, TIOCGSERIAL, &serial_info) == -1) {
        perror("TIOCGSERIAL");
        return;
    }
    serial_info.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &serial_info) == -1) {
        perror("TIOCSSERIAL");
    }
}

// Function to adjust VMIN and VTIME settings
void setSerialPortOptions(serial_port& serial) {
    int fd = serial.native_handle();
    struct termios tio;
    if (tcgetattr(fd, &tio) == -1) {
        perror("tcgetattr");
        return;
    }
    tio.c_cc[VMIN] = 0;   // Minimum number of characters to read
    tio.c_cc[VTIME] = 1;  // Timeout in tenths of a second (0.1s)
    if (tcsetattr(fd, TCSANOW, &tio) == -1) {
        perror("tcsetattr");
    }
}

// High-resolution clock function
double getCurrentUTCTime() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts); // High-resolution monotonic clock
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;
}

class SerialReader {
public:
    SerialReader(io_service& io, const std::string& port, unsigned int baud_rate, int duration_seconds)
        : io_(io), serial_(io, port), timer_(io, boost::posix_time::seconds(duration_seconds)) {
        serial_.set_option(serial_port_base::baud_rate(baud_rate));
        setLowLatencyMode(serial_);
        setSerialPortOptions(serial_);
        timer_.async_wait(boost::bind(&SerialReader::handle_timer, this));
        start_read();
    }

    void start_read() {
        // Start an asynchronous read operation with a minimum read size
        boost::asio::async_read(serial_, boost::asio::buffer(buf_),
            boost::asio::transfer_at_least(1), // Adjust as needed
            boost::bind(&SerialReader::handle_read, this, placeholders::error, placeholders::bytes_transferred));
    }

    void handle_read(const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (!ec) {
            // Copy the received data
            std::vector<uint8_t> data(buf_.data(), buf_.data() + bytes_transferred);
            // Start the next read operation immediately
            start_read();
            // Process the data asynchronously
            io_.post(boost::bind(&SerialReader::process_data, this, data));
        } else if (ec != boost::asio::error::operation_aborted) {
            std::cerr << "Error: " << ec.message() << std::endl;
        }
    }

    void handle_timer() {
        // Stop reading from serial port and write the collected data to CSV
        std::cout << "Duration elapsed, stopping read operation and writing to CSV." << std::endl;
        serial_.cancel(); // Cancel the serial read operation
        write_to_csv();
    }

    void write_to_csv() {
        // Write the collected UTC timestamps to a CSV file
        std::ofstream outfile("data_log.csv");
        outfile << "UTC_Timestamp\n";
        for (const auto& timestamp : utc_timestamps_) {
            outfile << std::fixed << std::setprecision(9) << timestamp << "\n";
        }
        outfile.close();
        std::cout << "Data written to data_log.csv successfully." << std::endl;
    }

private:
    void process_data(const std::vector<uint8_t>& data) {
        // Append the received data to our internal buffer
        data_buffer_.insert(data_buffer_.end(), data.begin(), data.end());

        // Check if we have a complete sentence
        while (data_buffer_.size() >= 5) { // Minimum length of a valid sentence
            if (data_buffer_[0] == 0xFA && data_buffer_[1] == 0xFF) {
                // We have the start of a sentence, check the length
                std::size_t data_len = data_buffer_[3];

                // Check if we have the full sentence available
                if (data_buffer_.size() >= 4 + data_len + 1) { // 4 header bytes + data_len + checksum
                    // Extract the full sentence
                    std::vector<uint8_t> imu_sentence(data_buffer_.begin(), data_buffer_.begin() + 4 + data_len + 1);

                    // Verify checksum
                    if (verify_checksum(imu_sentence)) {
                        // Process the complete sentence (e.g., store timestamp)
                        process_sentence(imu_sentence);
                    } else {
                        std::cerr << "Checksum mismatch, discarding sentence" << std::endl;
                    }

                    // Remove the processed sentence from the buffer
                    data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + 4 + data_len + 1);
                } else {
                    // Not enough data yet to process the full sentence
                    break;
                }
            } else {
                // Invalid start byte, discard it and continue
                data_buffer_.erase(data_buffer_.begin());
            }
        }
    }

    bool verify_checksum(const std::vector<uint8_t>& sentence) {
        // Calculate checksum: sum from the second byte to the last-second byte
        uint8_t calculated_checksum = 0;
        for (std::size_t i = 1; i < sentence.size() - 1; ++i) {
            calculated_checksum -= sentence[i];
        }
        calculated_checksum &= 0xFF; // Take the lower byte
        return calculated_checksum == sentence.back();
    }

    void process_sentence(const std::vector<uint8_t>& sentence) {
        // Store the UTC timestamp of the received IMU sentence
        double utc_time = getCurrentUTCTime();
        utc_timestamps_.push_back(utc_time);

        // // Print the complete sentence in hex format for demonstration
        // std::cout << "Complete IMU Sentence: ";
        // for (auto byte : sentence) {
        //     std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)byte << " ";
        // }
        // std::cout << " | UTC Time: " << std::fixed << std::setprecision(9) << utc_time << std::endl;
    }

    io_service& io_;
    serial_port serial_;
    deadline_timer timer_;
    std::array<char, 512> buf_; // Increased buffer size
    std::vector<uint8_t> data_buffer_;         // To accumulate data until a complete sentence is detected
    std::vector<double> utc_timestamps_;       // To store the UTC timestamps of complete sentences
};

int main() {
    try {
        io_service io;
        boost::posix_time::seconds timeToRun(10); // Run for 10 seconds
        SerialReader reader(io, "/dev/ttyUSB0", 115200, timeToRun.total_seconds());

        // Run the I/O service in a separate thread
        std::thread io_thread([&io]() { io.run(); });

        // Wait for the I/O thread to finish
        io_thread.join();

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
