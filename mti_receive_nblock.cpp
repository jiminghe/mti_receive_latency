#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <array>
#include <vector>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <chrono>
#include <thread>
#include <linux/serial.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <atomic>

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
void setTermiosOptions(serial_port& serial) {
    int fd = serial.native_handle();
    struct termios options;
    if (tcgetattr(fd, &options) == -1) {
        perror("tcgetattr");
        return;
    }
    options.c_iflag &= ~(IXOFF | IXON | IXANY); // Disable software flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; // 0.1s timeout
    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        perror("tcsetattr");
    }
}

// High-resolution clock function
double getCurrentUTCTime() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts); // High-resolution monotonic clock
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) / 1e9;
}

// Function to set real-time priority
void setRealtimePriority() {
    struct sched_param sched;
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sched) == -1) {
        perror("sched_setscheduler");
        std::cerr << "Warning: Unable to set real-time priority. Proceeding without it." << std::endl;
    }
}

class SerialReader {
public:
    SerialReader(io_service& io, const std::string& port, unsigned int baud_rate, int duration_seconds)
        : io_(io), serial_(io, port), timer_(io), duration_seconds_(duration_seconds) {
        serial_.set_option(serial_port_base::baud_rate(baud_rate));
        setLowLatencyMode(serial_);
        setTermiosOptions(serial_);
    }

    void start() {
        start_time_ = std::chrono::steady_clock::now();
        start_async_read();
        // Set a timer to stop the io_service after the duration
        timer_.expires_from_now(boost::posix_time::seconds(duration_seconds_));
        timer_.async_wait(boost::bind(&SerialReader::handle_timer, this));
        // Run the io_service in a separate thread
        io_thread_ = std::thread([this]() { io_.run(); });
    }

    void join() {
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        write_to_csv();
    }

private:
    void start_async_read() {
        serial_.async_read_some(boost::asio::buffer(buf_),
            boost::bind(&SerialReader::handle_read, this, placeholders::error, placeholders::bytes_transferred));
    }

    void handle_read(const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (!ec) {
            double utc_time = getCurrentUTCTime();
            std::vector<uint8_t> data(buf_.data(), buf_.data() + bytes_transferred);
            process_data(data, utc_time);
            start_async_read(); // Continue reading
        } else if (ec != boost::asio::error::operation_aborted) {
            std::cerr << "Error: " << ec.message() << std::endl;
        }
    }

    void handle_timer() {
        serial_.cancel(); // Stop any pending async operations
    }

    void process_data(const std::vector<uint8_t>& data, double utc_time) {
        // Append the received data to our internal buffer
        data_buffer_.insert(data_buffer_.end(), data.begin(), data.end());

        // Process the data as before
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
                        // Process the complete sentence
                        process_sentence(imu_sentence, utc_time);
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

    void process_sentence(const std::vector<uint8_t>& sentence, double utc_time) {
        // Store the UTC timestamp of the received IMU sentence
        utc_timestamps_.push_back(utc_time);
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

    io_service& io_;
    serial_port serial_;
    deadline_timer timer_;
    int duration_seconds_;
    std::thread io_thread_;
    std::chrono::steady_clock::time_point start_time_;
    std::array<char, 512> buf_;
    std::vector<uint8_t> data_buffer_;
    std::vector<double> utc_timestamps_;
};

int main() {
    // Attempt to set real-time priority
    setRealtimePriority();

    try {
        io_service io;
        int duration_seconds = 10; // Run for 10 seconds
        SerialReader reader(io, "/dev/ttyUSB0", 115200, duration_seconds);

        reader.start();

        // Wait for the reader to finish
        reader.join();

    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
