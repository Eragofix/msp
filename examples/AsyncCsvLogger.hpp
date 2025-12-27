#pragma once
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <atomic>
#include <cstdint>

struct LogRow {
    double roll_rad{};
    double omega_used_rad_s{};
    double omega_gyro_raw_rad_s{};
    double omega_gyro_filt_rad_s{};
    double u_raw{};
    double u_sat{};
    uint16_t motor1{};
    uint16_t motor2{};
    uint16_t motor3{};
    uint16_t motor4{};
};

class AsyncCsvLogger {
public:
    AsyncCsvLogger(std::string dir, std::string prefix, size_t max_queue = 4096);
    ~AsyncCsvLogger();

    void push(const LogRow& row);
    void stop();

    bool isOpen() const { return file_.is_open(); }
    std::string filename() const { return filename_; }

    AsyncCsvLogger(const AsyncCsvLogger&) = delete;
    AsyncCsvLogger& operator=(const AsyncCsvLogger&) = delete;

private:
    void workerLoop();
    std::string makeFilename() const;
    static std::string currentUtcTimestamp();            // with microseconds
    static std::string currentUtcTimestampForFilename(); // YYYYMMDD_HHMMSS

    const std::string dir_;
    const std::string prefix_;
    const size_t max_queue_;

    std::ofstream file_;
    std::string filename_;

    std::deque<LogRow> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic<bool> running_{false};

    // flush co N linii (żeby nie obciążać)
    uint64_t lines_written_{0};
    static constexpr uint64_t FLUSH_EVERY = 200;
};
