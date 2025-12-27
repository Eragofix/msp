#include "AsyncCsvLogger.hpp"

#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono;

AsyncCsvLogger::AsyncCsvLogger(std::string dir, std::string prefix, size_t max_queue)
    : dir_(std::move(dir)), prefix_(std::move(prefix)), max_queue_(max_queue)
{
    try {
        std::filesystem::create_directories(dir_);
    } catch (...) {
        // best-effort
    }

    filename_ = makeFilename();
    file_.open(filename_, std::ios::out | std::ios::trunc);

    if (!file_.is_open()) {
        // nie uruchamiamy wątku jeśli plik się nie otworzył
        running_ = false;
        return;
    }

    file_ << std::fixed << std::setprecision(6);
    file_ << "timestamp_utc;"
          << "roll_rad;"
          << "omega_used_rad_s;"
          << "omega_gyro_raw_rad_s;"
          << "omega_gyro_filt_rad_s;"
          << "u_raw;"
          << "u_sat;"
          << "motor1;"
          << "motor2;"
          << "motor3;"
          << "motor4\n";

    running_ = true;
    worker_ = std::thread(&AsyncCsvLogger::workerLoop, this);
}

AsyncCsvLogger::~AsyncCsvLogger() {
    stop();
}

void AsyncCsvLogger::push(const LogRow& row) {
    // jeśli logger nie działa albo plik nie jest otwarty — nic nie rób
    if (!running_.load(std::memory_order_relaxed) || !file_.is_open()) return;

    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (queue_.size() >= max_queue_) {
            queue_.pop_front(); // drop oldest
        }
        queue_.push_back(row);
    }
    cv_.notify_one();
}

void AsyncCsvLogger::stop() {
    // jeśli już zatrzymany, wyjdź
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        // expected było true; jeśli nie weszło, to znaczy że już false
        // ale i tak spróbuj dołączyć wątek jeśli istnieje
    }

    cv_.notify_all();
    if (worker_.joinable()) worker_.join();

    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

std::string AsyncCsvLogger::makeFilename() const {
    std::string fname = dir_;
    if (!fname.empty() && fname.back() != '/' && fname.back() != '\\') fname += '/';
    fname += prefix_;
    fname += currentUtcTimestampForFilename();
    fname += ".csv";
    return fname;
}

std::string AsyncCsvLogger::currentUtcTimestampForFilename() {
    auto now = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);

    std::tm tm_utc;
#if defined(_WIN32)
    gmtime_s(&tm_utc, &now_c);
#else
    gmtime_r(&now_c, &tm_utc);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string AsyncCsvLogger::currentUtcTimestamp() {
    auto now = system_clock::now();
    auto now_us = time_point_cast<microseconds>(now);
    auto us = now_us.time_since_epoch() % seconds(1);

    std::time_t now_c = system_clock::to_time_t(now);
    std::tm tm_utc;
#if defined(_WIN32)
    gmtime_s(&tm_utc, &now_c);
#else
    gmtime_r(&now_c, &tm_utc);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%d_%H:%M:%S")
        << '.' << std::setw(6) << std::setfill('0') << us.count();
    return oss.str();
}

void AsyncCsvLogger::workerLoop() {
    // zapisujemy paczkami, żeby minimalnie trzymać mutex
    while (running_.load(std::memory_order_relaxed) || !queue_.empty()) {
        std::deque<LogRow> local;

        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [&] {
                return !running_.load(std::memory_order_relaxed) || !queue_.empty();
            });

            local.swap(queue_);
        }

        for (const auto& row : local) {
            if (!file_.is_open()) break;

            file_ << currentUtcTimestamp() << ';'
                  << row.roll_rad << ';'
                  << row.omega_used_rad_s << ';'
                  << row.omega_gyro_raw_rad_s << ';'
                  << row.omega_gyro_filt_rad_s << ';'
                  << row.u_raw << ';'
                  << row.u_sat << ';'
                  << row.motor1 << ';'
                  << row.motor2 << ';'
                  << row.motor3 << ';'
                  << row.motor4 << '\n';

            if ((++lines_written_ % FLUSH_EVERY) == 0) {
                file_.flush();
            }
        }
    }

    if (file_.is_open()) file_.flush();
}
