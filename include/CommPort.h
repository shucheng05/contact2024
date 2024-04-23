#ifndef ROBO_CV_COMMPORT_H
#define ROBO_CV_COMMPORT_H

#include <serial/serial.h>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <chrono>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>
#include <Checksum.h>

constexpr size_t packet_size = 15;

struct TxPacket {
    unsigned char cache[packet_size];

    unsigned char &operator[](int p) {
        return cache[p];
    }

    unsigned char operator[](int p) const {
        return cache[p];
    }

    unsigned char *operator&() {
        return cache;
    }

    /*Constructor*/
    TxPacket() {
        memset(cache, 0, sizeof(0));
    }
};

class CommPort {
private:

public:

    typedef struct ProjectileRx
    {
        uint8_t header;
        float q[3];
        uint8_t vision_mode;
        uint8_t shoot_remote;
        uint8_t armor_color;
        float bullet_speed;
        uint16_t enemy_hp[6];
        uint32_t timestamp;
        float yaw;
        float pitch;
        uint8_t checksum;
    }  __attribute__((packed));

    ProjectileRx rx_struct_{};

    std::atomic<bool> read_stop_flag_{};
    std::atomic<bool> write_stop_flag_{};
    std::atomic<bool> write_clear_flag_{};
    std::atomic<bool> exception_handled_flag_{};

    uint8_t tx_buffer_[10]{};
    uint8_t rx_buffer_[49]{};
    serial::Serial port_;

    std::shared_ptr<spdlog::logger> logger_;
    std::vector<serial::PortInfo> serial_port_info_;
    std::string device_desc_;


    enum SERIAL_MODE {
        TX_SYNC,
        TX_RX_ASYNC
    };

    CommPort();

    ~CommPort();

    void RunAsync(SERIAL_MODE mode);

    void Start();

    void Stop();

    void Write(const uint8_t *tx_packet, size_t size, bool safe_write);

    void Read();

    void RxHandler();

    void SerialFailsafeCallback(bool reopen);

    float get_Pitch();
    
    float get_Yaw();

    float* getQuaternion() {
        return rx_struct_.q;
    }
};


#endif //ROBO_CV_COMMPORT_H
