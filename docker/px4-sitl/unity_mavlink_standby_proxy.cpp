// Licensed under the Apache License, Version 2.0.
// Lightweight MAVLink v2 TCP Standby & Monotonic Clock Proxy for PX4 SITL <-> Unity 6-DOF.
// Allows unmodified PX4-Autopilot (SimulatorMavlink.cpp) to connect to 127.0.0.1:4560
// and stay booted/alive with 250 Hz standby ground sensors when Unity is not in Play Mode,
// seamlessly bridging to Unity's TCP :4560 when Play Mode starts.

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t kMavlinkV2Stx = 0xFD;
constexpr uint8_t kMavlinkV1Stx = 0xFE;
constexpr uint32_t kMsgIdHilSensor = 107;
constexpr uint32_t kMsgIdHilGps = 113;
constexpr uint8_t kCrcExtraHilSensor = 108;
constexpr uint8_t kCrcExtraHilGps = 124;

uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
    uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                 (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

uint16_t mavlink_crc(const uint8_t* data, size_t len, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) crc = crc_accumulate(data[i], crc);
    return crc_accumulate(crc_extra, crc);
}

void write_u16_le(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void write_i16_le(uint8_t* p, int16_t v) { write_u16_le(p, static_cast<uint16_t>(v)); }

void write_i32_le(uint8_t* p, int32_t v) {
    uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

void write_u32_le(uint8_t* p, uint32_t v) { write_i32_le(p, static_cast<int32_t>(v)); }

void write_u64_le(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
}

uint64_t read_u64_le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= (static_cast<uint64_t>(p[i]) << (8 * i));
    return v;
}

void write_f32_le(uint8_t* p, float f) {
    uint32_t u = 0;
    std::memcpy(&u, &f, 4);
    write_u32_le(p, u);
}

size_t encode_mavlink_v2(uint32_t msgid, uint8_t seq, uint8_t crc_extra,
                         const uint8_t* payload, uint8_t plen, uint8_t* out) {
    out[0] = kMavlinkV2Stx;
    out[1] = plen;
    out[2] = 0; // incompat_flags
    out[3] = 0; // compat_flags
    out[4] = seq;
    out[5] = 1; // sysid
    out[6] = 1; // compid
    out[7] = static_cast<uint8_t>(msgid & 0xFF);
    out[8] = static_cast<uint8_t>((msgid >> 8) & 0xFF);
    out[9] = static_cast<uint8_t>((msgid >> 16) & 0xFF);
    std::memcpy(out + 10, payload, plen);
    uint16_t crc = mavlink_crc(out + 1, 9 + plen, crc_extra);
    write_u16_le(out + 10 + plen, crc);
    return 12 + plen;
}

size_t build_standby_hil_sensor(uint8_t seq, uint64_t time_usec, uint64_t tick, uint8_t* out) {
    uint32_t h = static_cast<uint32_t>(tick * 2654435761ULL);
    float n1 = (static_cast<int32_t>(h & 0xFFu) - 128) * 1.0e-4f;
    float n2 = (static_cast<int32_t>((h >> 8) & 0xFFu) - 128) * 1.0e-4f;
    float n3 = (static_cast<int32_t>((h >> 16) & 0xFFu) - 128) * 1.0e-4f;

    uint8_t p[64]{};
    write_u64_le(p + 0, time_usec);
    write_f32_le(p + 8, n1);
    write_f32_le(p + 12, n2);
    write_f32_le(p + 16, -9.80665f + n3);
    write_f32_le(p + 20, n1 * 0.1f);
    write_f32_le(p + 24, n2 * 0.1f);
    write_f32_le(p + 28, n3 * 0.1f);
    write_f32_le(p + 32, 0.32f + n1);
    write_f32_le(p + 36, 0.04f + n2);
    write_f32_le(p + 40, 0.40f + n3);
    write_f32_le(p + 44, 1007.26f + n1 * 0.05f); // ISA pressure at 50 m MSL (matches OriginAltitudeMsl)
    write_f32_le(p + 48, 0.0f);
    write_f32_le(p + 52, 50.0f + n2 * 0.05f);
    write_f32_le(p + 56, 20.0f);
    write_u32_le(p + 60, 0x1FFFu);
    return encode_mavlink_v2(kMsgIdHilSensor, seq, kCrcExtraHilSensor, p, 64, out);
}

size_t build_standby_hil_gps(uint8_t seq, uint64_t time_usec, uint8_t* out) {
    uint8_t p[36]{};
    write_u64_le(p + 0, time_usec);
    write_i32_le(p + 8, 375665000);
    write_i32_le(p + 12, 1269780000);
    write_i32_le(p + 16, 50000);
    write_u16_le(p + 20, 80);
    write_u16_le(p + 22, 120);
    write_u16_le(p + 24, 0);
    write_i16_le(p + 26, 0);
    write_i16_le(p + 28, 0);
    write_i16_le(p + 30, 0);
    write_u16_le(p + 32, 0);
    p[34] = 3;  // 3D fix
    p[35] = 14; // satellites
    return encode_mavlink_v2(kMsgIdHilGps, seq, kCrcExtraHilGps, p, 36, out);
}

bool set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool send_all(int fd, const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
#ifdef MSG_NOSIGNAL
        ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
#else
        ssize_t n = ::send(fd, data + sent, len - sent, 0);
#endif
        if (n > 0) {
            sent += static_cast<size_t>(n);
        } else if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd{fd, POLLOUT, 0};
            if (::poll(&pfd, 1, 50) <= 0) return false;
        } else {
            return false;
        }
    }
    return true;
}

int try_start_unity_connect(const char* host, uint16_t port) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    int yes = 1;
    (void)::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    set_nonblocking(sock);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        ::close(sock);
        return -1;
    }

    int rc = ::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc == 0 || errno == EINPROGRESS || errno == EALREADY) return sock;
    ::close(sock);
    return -1;
}

int64_t now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

} // namespace

int main(int argc, char** argv) {
    const char* unity_host = "192.168.5.2";
    uint16_t unity_port = 4560;
    uint16_t listen_port = 4560;

    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--host") == 0) unity_host = argv[++i];
        else if (std::strcmp(argv[i], "--port") == 0) unity_port = static_cast<uint16_t>(std::atoi(argv[++i]));
        else if (std::strcmp(argv[i], "--listen-port") == 0) listen_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    }

    int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        std::perror("[mavlink-proxy] socket");
        return 1;
    }
    int reuse = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(listen_port);
    ::inet_pton(AF_INET, "127.0.0.1", &bind_addr.sin_addr);

    if (::bind(srv, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0 || ::listen(srv, 1) < 0) {
        std::perror("[mavlink-proxy] bind/listen");
        ::close(srv);
        return 1;
    }

    std::printf("[mavlink-proxy] Listening on 127.0.0.1:%u -> Unity %s:%u\n",
                listen_port, unity_host, unity_port);
    std::fflush(stdout);

    while (true) {
        int px4_fd = ::accept(srv, nullptr, nullptr);
        if (px4_fd < 0) continue;

        int yes = 1;
        ::setsockopt(px4_fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        set_nonblocking(px4_fd);
        std::printf("[mavlink-proxy] PX4 connected.\n");
        std::fflush(stdout);

        uint64_t mono_time_usec = 1000000ULL;
        uint64_t last_unity_raw_usec = 0;
        bool has_unity_raw_usec = false;
        uint64_t standby_tick = 0;
        uint8_t standby_seq = 0;
        int64_t next_step_us = now_us();

        int unity_fd = -1;
        bool unity_stream_active = false;
        unsigned pending_handshake_ticks = 0;
        int64_t last_unity_rx_us = 0;

        std::vector<uint8_t> unity_rx;
        unity_rx.reserve(8192);
        uint8_t tmp_buf[4096];

        auto close_unity = [&]() {
            if (unity_fd >= 0) {
                ::close(unity_fd);
                unity_fd = -1;
            }
            unity_stream_active = false;
            pending_handshake_ticks = 0;
            has_unity_raw_usec = false;
            unity_rx.clear();
        };

        while (true) {
            struct pollfd pfds[2];
            nfds_t nfds = 1;
            pfds[0] = {px4_fd, POLLIN, 0};
            if (unity_fd >= 0) {
                pfds[1] = {unity_fd, POLLIN, 0};
                nfds = 2;
            }

            int poll_ms = unity_stream_active ? 10 : 2;
            int pr = ::poll(pfds, nfds, poll_ms);
            if (pr < 0) {
                if (errno == EINTR) continue;
                break;
            }

            if (pfds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) break;
            if (pfds[0].revents & POLLIN) {
                ssize_t n = ::recv(px4_fd, tmp_buf, sizeof(tmp_buf), 0);
                if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) break;
                if (n > 0 && unity_stream_active && unity_fd >= 0) {
                    if (!send_all(unity_fd, tmp_buf, static_cast<size_t>(n))) close_unity();
                }
            }

            if (nfds == 2) {
                if (pfds[1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
                    if (unity_stream_active) {
                        std::printf("[mavlink-proxy] Unity disconnected; returning to standby sensors.\n");
                        std::fflush(stdout);
                    }
                    close_unity();
                } else if (pfds[1].revents & POLLIN) {
                    ssize_t n = ::recv(unity_fd, tmp_buf, sizeof(tmp_buf), 0);
                    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        if (unity_stream_active) {
                            std::printf("[mavlink-proxy] Unity closed connection; returning to standby sensors.\n");
                            std::fflush(stdout);
                        }
                        close_unity();
                    } else if (n > 0) {
                        last_unity_rx_us = now_us();
                        unity_rx.insert(unity_rx.end(), tmp_buf, tmp_buf + n);

                        size_t pos = 0;
                        bool px4_ok = true;
                        while (pos < unity_rx.size()) {
                            uint8_t stx = unity_rx[pos];
                            if (stx == kMavlinkV2Stx) {
                                if (pos + 12 > unity_rx.size()) break;
                                uint8_t plen = unity_rx[pos + 1];
                                uint8_t incompat = unity_rx[pos + 2];
                                size_t flen = 12u + plen + ((incompat & 0x01u) ? 13u : 0u);
                                if (pos + flen > unity_rx.size()) break;

                                uint32_t msgid = static_cast<uint32_t>(unity_rx[pos + 7]) |
                                                 (static_cast<uint32_t>(unity_rx[pos + 8]) << 8) |
                                                 (static_cast<uint32_t>(unity_rx[pos + 9]) << 16);

                                if ((msgid == kMsgIdHilSensor || msgid == kMsgIdHilGps) && plen >= 8) {
                                    if (msgid == kMsgIdHilSensor && !unity_stream_active) {
                                        unity_stream_active = true;
                                        std::printf("[mavlink-proxy] Unity HIL_SENSOR stream active.\n");
                                        std::fflush(stdout);
                                    }
                                    uint64_t raw_t = read_u64_le(&unity_rx[pos + 10]);
                                    if (!has_unity_raw_usec || raw_t != last_unity_raw_usec) {
                                        if (has_unity_raw_usec && raw_t > last_unity_raw_usec &&
                                            (raw_t - last_unity_raw_usec) <= 200000ULL) {
                                            mono_time_usec += (raw_t - last_unity_raw_usec);
                                        } else {
                                            mono_time_usec += 4000ULL;
                                        }
                                        last_unity_raw_usec = raw_t;
                                        has_unity_raw_usec = true;
                                    }

                                    write_u64_le(&unity_rx[pos + 10], mono_time_usec);
                                    uint8_t crc_extra = (msgid == kMsgIdHilSensor) ? kCrcExtraHilSensor : kCrcExtraHilGps;
                                    uint16_t crc = mavlink_crc(&unity_rx[pos + 1], 9 + plen, crc_extra);
                                    write_u16_le(&unity_rx[pos + 10 + plen], crc);
                                }

                                if (!send_all(px4_fd, &unity_rx[pos], flen)) {
                                    px4_ok = false;
                                    break;
                                }
                                pos += flen;
                            } else if (stx == kMavlinkV1Stx) {
                                if (pos + 8 > unity_rx.size()) break;
                                size_t flen = 8u + unity_rx[pos + 1];
                                if (pos + flen > unity_rx.size()) break;
                                if (!send_all(px4_fd, &unity_rx[pos], flen)) {
                                    px4_ok = false;
                                    break;
                                }
                                pos += flen;
                            } else {
                                ++pos;
                            }
                        }
                        if (!px4_ok) break;
                        if (pos > 0) unity_rx.erase(unity_rx.begin(), unity_rx.begin() + pos);
                    }
                }
            }

            int64_t t_us = now_us();
            if (unity_stream_active) {
                if (t_us - last_unity_rx_us > 1500000) {
                    std::printf("[mavlink-proxy] Unity stream timed out; returning to standby sensors.\n");
                    std::fflush(stdout);
                    close_unity();
                }
                next_step_us = t_us + 4000;
            } else if (t_us >= next_step_us) {
                if (t_us - next_step_us > 40000) next_step_us = t_us - 4000;
                int steps = 0;
                bool px4_ok = true;
                while (t_us >= next_step_us && steps < 4) {
                    mono_time_usec += 4000ULL;
                    uint8_t pkt[128];
                    size_t len = build_standby_hil_sensor(standby_seq++, mono_time_usec, standby_tick, pkt);
                    if (!send_all(px4_fd, pkt, len)) {
                        px4_ok = false;
                        break;
                    }
                    if ((standby_tick % 25ULL) == 0ULL) {
                        len = build_standby_hil_gps(standby_seq++, mono_time_usec, pkt);
                        if (!send_all(px4_fd, pkt, len)) {
                            px4_ok = false;
                            break;
                        }
                    }
                    ++standby_tick;
                    next_step_us += 4000;
                    ++steps;

                    if (unity_fd >= 0 && !unity_stream_active) {
                        if (++pending_handshake_ticks > 62) close_unity();
                    } else if (unity_fd < 0 && (standby_tick % 62ULL) == 0ULL) {
                        unity_fd = try_start_unity_connect(unity_host, unity_port);
                        pending_handshake_ticks = 0;
                    }
                }
                if (!px4_ok) break;
            }
        }

        close_unity();
        ::close(px4_fd);
        std::printf("[mavlink-proxy] PX4 disconnected.\n");
        std::fflush(stdout);
    }

    ::close(srv);
    return 0;
}

