#include "can/can_map.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <linux/can.h>

namespace can {

// ── internal parse helpers ────────────────────────────────────────────────────

namespace {

const char* ltrim(const char* s) {
    while (*s == ' ' || *s == '\t') ++s;
    return s;
}

// Extract the first "quoted" substring found in [s]; returns false if none.
bool extract_quoted(const char* s, std::string& out) {
    const char* q1 = strchr(s, '"');
    if (!q1) return false;
    const char* q2 = strchr(q1 + 1, '"');
    if (!q2) return false;
    out.assign(q1 + 1, static_cast<size_t>(q2 - q1 - 1));
    return true;
}

struct TempSig {
    std::string name;
    int  start_bit  = 0;
    int  bit_length = 0;
    char byte_order = '1'; // '1' = Intel/little-endian, '0' = Motorola/big-endian
    char value_type = '+'; // '+' = unsigned, '-' = signed
    double factor = 1.0, offset = 0.0, min_val = 0.0, max_val = 0.0;
    std::string unit;
    // Filled from BA_ attributes:
    std::string target;
    double default_value = 0.0;
};

struct TempFrame {
    uint32_t    id  = 0;
    std::string name;
    int         dlc = 8;
    // J1939 extended-frame fields (decoded from 29-bit ID):
    bool        is_extended = false;
    uint32_t    pgn         = 0;
    uint8_t     sa          = 0;
    uint8_t     j1939_prio  = 6;
    // Filled from BA_ attributes:
    std::string direction = "tx";
    int         cycle_ms  = 0;
    std::vector<TempSig> signals;
};

} // namespace

// ── CanMap::load ──────────────────────────────────────────────────────────────
//
// Parses a DBC file produced by tools/csv_to_dbc.py.
//
// Two logical sections are processed:
//   1. BO_ / SG_ blocks  — frame and signal definitions
//   2. BA_ lines         — custom attributes (Direction, GenMsgCycleTime,
//                          Target, DefaultValue)
//
// All other DBC keywords (NS_, BS_, BU_, CM_, BA_DEF_, VAL_, …) are ignored.

bool CanMap::load(const std::string& dbc_path) {
    std::ifstream file(dbc_path);
    if (!file.is_open()) return false;

    rx_frames_.clear();
    tx_frames_.clear();

    std::unordered_map<uint32_t, TempFrame> frames;
    uint32_t current_bo_id = 0;

    std::string line;
    while (std::getline(file, line)) {
        // Strip trailing CR (Windows line endings)
        if (!line.empty() && line.back() == '\r') line.pop_back();

        const bool indented = !line.empty() && (line[0] == ' ' || line[0] == '\t');
        const char* s = ltrim(line.c_str());

        if (*s == '\0' || *s == '/') {
            // Blank line or comment ends the current BO_ block
            if (!indented) current_bo_id = 0;
            continue;
        }

        // ── BO_ <id> <name>: <dlc> <sender> ──────────────────────────────
        if (strncmp(s, "BO_ ", 4) == 0) {
            TempFrame f;
            char name_buf[128] = {};
            if (sscanf(s, "BO_ %u %127[^:]: %d", &f.id, name_buf, &f.dlc) == 3) {
                f.name = name_buf;

                // ── J1939 / EFF detection ─────────────────────────────────
                // DBC uses Vector EFF convention: bit 31 set → 29-bit extended.
                if (f.id & 0x80000000u) {
                    uint32_t raw29 = f.id & 0x1FFFFFFFu;
                    // Store SocketCAN-ready ID (CAN_EFF_FLAG preserved)
                    f.id           = CAN_EFF_FLAG | raw29;
                    f.is_extended  = true;
                    f.sa           = static_cast<uint8_t>(raw29 & 0xFF);
                    f.j1939_prio   = static_cast<uint8_t>((raw29 >> 26) & 0x07);
                    uint8_t pf     = static_cast<uint8_t>((raw29 >> 16) & 0xFF);
                    uint8_t dp     = static_cast<uint8_t>((raw29 >> 24) & 0x01);
                    uint8_t ps     = static_cast<uint8_t>((raw29 >>  8) & 0xFF);
                    // PDU2 (PF >= 0xF0): GE is part of PGN; PDU1: DA not in PGN
                    f.pgn = (pf >= 0xF0u)
                          ? ((uint32_t)dp << 17) | ((uint32_t)pf << 8) | ps
                          : ((uint32_t)dp << 17) | ((uint32_t)pf << 8);
                }

                current_bo_id = f.id;
                frames.emplace(f.id, std::move(f));
            }
            continue;
        }

        // ── SG_ <name> : <start>|<len>@<bo><vt> (<f>,<o>) [<mn>|<mx>] "<unit>" ──
        if (strncmp(s, "SG_ ", 4) == 0 && current_bo_id != 0) {
            TempSig sig;
            char sig_name[128] = {}, unit[64] = {};
            int n = sscanf(s,
                "SG_ %127s : %d|%d@%c%c (%lf,%lf) [%lf|%lf] \"%63[^\"]\"",
                sig_name,
                &sig.start_bit, &sig.bit_length,
                &sig.byte_order, &sig.value_type,
                &sig.factor, &sig.offset,
                &sig.min_val, &sig.max_val,
                unit);
            if (n == 10) {
                sig.name = sig_name;
                sig.unit = unit;
                frames[current_bo_id].signals.push_back(std::move(sig));
            }
            continue;
        }

        // ── BA_ "AttrName" BO_ <id> <value> ; ────────────────────────────
        // ── BA_ "AttrName" SG_ <id> <signame> <value> ; ──────────────────
        if (strncmp(s, "BA_ ", 4) == 0) {
            current_bo_id = 0; // BA_ is never inside a BO_ block

            // Extract attribute name (first quoted token)
            const char* q1 = strchr(s + 4, '"');
            if (!q1) continue;
            const char* q2 = strchr(q1 + 1, '"');
            if (!q2) continue;
            const std::string attr(q1 + 1, static_cast<size_t>(q2 - q1 - 1));
            const char* rest = ltrim(q2 + 1);

            if (strncmp(rest, "BO_ ", 4) == 0) {
                // ── frame attribute ───────────────────────────────────────
                uint32_t fid = 0;
                if (sscanf(rest + 4, "%u", &fid) != 1) continue;
                auto it = frames.find(fid);
                if (it == frames.end()) continue;

                // Advance past the frame id to the value token
                const char* vp = ltrim(rest + 4);
                while (*vp && !isspace(static_cast<unsigned char>(*vp))) ++vp;
                vp = ltrim(vp);

                if (attr == "Direction") {
                    std::string val;
                    if (extract_quoted(vp, val)) it->second.direction = val;
                } else if (attr == "GenMsgCycleTime") {
                    int cycle = 0;
                    if (sscanf(vp, "%d", &cycle) == 1) it->second.cycle_ms = cycle;
                }

            } else if (strncmp(rest, "SG_ ", 4) == 0) {
                // ── signal attribute ──────────────────────────────────────
                uint32_t fid = 0;
                char sig_name[128] = {};
                if (sscanf(rest + 4, "%u %127s", &fid, sig_name) != 2) continue;
                auto it = frames.find(fid);
                if (it == frames.end()) continue;

                // Advance past "SG_ <id> <signame>" to the value token
                const char* vp = ltrim(rest + 4);           // skip "SG_ "
                while (*vp && !isspace(static_cast<unsigned char>(*vp))) ++vp; // skip id
                vp = ltrim(vp);
                while (*vp && !isspace(static_cast<unsigned char>(*vp))) ++vp; // skip signame
                vp = ltrim(vp);

                for (auto& sig : it->second.signals) {
                    if (sig.name != sig_name) continue;
                    if (attr == "Target") {
                        std::string val;
                        if (extract_quoted(vp, val)) sig.target = val;
                    } else if (attr == "DefaultValue") {
                        double d = 0.0;
                        if (sscanf(vp, "%lf", &d) == 1) sig.default_value = d;
                    }
                    break;
                }
            }
        }
    }

    // ── build CanMap from collected TempFrame data ────────────────────────────
    for (auto& [id, f] : frames) {
        FrameDef fd;
        fd.frame_id    = f.id;
        fd.frame_name  = f.name;
        fd.dlc         = f.dlc;
        fd.cycle_ms    = f.cycle_ms;
        fd.is_extended = f.is_extended;
        fd.pgn         = f.pgn;
        fd.sa          = f.sa;
        fd.priority    = f.j1939_prio;

        for (auto& ts : f.signals) {
            SignalRule sr;
            sr.signal_name   = ts.name;
            sr.target        = ts.target;
            sr.start_bit     = ts.start_bit;
            sr.bit_length    = ts.bit_length;
            sr.endianness    = (ts.byte_order == '1') ? utils::Endianness::Little
                                                      : utils::Endianness::Big;
            sr.is_signed     = (ts.value_type == '-');
            sr.factor        = ts.factor;
            sr.offset        = ts.offset;
            sr.min           = ts.min_val;
            sr.max           = ts.max_val;
            sr.default_value = ts.default_value;
            sr.unit          = ts.unit;
            fd.signals.push_back(std::move(sr));
        }

        if (f.direction == "rx") {
            rx_frames_[id] = std::move(fd);
        } else {
            tx_frames_.push_back(std::move(fd));
        }
    }

    // Preserve cycle-time ordering for the TX scheduler
    std::sort(tx_frames_.begin(), tx_frames_.end(),
              [](const FrameDef& a, const FrameDef& b) {
                  return a.cycle_ms < b.cycle_ms;
              });

    return !frames.empty();
}

// ── lookup helpers ────────────────────────────────────────────────────────────

const FrameDef* CanMap::find_rx_frame(uint32_t frame_id) const {
    auto it = rx_frames_.find(frame_id);
    if (it == rx_frames_.end()) return nullptr;
    return &it->second;
}

const FrameDef* CanMap::find_tx_frame(uint32_t frame_id) const {
    for (const auto& f : tx_frames_) {
        if (f.frame_id == frame_id) return &f;
    }
    return nullptr;
}

} // namespace can
