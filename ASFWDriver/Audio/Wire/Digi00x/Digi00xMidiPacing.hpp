// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 ASFireWire Project
//
// Digi00xMidiPacing.hpp - the least time between two MIDI bytes for one
// Digi 002 / 003 port (issue #249).
//
// Fed MIDI faster than they send it, the Digi 002 and 003 occasionally put one
// byte out of their MIDI Out jack a place late, after the byte that followed
// it. The AM824 UART model the composer already applies does not prevent it.
// Loopback on a 002 Rack, Out 1 to In 1, 66 runs of 600 or 1536 bytes per
// sweep, counting runs with a late byte:
//
//   UART model only (two cadences)       48 kHz 7 of 132
//   bytes for one port >= 375 us apart   48 kHz 0 of 132, 44.1 kHz 2 of 66
//   bytes for one port >= 500 us apart   48 kHz 0 of 66,  44.1 kHz 0 of 66
//
// The vendor driver on the same device and loop sends a 600-byte burst at
// about 525 us per byte, and shows no late bytes at 48 kHz (0 of 66). So two
// bytes for one port are kept at least kMinMidiByteIntervalUs apart.
//
// Time is the stream's sample time: the data blocks between the two bytes'
// blocks, one block per frame. That means the same at every rate and needs no
// clock. It is counted over the packets the composer examines; packets the
// finality frontier overtakes before a pass reaches them are not counted, which
// only ever makes the interval longer.
//
// Pure logic: no DriverKit dependency, no allocation, no locking.
#pragma once

#include "Digi00xMidi.hpp"

#include <cstdint>

namespace ASFW::Encoding::Digi00x {

/// The least time between two MIDI bytes for one port.
inline constexpr uint32_t kMinMidiByteIntervalUs = 500;

class MidiBytePacer final {
public:
    static constexpr uint32_t kPorts = kMpxMidiPorts;

    /// Space bytes at least `minIntervalUs` apart at `sampleRateHz`. Either
    /// one zero switches pacing off: every port is always allowed.
    void Configure(uint32_t sampleRateHz, uint32_t minIntervalUs) noexcept {
        const uint64_t product = uint64_t{sampleRateHz} * minIntervalUs;
        minBlocks_ = static_cast<uint32_t>((product + 999'999u) / 1'000'000u);
        Reset();
    }

    /// Forget every byte sent: the next byte for each port may go at once.
    void Reset() noexcept {
        clock_ = 0;
        for (auto& next : next_) next = 0;
    }

    /// Blocks that must separate two bytes for one port (0 when off).
    [[nodiscard]] uint32_t MinBlocks() const noexcept { return minBlocks_; }

    /// The ports (bit n = port n) whose block in the packet that starts now,
    /// with this DBC and block count, is far enough from their last byte.
    /// Bits above kPorts are left set: the pacer does not own them.
    [[nodiscard]] uint32_t AllowedPorts(uint8_t dbc, uint32_t blocks) const noexcept {
        if (minBlocks_ == 0) return ~0u;  // off: the Digi rotation does not apply
        uint32_t mask = ~((1u << kPorts) - 1u);
        for (uint32_t port = 0; port < kPorts; ++port) {
            uint32_t block = 0;
            if (BlockForPort(dbc, blocks, port, block) && clock_ + block >= next_[port]) {
                mask |= 1u << port;
            }
        }
        return mask;
    }

    /// A byte for `port` went into the packet that starts now.
    void Charge(uint8_t dbc, uint32_t blocks, uint32_t port) noexcept {
        uint32_t block = 0;
        if (port >= kPorts || !BlockForPort(dbc, blocks, port, block)) return;
        next_[port] = clock_ + block + minBlocks_;
    }

    /// The packet that started now carried `blocks` data blocks.
    void Advance(uint32_t blocks) noexcept { clock_ += blocks; }

private:
    /// The block WriteMidiSlot uses for `port`, if the packet has one.
    [[nodiscard]] static bool BlockForPort(uint8_t dbc, uint32_t blocks, uint32_t port,
                                           uint32_t& outBlock) noexcept {
        for (uint32_t f = 0; f < blocks && f < 8u; ++f) {
            if (PortForBlock(dbc, f) == port) {
                outBlock = f;
                return true;
            }
        }
        return false;
    }

    uint64_t clock_{0};
    uint64_t next_[kPorts]{};
    uint32_t minBlocks_{0};
};

} // namespace ASFW::Encoding::Digi00x
