//
// MidiWireLayout.hpp
// ASFWDriver
//
// A family's own statement of its MIDI layout, for families whose MIDI is not
// derived from the stream formation (DICE derives it: one trailing MPX slot).
// Read only by MIDI publication; the audio path never sees it, so declaring
// MIDI here cannot change a stream's audio geometry.
//
// Also the CoreMIDI entity layout: which receive port feeds each entity's
// source and which transmit port its destination writes. CoreMIDI entities
// may have any number of sources and destinations, including none
// (IOUserMIDIEntity::Create), so ports need not come in matched pairs.
//
// Pure data: no DriverKit dependency.
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "../../Audio/Wire/AM824/MpxMidiDemux.hpp"   // MidiWireFormat

namespace ASFW::Midi {

inline constexpr uint8_t kNoMidiPort = 0xFF;
inline constexpr size_t kMaxMidiEntities = 8;
inline constexpr size_t kMidiEntityNameBytes = 24;

/// One CoreMIDI entity: a name, and the wire port behind each endpoint.
struct MidiEntityLayout final {
    std::array<char, kMidiEntityNameBytes> name{};
    /// Device -> host port feeding the entity's source, or kNoMidiPort.
    uint8_t sourcePort{kNoMidiPort};
    /// Host -> device port the entity's destination writes, or kNoMidiPort.
    uint8_t destinationPort{kNoMidiPort};

    friend constexpr bool operator==(const MidiEntityLayout&,
                                     const MidiEntityLayout&) noexcept = default;
};

/// Entity table. Empty means the positional default: entity n uses port n in
/// both directions, as many entities as the larger port count.
struct MidiEntityTable final {
    std::array<MidiEntityLayout, kMaxMidiEntities> entities{};
    uint8_t count{0};

    [[nodiscard]] bool Add(const char* name, uint8_t sourcePort,
                           uint8_t destinationPort) noexcept {
        if (count >= kMaxMidiEntities) return false;
        auto& entity = entities[count++];
        size_t i = 0;
        for (; name != nullptr && name[i] != '\0' && i + 1 < entity.name.size(); ++i) {
            entity.name[i] = name[i];
        }
        entity.name[i] = '\0';
        entity.sourcePort = sourcePort;
        entity.destinationPort = destinationPort;
        return true;
    }

    friend constexpr bool operator==(const MidiEntityTable&,
                                     const MidiEntityTable&) noexcept = default;
};

/// A family-declared MIDI layout for stream 0.
struct MidiWireLayout final {
    bool present{false};
    ::ASFW::Encoding::MidiWireFormat format{::ASFW::Encoding::MidiWireFormat::kAm824Mpx};
    /// AM824 slot of the MIDI channel within each data block.
    uint8_t slotIndex{0};
    /// Port index bounds per direction (ports 0..n-1 may carry bytes).
    uint8_t sourcePorts{0};
    uint8_t destinationPorts{0};
    MidiEntityTable entities{};

    friend constexpr bool operator==(const MidiWireLayout&,
                                     const MidiWireLayout&) noexcept = default;
};

} // namespace ASFW::Midi
