//
// MidiEndpointPublication.cpp
// ASFWDriver
//

#include "MidiEndpointPublication.hpp"

#include "../../Audio/Core/AudioEndpointRuntime.hpp"
#include "../../Audio/Core/AudioRuntimeRegistry.hpp"
#include "../../Audio/Protocols/AudioTypes.hpp"
#include "../../Logging/Logging.hpp"
#include "../Capabilities/MidiEndpointCapabilities.hpp"

namespace ASFW::Midi {

namespace {

constexpr const char* kManufacturerName = "FireWire";

// The published description carries the per-stream wire geometry the audio side
// was given (pcm channels, DBS, MIDI ports). The capability projection takes the
// shape discovery produced, so rebuild exactly that shape from it.
void FillStreams(const std::vector<ASFW::Audio::Model::ASFWAudioWireStream>& streams,
                 ASFW::Audio::AudioStreamWireInfo* out, uint32_t& count) noexcept {
    count = 0;
    for (const auto& stream : streams) {
        if (count >= ASFW::Audio::kMaxAudioStreamsPerDirection) break;
        auto& info = out[count++];
        info.pcmChannels = static_cast<uint16_t>(stream.pcmChannels);
        info.am824Slots = static_cast<uint16_t>(stream.am824Slots);
        info.midiPorts = static_cast<uint16_t>(stream.midiPorts);
    }
}

} // namespace

MidiEndpointCapabilities MidiEndpointPublication::Project(
    const uint64_t guid, const ASFW::Audio::Model::ASFWAudioDevice& description) noexcept {
    if (description.midiLayout.present) {
        // The family states its own MIDI layout instead of deriving it from the
        // stream formation. Check it against stream 0 of each direction.
        ASFW::Audio::AudioStreamWireInfo d2h{};
        ASFW::Audio::AudioStreamWireInfo h2d{};
        uint64_t d2hPcm = 0;
        uint64_t h2dPcm = 0;
        const auto fill = [](const std::vector<ASFW::Audio::Model::ASFWAudioWireStream>& streams,
                             ASFW::Audio::AudioStreamWireInfo& info, uint64_t& pcmSlots) {
            if (streams.empty()) return;
            const auto& stream = streams.front();
            info.pcmChannels = static_cast<uint16_t>(stream.pcmChannels);
            info.am824Slots = static_cast<uint16_t>(stream.am824Slots);
            for (uint32_t c = 0; c < stream.pcmChannels; ++c) {
                const uint32_t slot = stream.pcmSlotMap.SlotFor(c);
                if (slot < 64) pcmSlots |= uint64_t{1} << slot;
            }
        };
        fill(description.captureStreams, d2h, d2hPcm);
        fill(description.playbackStreams, h2d, h2dPcm);
        return ProjectDeclaredMidiCapabilities(description.midiLayout, d2h, h2d, d2hPcm, h2dPcm,
                                               guid);
    }
    ASFW::Audio::AudioStreamRuntimeCaps caps{};
    FillStreams(description.captureStreams, caps.deviceToHostStreams, caps.deviceToHostStreamCount);
    FillStreams(description.playbackStreams, caps.hostToDeviceStreams, caps.hostToDeviceStreamCount);
    // The stream epoch is a runtime value the endpoint owns; the projection's
    // copy of it is not used for publication.
    return ProjectMidiCapabilities(caps, guid, /*streamEpoch=*/0);
}

void MidiEndpointPublication::OnEndpointPublished(
    const uint64_t guid, const ASFW::Audio::AudioBackendKind kind,
    const ASFW::Audio::Model::ASFWAudioDevice& description) noexcept {
    // Family gate: DICE's MIDI geometry comes from its stream-format registers;
    // Digi00x declares its own layout. Other families' declared MIDI ports are
    // not trusted for publication yet.
    if (kind != ASFW::Audio::AudioBackendKind::Dice &&
        kind != ASFW::Audio::AudioBackendKind::Digi00xRegister) {
        return;
    }

    const MidiEndpointCapabilities caps = Project(guid, description);
    if (!caps.AnyUsable()) {
        // A device that reports no MIDI is the ordinary case and says nothing.
        // A device that reports some but whose geometry is rejected is not: say
        // why, so a device that should have MIDI and does not is visible
        // without debug logging.
        const bool reportedNone = caps.deviceToHost.status == MidiCapabilityStatus::kNoMidi &&
                                  caps.hostToDevice.status == MidiCapabilityStatus::kNoMidi;
        if (!reportedNone) {
            ASFW_LOG(Midi,
                     "[MidiPublish] guid=%016llx not published (deviceToHost=%{public}s "
                     "hostToDevice=%{public}s)",
                     guid, MidiCapabilityStatusName(caps.deviceToHost.status),
                     MidiCapabilityStatusName(caps.hostToDevice.status));
        }
        return;
    }

    // The endpoint owns the byte seam and the geometry; set both before the nub
    // exists so the MIDI service never starts against an unconfigured endpoint.
    const auto endpoint = runtime_.EnsureEndpointRuntime(guid);
    if (!endpoint) {
        ASFW_LOG_ERROR(Midi, "[MidiPublish] guid=%016llx no endpoint runtime", guid);
        return;
    }
    const kern_return_t kr = endpoint->ConfigureMidi(caps);
    if (kr != kIOReturnSuccess) {
        ASFW_LOG_ERROR(Midi, "[MidiPublish] guid=%016llx ConfigureMidi failed kr=0x%x", guid, kr);
        return;
    }
    endpoint->SetMidiReceiveWake(&wake_);

    const char* name = description.deviceName.c_str();
    if (!nubs_.EnsureNub(caps, guid, name, name, kManufacturerName)) {
        ASFW_LOG_ERROR(Midi, "[MidiPublish] guid=%016llx nub publication failed", guid);
    }
}

void MidiEndpointPublication::OnEndpointRetired(const uint64_t guid) noexcept {
    nubs_.TerminateNub(guid);
}

} // namespace ASFW::Midi
