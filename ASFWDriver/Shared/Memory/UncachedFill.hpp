// UncachedFill.hpp
// ASFW - filling and reading cache-inhibited DMA mappings.
//
// DMA slabs are mapped kIOMemoryMapCacheModeInhibit, so CPU stores reach RAM
// without a flush. That memory does not accept cache-maintenance instructions,
// and `dc zva` is one. Apple's __bzero picks `dc zva` by block size, so
// memset() over such a mapping works below a libc-internal size and faults
// above it -- reported as EXC_ARM_DA_ALIGN at a correctly aligned address.
// Found on the midi branch (51a150d6, 2026-09-07) when the IT descriptor slab
// grew from 4 KiB to 32 KiB. Size is not the safety argument; the store
// instruction is.
//
// Use this for any fill wider than one descriptor. A memset over a single
// 16-byte OHCIDescriptor is plain stores at any size libc supports.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ASFW::Shared {

/// Byte-fill a cache-inhibited DMA region with plain volatile stores.
inline void FillUncachedDma(void* base, uint8_t value, size_t length) noexcept {
    if (base == nullptr || length == 0) {
        return;
    }
    auto* bytes = static_cast<volatile uint8_t*>(base);
    for (size_t i = 0; i < length; ++i) {
        bytes[i] = value;
    }
}

/// Copy a region of a cache-inhibited DMA mapping into ordinary memory.
///
/// Every CPU load from such a mapping is its own uncached memory access, so
/// code that parses a DMA buffer in place pays that cost on every field it
/// reads, and again on every re-read. Copying once in wide aligned loads and
/// parsing the copy is several times cheaper: an AM824 receive packet went
/// from about 24 us of decode in place to 3.6 us of copy plus 1.3 us of decode.
///
/// Volatile loads, for the reason FillUncachedDma uses volatile stores:
/// memcpy() picks its own instructions, and this memory faults on some of
/// them. Only `length` bytes are read; nothing past the end of the region.
inline void CopyFromUncachedDma(void* destination, const void* source, size_t length) noexcept {
    if (destination == nullptr || source == nullptr || length == 0) {
        return;
    }
    auto* out = static_cast<uint8_t*>(destination);
    const auto* in = static_cast<const volatile uint8_t*>(source);
    size_t i = 0;
    if ((reinterpret_cast<uintptr_t>(source) & 0xFu) == 0) {
        for (; i + 16 <= length; i += 16) {
            const __uint128_t chunk = *reinterpret_cast<const volatile __uint128_t*>(in + i);
            std::memcpy(out + i, &chunk, sizeof(chunk));
        }
    }
    if ((reinterpret_cast<uintptr_t>(in + i) & 0x3u) == 0) {
        for (; i + 4 <= length; i += 4) {
            const uint32_t quadlet = *reinterpret_cast<const volatile uint32_t*>(in + i);
            std::memcpy(out + i, &quadlet, sizeof(quadlet));
        }
    }
    for (; i < length; ++i) {
        out[i] = in[i];
    }
}

} // namespace ASFW::Shared
