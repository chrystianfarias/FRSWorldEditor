#pragma once

#include "core/Math.h"
#include "core/Types.h"

#include <algorithm>
#include <cstring>
#include <string>

namespace nfs {

// Small readers shared by the UG2 and NFS World binary layouts. Keeping them
// here avoids a private copy in every parser.
inline Vec3 ReadVec3(ByteSpan span, size_t offset) {
    return { span.read<float>(offset),
             span.read<float>(offset + 4),
             span.read<float>(offset + 8) };
}

inline Mat4 ReadMat4(ByteSpan span, size_t offset) {
    Mat4 m;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            m.m[row][col] = span.read<float>(offset + (size_t)(row * 4 + col) * 4);
    return m;
}

inline std::string ReadName(ByteSpan span, size_t offset, size_t capacity) {
    if (offset >= span.size) return {};
    const size_t available = std::min(capacity, span.size - offset);
    const char* text = (const char*)span.data + offset;
    return std::string(text, ::strnlen(text, available));
}

inline std::string ReadName(ByteSpan span, size_t offset) {
    return ReadName(span, offset, span.size - offset);
}

} // namespace nfs
