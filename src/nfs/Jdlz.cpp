#include "nfs/Jdlz.h"

#include <cstring>

namespace nfs {

namespace {

constexpr size_t kHeader = 16;

u32 Read32(const u8* at) {
    u32 value = 0;
    std::memcpy(&value, at, 4);
    return value;
}

void Write32(u8* at, u32 value) { std::memcpy(at, &value, 4); }

} // namespace

std::vector<u8> JdlzDecompress(ByteSpan data) {
    if (data.size < kHeader || std::memcmp(data.data, "JDLZ", 4) != 0) return {};
    const size_t size = Read32(data.data + 8);
    const size_t packed = Read32(data.data + 12);
    if (packed > data.size || packed < kHeader) return {};

    std::vector<u8> out(size);
    const u8* in = data.data;
    size_t pos = kHeader, o = 0;
    unsigned flags1 = 1, flags2 = 1;
    while (pos < packed && o < size) {
        if (flags1 == 1) {
            if (pos >= packed) break;
            flags1 = in[pos++] | 0x100u;
        }
        if (flags2 == 1) {
            if (pos >= packed) break;
            flags2 = in[pos++] | 0x100u;
        }
        if (flags1 & 1) {
            if (pos + 2 > packed) break;
            const unsigned b0 = in[pos], b1 = in[pos + 1];
            pos += 2;
            size_t length, offset;
            if (flags2 & 1) {
                length = (b1 | ((b0 & 0xF0u) << 4)) + 3;
                offset = (b0 & 0x0Fu) + 1;
            } else {
                offset = (b1 | ((b0 & 0xE0u) << 3)) + 17;
                length = (b0 & 0x1Fu) + 3;
            }
            if (offset > o) return {};
            for (size_t k = 0; k < length && o < size; ++k, ++o) out[o] = out[o - offset];
            flags2 >>= 1;
        } else {
            out[o++] = in[pos++];
        }
        flags1 >>= 1;
    }
    if (o != size) return {};
    return out;
}

// The game's own JLZCompress, as ported in Underground2\Cops\tools\car\eacomp.py:
// a 2064-entry window of positions chained by a 13-bit hash of the next three
// bytes. Like the original it reads up to two bytes past the end and ends with
// one extra literal (the loop runs while the bytes left are >= 0); those bytes
// are zeros here.
std::vector<u8> JdlzCompress(ByteSpan data) {
    constexpr int kWindow = 2064, kNearRunMax = 4098, kFarRunMax = 34;
    std::vector<u8> src(data.data, data.data + data.size);
    src.resize(data.size + 4, 0);
    const int size = (int)data.size;

    std::vector<int> poolOffset(kWindow, 0), poolPrev(kWindow, -1), poolNext(kWindow, -1);
    std::vector<bool> poolValid(kWindow, false);
    std::vector<int> head(8192, -1);
    const auto hashAt = [&](int p) {
        return (int)((((u32)src[p + 2] << 8 ^ (u32)src[p + 1] << 4 ^ (u32)src[p]) * 0x9E5Fu) & 0x1FFFu);
    };
    const auto update = [&](int p) {
        const int w = p % kWindow;
        if (poolValid[w]) {
            const int next = poolNext[w], prev = poolPrev[w];
            if (next != -1) poolPrev[next] = prev;
            if (prev != -1) poolNext[prev] = next;
            const int t = hashAt(poolOffset[w]);
            if (head[t] == w) head[t] = next;
        }
        poolOffset[w] = p;
        poolValid[w] = true;
        const int t = hashAt(p);
        poolPrev[w] = -1;
        poolNext[w] = head[t];
        head[t] = w;
        if (poolNext[w] != -1) poolPrev[poolNext[w]] = w;
    };

    std::vector<u8> out(kHeader);
    std::memcpy(out.data(), "JDLZ", 4);
    out[4] = 0x02;
    out[5] = 0x10;
    Write32(out.data() + 8, (u32)size);
    u32 control = 0xFF00, runType = 0xFF00;
    size_t controlAt = out.size();
    out.push_back(0);
    size_t runTypeAt = out.size();
    out.push_back(0);
    int pos = 0, remaining = size;
    while (remaining >= 0) {
        const int maxLength = remaining >= kNearRunMax ? kNearRunMax : remaining;
        int longest = 2, best = -1;
        for (int h = head[hashAt(pos)]; h != -1; h = poolNext[h]) {
            const int q = poolOffset[h];
            int run = 0;
            while (run < maxLength && src[q + run] == src[pos + run]) ++run;
            if (run > longest) {
                const int back = pos - q;
                if (run <= kFarRunMax || back < 0x10 || longest <= kFarRunMax) {
                    longest = run;
                    best = q;
                }
            }
            if (longest >= kNearRunMax) break;
        }
        if (longest > 2) {
            control >>= 1;
            const int back = pos - best - 1;
            if (back > 15) {
                if (longest > kFarRunMax) longest = kFarRunMax;
                runType = (runType >> 1) & 0x7F7F;
                out.push_back((u8)((((back - 0x10) >> 8) << 5) | (longest - 3)));
                out.push_back((u8)(back - 0x10));
            } else {
                runType >>= 1;
                out.push_back((u8)((((longest - 3) >> 8) << 4) | back));
                out.push_back((u8)(longest - 3));
            }
            remaining -= longest;
            for (int i = 0; i < longest; ++i) update(pos++);
        } else {
            update(pos);
            control = (control >> 1) & 0x7F7F;
            --remaining;
            out.push_back(src[pos]);
            ++pos;
        }
        if (control < 256) {
            out[controlAt] = (u8)control;
            controlAt = out.size();
            control = 0xFF00;
            out.push_back(0);
        }
        if (runType < 256) {
            out[runTypeAt] = (u8)runType;
            runTypeAt = out.size();
            runType = 0xFF00;
            out.push_back(0);
        }
    }
    while (control & 0xFF00) control >>= 1;
    out[controlAt] = (u8)control;
    while (runType & 0xFF00) runType >>= 1;
    out[runTypeAt] = (u8)runType;
    Write32(out.data() + 12, (u32)out.size());
    return out;
}

std::vector<u8> JdlzStore(ByteSpan data) {
    // The decoder reads the first flag byte of each stream up front, then the
    // literal stream's again every eight steps; the reference stream's never
    // moves, so it is read once.
    std::vector<u8> out(kHeader);
    std::memcpy(out.data(), "JDLZ", 4);
    out[4] = 0x02;
    out[5] = 0x10;
    Write32(out.data() + 8, (u32)data.size);
    for (size_t at = 0; at < data.size; at += 8) {
        out.push_back(0);                   // eight literals
        if (at == 0) out.push_back(0);      // the reference stream's only byte
        const size_t run = data.size - at < 8 ? data.size - at : 8;
        out.insert(out.end(), data.data + at, data.data + at + run);
    }
    Write32(out.data() + 12, (u32)out.size());
    return out;
}

} // namespace nfs
