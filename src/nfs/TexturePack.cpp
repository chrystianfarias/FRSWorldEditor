#include "nfs/TexturePack.h"

#include "core/Log.h"
#include "nfs/EaCompress.h"

#include <algorithm>
#include <cstring>
#include <functional>

namespace nfs {

namespace {

// --- Texture header, chunk 0x33310004 --------------------------------------
constexpr size_t kTextureEntrySize = 124;
constexpr size_t kTexName          = 0x0C;  // 20 bytes, zero terminated
constexpr size_t kTexNameCapacity  = 20;
constexpr size_t kTexHash          = 0x24;
constexpr size_t kTexDataOffset    = 0x30;  // byte offset into the pixel block
constexpr size_t kTexPaletteOffset = 0x34;  // zero unless the texture is palettised
constexpr size_t kTexDataSize      = 0x38;  // bytes covering every mip level
constexpr size_t kTexPaletteSize   = 0x3C;  // 1024 for a 256-entry palette
constexpr size_t kTexBaseLevelSize = 0x40;
constexpr size_t kTexWidth         = 0x44;
constexpr size_t kTexHeight        = 0x46;
constexpr size_t kTexMipCount      = 0x4E;
constexpr size_t kTexScrollType    = 0x52;
constexpr size_t kTexScrollStep    = 0x58;  // i16, 8.8 fixed point, as are the two after it
constexpr size_t kTexScrollS       = 0x5A;
constexpr size_t kTexScrollT       = 0x5C;
// Across the 1 821 textures of the eight track streams +0x55 only ever holds
// 0..2 and +0x56 0..2, and they line up with what the game draws: every neon
// sign and flare is 2/2, the barrier tape and smoke 2/1, trees and chain-link
// fences 1/0, and the road textures -- whose alpha is gloss -- 2/0 or 0/0.
constexpr size_t kTexAlphaUsage    = 0x55;
constexpr size_t kTexAlphaBlend    = 0x56;

// --- Format entry, chunk 0x33310005 ----------------------------------------
constexpr size_t kFormatEntrySize = 32;
constexpr size_t kFormatFourCc    = 0x14;
constexpr size_t kNfswFormatFourCc = 0x0C;

// --- NFS World texture header, chunk 0x33310004 ----------------------------
// 144 bytes plus a one-byte name length and the name. The alpha fields moved
// from UG2's +0x55/+0x56 to +0x38/+0x39; +0x18 is the size of every mip level,
// +0x1C the size of mip 0.
constexpr size_t kNfswTextureEntrySize = 144;
constexpr size_t kNfswTexHash          = 0x0C;
constexpr size_t kNfswTexImageSize     = 0x18;
constexpr size_t kNfswTexBaseSize      = 0x1C;
constexpr size_t kNfswTexWidth         = 0x20;
constexpr size_t kNfswTexHeight        = 0x24;
constexpr size_t kNfswTexAlphaUsage    = 0x38;
constexpr size_t kNfswTexAlphaBlend    = 0x39;
constexpr size_t kNfswTexScrollType    = 0x35;
constexpr size_t kNfswTexScrollStep    = 0x3C;
constexpr size_t kNfswTexScrollS       = 0x3E;
constexpr size_t kNfswTexScrollT       = 0x40;
constexpr size_t kNfswTexDataOffset    = 0x50;

// --- NFS World animation, chunks 0xB3312000 / 0xB3312004 -------------------
// Part 1 holds the name, key hash, frame count and fps; part 2 is a list of
// 12-byte entries, the first four bytes of each being the frame texture hash.
constexpr u32 kChunkTpkAnimationContainer = 0xB3312000;
constexpr u32 kChunkTpkAnimation          = 0xB3312004;
constexpr u32 kNfswAnimPart1              = 0x33312001;
constexpr u32 kNfswAnimPart2              = 0x33312002;
constexpr size_t kNfswAnimName            = 0x10;
constexpr size_t kNfswAnimKey             = 0x20;
constexpr size_t kNfswAnimCount           = 0x24;
constexpr size_t kNfswAnimFps             = 0x25;
constexpr size_t kNfswAnimPart2Offset     = 0x34;
constexpr size_t kNfswAnimEntrySize       = 12;

u32 LevelBytes(TextureFormat format, u32 width, u32 height) {
    const u32 blocksX = std::max(1u, (width + 3) / 4);
    const u32 blocksY = std::max(1u, (height + 3) / 4);
    switch (format) {
        case TextureFormat::Bc4:
        case TextureFormat::Dxt1:
            return blocksX * blocksY * 8;
        case TextureFormat::Bc5:
        case TextureFormat::Dxt3:
        case TextureFormat::Dxt5:
            return blocksX * blocksY * 16;
        default:
            return width * height * 4;
    }
}

u32 NfswMipCount(u32 width, u32 height, TextureFormat format, u32 total) {
    u32 count = 0;
    u32 sum = 0;
    u32 w = width, h = height;
    for (;;) {
        const u32 level = LevelBytes(format, w, h);
        if (sum + level > total) break;
        sum += level;
        ++count;
        if (w == 1 && h == 1) break;
        w = std::max(1u, w >> 1);
        h = std::max(1u, h >> 1);
    }
    return std::max(1u, count);
}

TextureFormat FormatFromFourCc(u32 fourCc) {
    switch (fourCc) {
        case 0x31545844: return TextureFormat::Dxt1;  // "DXT1"
        case 0x33545844: return TextureFormat::Dxt3;  // "DXT3"
        case 0x35545844: return TextureFormat::Dxt5;  // "DXT5"
        case 0x31495441: return TextureFormat::Bc4;   // "ATI1"
        case 0x32495441: return TextureFormat::Bc5;   // "ATI2"
        case 41:         return TextureFormat::Palettised8;  // D3DFMT_P8
        case 21:         return TextureFormat::Argb8888;     // D3DFMT_A8R8G8B8
        default:         return TextureFormat::Unknown;
    }
}

std::string ReadFixedName(ByteSpan span, size_t offset, size_t capacity) {
    if (offset >= span.size) return {};
    const size_t available = std::min(capacity, span.size - offset);
    const char* text = (const char*)span.data + offset;
    return std::string(text, ::strnlen(text, available));
}

bool ParseNfswAnimation(ByteSpan data, TextureFrameAnimation& out) {
    if (data.size < kNfswAnimPart2Offset + 8) return false;
    if (data.read<u32>(0) != kNfswAnimPart1) return false;
    const u32 part1Size = data.read<u32>(4);
    if (part1Size < 0x2C || (size_t)8 + part1Size + 8 > data.size) return false;
    const size_t part2 = 8 + part1Size;
    if (data.read<u32>(part2) != kNfswAnimPart2) return false;

    out.name    = ReadFixedName(data, kNfswAnimName, 16);
    out.keyHash = data.read<u32>(kNfswAnimKey);
    out.fps     = (float)data.read<u8>(kNfswAnimFps);
    const u32 count = data.read<u8>(kNfswAnimCount);
    const u32 part2Size = data.read<u32>(part2 + 4);
    const size_t available = (data.size - part2 - 8) / kNfswAnimEntrySize;
    const size_t frames = std::min<size_t>({ count, part2Size / kNfswAnimEntrySize, available });
    out.frames.clear();
    out.frames.reserve(frames);
    for (size_t i = 0; i < frames; ++i)
        out.frames.push_back(data.read<u32>(part2 + 8 + i * kNfswAnimEntrySize));
    return out.frames.size() > 1 && out.fps > 0.0f;
}

} // namespace

const char* ToString(TextureFormat format) {
    switch (format) {
        case TextureFormat::Dxt1: return "DXT1";
        case TextureFormat::Dxt3: return "DXT3";
        case TextureFormat::Dxt5: return "DXT5";
        case TextureFormat::Palettised8: return "P8";
        case TextureFormat::Argb8888: return "A8R8G8B8";
        case TextureFormat::Bc4: return "ATI1/BC4";
        case TextureFormat::Bc5: return "ATI2/BC5";
        default:                  return "unknown";
    }
}

static Texture ReadHeader(ByteSpan entry, ByteSpan format) {
    Texture texture;
    texture.name          = ReadFixedName(entry, kTexName, kTexNameCapacity);
    texture.hash          = entry.read<u32>(kTexHash);
    texture.width         = entry.read<u16>(kTexWidth);
    texture.height        = entry.read<u16>(kTexHeight);
    texture.mipCount      = entry.read<u8>(kTexMipCount);
    texture.baseLevelSize = entry.read<u32>(kTexBaseLevelSize);
    texture.alphaUsage    = (AlphaUsage)std::min<u8>(entry.read<u8>(kTexAlphaUsage), 2);
    texture.alphaBlend    = (AlphaBlend)std::min<u8>(entry.read<u8>(kTexAlphaBlend), 2);
    if (!format.empty()) texture.format = FormatFromFourCc(format.read<u32>(kFormatFourCc));
    texture.scrollType    = entry.read<u8>(kTexScrollType);
    texture.scrollStep    = entry.read<i16>(kTexScrollStep) / 256.0f;
    texture.scrollS       = entry.read<i16>(kTexScrollS) / 256.0f;
    texture.scrollT       = entry.read<i16>(kTexScrollT) / 256.0f;
    if (texture.scrollType > 2 || (texture.scrollS == 0.0f && texture.scrollT == 0.0f)) texture.scrollType = 0;
    if (!(texture.scrollStep > 0.0f)) texture.scrollStep = 1.0f;
    if (texture.mipCount == 0) texture.mipCount = 1;
    return texture;
}

bool TexturePack::ParseCompressed(const Chunk& packChunk, ByteSpan index) {
    // Index entries: hash, absolute file offset, compressed size, size, flags, 0.
    const u8* file = packChunk.data.data - packChunk.dataOffset;
    const u64 fileEnd = packChunk.dataOffset + packChunk.data.size;
    constexpr size_t kEntry = 24;
    constexpr size_t kTail = kTextureEntrySize + kFormatEntrySize;
    size_t failed = 0;
    for (size_t i = 0; i + kEntry <= index.size; i += kEntry) {
        const u32 offset = index.read<u32>(i + 4), size = index.read<u32>(i + 8);
        if (offset < packChunk.dataOffset || (u64)offset + size > fileEnd) { ++failed; continue; }
        std::vector<u8> block;
        if (!EaDecompress({ file + offset, size }, block) || block.size() < kTail) { ++failed; continue; }
        owned_.push_back(std::move(block));
        const ByteSpan all{ owned_.back().data(), owned_.back().size() };
        const ByteSpan entry = all.sub(all.size - kTail, kTextureEntrySize);
        Texture texture = ReadHeader(entry, all.sub(all.size - kFormatEntrySize, kFormatEntrySize));
        texture.pixels = all.sub(0, std::min<size_t>(entry.read<u32>(kTexDataSize), all.size - kTail));
        if (texture.format == TextureFormat::Palettised8) { ++failed; continue; }   // none ship
        byHash_.emplace(texture.hash, textures_.size());
        textures_.push_back(std::move(texture));
    }
    LOG_INFO("compressed texture pack '%s': %zu textures, %zu unreadable", name_.c_str(), textures_.size(), failed);
    return !textures_.empty();
}

bool TexturePack::ParseNfswCompressed(const Chunk& packChunk, ByteSpan index) {
    // NFS World's compressed packs store each texture as a run of 0x55441122
    // blocks. Each block wraps an ordinary EA block at +0x18; the decompressed
    // pixels are followed by a 144-byte NFSW texture header, its name length,
    // the name and 48 bytes of tail.
    const u8* file = packChunk.data.data - packChunk.dataOffset;
    const u64 fileEnd = packChunk.dataOffset + packChunk.data.size;
    constexpr size_t kEntry = 36;
    constexpr u32 kLzcBlock = 0x55441122;
    size_t failed = 0;
    const size_t before = textures_.size();
    for (size_t i = 0; i + kEntry <= index.size; i += kEntry) {
        const u32 offset      = index.read<u32>(i + 4);
        const u32 storedBytes = index.read<u32>(i + 8);
        const u32 unpacked    = index.read<u32>(i + 12);
        if (!storedBytes || offset < packChunk.dataOffset ||
            (u64)offset + storedBytes > fileEnd) {
            ++failed;
            continue;
        }

        struct Block { std::vector<u8> bytes; u32 unpacked = 0; };
        std::vector<Block> blocks;
        u32 fullBlockSize = 0;
        size_t at = offset;
        const size_t end = (size_t)offset + storedBytes;
        bool blocksOk = true;
        while (at + 24 <= end) {
            const ByteSpan blockAt{ file + at, end - at };
            const u32 blockId       = blockAt.read<u32>(0);
            const u32 blockUnpacked = blockAt.read<u32>(4);
            const u32 stored        = blockAt.read<u32>(8);
            if (blockId != kLzcBlock || stored < 40 || at + stored > end) {
                if (failed == 0)
                    LOG_INFO("NFSW compressed block bad header id %08X stored %u at %08zX",
                             blockId, stored, at);
                blocksOk = false;
                break;
            }
            std::vector<u8> block;
            // Outer header: id, decompressed size, stored size, decompressed
            // size again, then eight zero bytes; the EA block starts at +0x18.
            if (!EaDecompress({ file + at + 24, stored - 24 }, block)) {
                if (failed == 0)
                    LOG_INFO("NFSW compressed block decode failed id %08X stored %u",
                             blockId, stored);
                blocksOk = false;
                break;
            }
            fullBlockSize = std::max(fullBlockSize, blockUnpacked);
            blocks.push_back({ std::move(block), blockUnpacked });
            at += stored;
        }
        if (!blocksOk) { ++failed; continue; }

        std::vector<u8> data;
        data.reserve(unpacked);
        // The file order puts the short tail block before the last full pixel
        // block; the stream is every full block, then the short one.
        for (const Block& block : blocks)
            if (block.unpacked == fullBlockSize)
                data.insert(data.end(), block.bytes.begin(), block.bytes.end());
        for (const Block& block : blocks)
            if (block.unpacked != fullBlockSize)
                data.insert(data.end(), block.bytes.begin(), block.bytes.end());

        const u32 tailSize = index.read<u32>(i + 0x10);
        if (tailSize < kNfswTextureEntrySize + 1 || tailSize > data.size()) {
            ++failed;
            continue;
        }
        const size_t headerAt = data.size() - tailSize;
        const ByteSpan all{ data.data(), data.size() };
        const ByteSpan entry = all.sub(headerAt, kNfswTextureEntrySize);
        const u8 nameLength = all.read<u8>(headerAt + kNfswTextureEntrySize);
        const std::string name = ReadFixedName(
            all, headerAt + kNfswTextureEntrySize + 1,
            std::min<size_t>(nameLength, tailSize - kNfswTextureEntrySize - 1));

        Texture texture;
        texture.name          = name;
        texture.hash          = entry.read<u32>(kNfswTexHash);
        texture.width         = (u16)entry.read<u32>(kNfswTexWidth);
        texture.height        = (u16)entry.read<u32>(kNfswTexHeight);
        texture.baseLevelSize = entry.read<u32>(kNfswTexBaseSize);
        texture.alphaUsage    = (AlphaUsage)std::min<u8>(entry.read<u8>(kNfswTexAlphaUsage), 2);
        texture.alphaBlend    = (AlphaBlend)std::min<u8>(entry.read<u8>(kNfswTexAlphaBlend), 2);
        const u8 scrollType = entry.read<u8>(kNfswTexScrollType);
        const i16 scrollS   = entry.read<i16>(kNfswTexScrollS);
        const i16 scrollT   = entry.read<i16>(kNfswTexScrollT);
        if (scrollType && (scrollS || scrollT)) {
            const i16 scrollStep = entry.read<i16>(kNfswTexScrollStep);
            texture.scrollType = scrollType == 2 ? 2 : 1;
            texture.scrollStep = scrollStep > 0 ? scrollStep / 256.0f : 1.0f;
            texture.scrollS    = scrollS / 16384.0f;
            texture.scrollT    = scrollT / 16384.0f;
        }

        // Compressed packs carry no format table; the base level against the
        // pixel count distinguishes the formats they use.
        const u64 pixels = (u64)texture.width * texture.height;
        if (pixels && texture.baseLevelSize == pixels * 4)
            texture.format = TextureFormat::Argb8888;
        else if (pixels && texture.baseLevelSize == pixels)
            texture.format = TextureFormat::Dxt5;
        else if (pixels && (u64)texture.baseLevelSize * 2 == pixels)
            texture.format = TextureFormat::Dxt1;
        else {
            ++failed;
            continue;
        }

        const u32 imageSize = entry.read<u32>(kNfswTexImageSize);
        if (imageSize > headerAt) { ++failed; continue; }
        texture.mipCount = (u8)std::min<u32>(
            255u, NfswMipCount(texture.width, texture.height, texture.format, imageSize));
        texture.pixels = all.sub(0, imageSize);

        if (texture.hash && texture.width && texture.height) {
            byHash_.emplace(texture.hash, textures_.size());
            textures_.push_back(std::move(texture));
            if (name_.empty()) name_ = textures_.back().name;
            owned_.push_back(std::move(data));
        } else {
            ++failed;
        }
    }
    LOG_INFO("compressed NFSW texture pack '%s': %zu textures, %zu unreadable",
             name_.c_str(), textures_.size() - before, failed);
    return textures_.size() > before;
}

bool TexturePack::Parse(const Chunk& packChunk) {
    textures_.clear();
    byHash_.clear();
    owned_.clear();

    ByteSpan headers, formats, pixels, compressed;

    ChunkReader top = ChunkReader::Children(packChunk);
    Chunk block;
    while (top.Next(block)) {
        if (block.id == kChunkTpkInfoBlock) {
            ChunkReader info = ChunkReader::Children(block);
            Chunk part;
            while (info.Next(part)) {
                switch (part.id) {
                    case kChunkTpkHeader:
                        name_ = ReadFixedName(part.data, 0x0C, 28);
                        break;
                    case kChunkTpkTextureHeaders: headers = part.data; break;
                    case kChunkTpkFormatTable:    formats = part.data; break;
                    case kChunkTpkCompressedIndex: compressed = part.data; break;
                    default: break;
                }
            }
        } else if (block.id == kChunkTpkDataBlock) {
            ChunkReader data = ChunkReader::Children(block);
            Chunk part;
            while (data.Next(part)) {
                if (part.id == kChunkTpkPixelData) pixels = part.data;
            }
        }
    }

    if (headers.empty() && !compressed.empty()) return ParseCompressed(packChunk, compressed);
    if (headers.empty() || pixels.empty()) {
        LOG_WARN("texture pack '%s' is missing headers or pixel data", name_.c_str());
        return false;
    }

    const size_t count = headers.size / kTextureEntrySize;
    textures_.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        const ByteSpan entry = headers.sub(i * kTextureEntrySize, kTextureEntrySize);

        // The format table runs parallel to the header table.
        const ByteSpan format = (i + 1) * kFormatEntrySize <= formats.size
                                    ? formats.sub(i * kFormatEntrySize, kFormatEntrySize) : ByteSpan{};
        Texture texture = ReadHeader(entry, format);

        const u32 dataOffset = entry.read<u32>(kTexDataOffset);
        const u32 dataSize   = entry.read<u32>(kTexDataSize);
        texture.pixels = pixels.sub(dataOffset, dataSize);

        // Palettes live in the gap the pixel block leaves before its first
        // texture slice, addressed the same way as the pixels.
        if (const u32 paletteSize = entry.read<u32>(kTexPaletteSize))
            texture.palette = pixels.sub(entry.read<u32>(kTexPaletteOffset), paletteSize);

        byHash_.emplace(texture.hash, textures_.size());
        textures_.push_back(std::move(texture));
    }

    LOG_INFO("texture pack '%s': %zu textures", name_.c_str(), textures_.size());
    return !textures_.empty();
}

bool TexturePack::ParseNfsw(const Chunk& packChunk) {
    textures_.clear();
    animations_.clear();
    byHash_.clear();
    owned_.clear();

    ByteSpan headers, formats, pixels, compressedIndex;
    std::function<void(const Chunk&)> visit = [&](const Chunk& parent) {
        ChunkReader reader = ChunkReader::Children(parent);
        Chunk child;
        while (reader.Next(child)) {
            switch (child.id) {
                case kChunkTpkHeader:
                    name_ = ReadFixedName(child.data, 0x0C, 28);
                    break;
                case kChunkTpkTextureHeaders:
                    headers = child.data;
                    break;
                case kChunkTpkFormatTable:
                    formats = child.data;
                    break;
                case kChunkTpkPixelData:
                    pixels = child.data;
                    break;
                case kChunkTpkCompressedIndex:
                    compressedIndex = child.data;
                    break;
                case kChunkTpkAnimationContainer: {
                    ChunkReader animReader = ChunkReader::Children(child);
                    Chunk anim;
                    while (animReader.Next(anim)) {
                        if (anim.id != kChunkTpkAnimation) continue;
                        TextureFrameAnimation parsed;
                        if (ParseNfswAnimation(anim.data, parsed))
                            animations_.push_back(std::move(parsed));
                    }
                    break;
                }
                default:
                    if (child.IsContainer()) visit(child);
                    break;
            }
        }
    };
    visit(packChunk);

    if (headers.empty() && !compressedIndex.empty())
        return ParseNfswCompressed(packChunk, compressedIndex);

    if (headers.empty() || pixels.empty()) {
        LOG_WARN("NFS World texture pack '%s' is missing headers or pixel data", name_.c_str());
        return false;
    }

    size_t headerAt = 0;
    size_t index = 0;
    size_t pixelAt = 0;
    while (headerAt + kNfswTextureEntrySize + 1 <= headers.size) {
        const ByteSpan entry = headers.sub(headerAt, kNfswTextureEntrySize);
        const u8 nameLength = headers.read<u8>(headerAt + kNfswTextureEntrySize);
        const std::string name = ReadFixedName(
            headers, headerAt + kNfswTextureEntrySize + 1,
            std::min<size_t>(nameLength, headers.size - headerAt - kNfswTextureEntrySize - 1));

        Texture texture;
        texture.name          = name;
        texture.hash          = entry.read<u32>(kNfswTexHash);
        texture.width         = (u16)entry.read<u32>(kNfswTexWidth);
        texture.height        = (u16)entry.read<u32>(kNfswTexHeight);
        texture.baseLevelSize = entry.read<u32>(kNfswTexBaseSize);
        texture.alphaUsage    = (AlphaUsage)std::min<u8>(entry.read<u8>(kNfswTexAlphaUsage), 2);
        texture.alphaBlend    = (AlphaBlend)std::min<u8>(entry.read<u8>(kNfswTexAlphaBlend), 2);
        // NFS World's scroll accessors (sub_62BBD0/62BC20) read the type at
        // +0x35, the step as 8.8 at +0x3C and the two speeds as 1/16384 at
        // +0x3E/+0x40. UG2 uses different offsets and an 8.8 speed scale.
        const u8 scrollType = entry.read<u8>(kNfswTexScrollType);
        const i16 scrollS = entry.read<i16>(kNfswTexScrollS);
        const i16 scrollT = entry.read<i16>(kNfswTexScrollT);
        if (scrollType && (scrollS || scrollT)) {
            const i16 scrollStep = entry.read<i16>(kNfswTexScrollStep);
            texture.scrollType = scrollType == 2 ? 2 : 1;
            texture.scrollStep = scrollStep > 0 ? scrollStep / 256.0f : 1.0f;
            texture.scrollS    = scrollS / 16384.0f;
            texture.scrollT    = scrollT / 16384.0f;
        }
        if ((index + 1) * kFormatEntrySize <= formats.size)
            texture.format = FormatFromFourCc(
                formats.read<u32>(index * kFormatEntrySize + kNfswFormatFourCc));

        const u32 imageSize = entry.read<u32>(kNfswTexImageSize);
        const u32 dataOffset = entry.read<u32>(kNfswTexDataOffset);
        texture.mipCount = (u8)std::min<u32>(
            255u, NfswMipCount(texture.width, texture.height, texture.format, imageSize));
        texture.pixels = pixels.sub(dataOffset, imageSize);
        pixelAt = std::max<size_t>(pixelAt, (size_t)dataOffset + imageSize);

        if (texture.hash && texture.width && texture.height) {
            byHash_.emplace(texture.hash, textures_.size());
            textures_.push_back(std::move(texture));
        }

        headerAt += kNfswTextureEntrySize + 1 + nameLength;
        ++index;
    }

    if (pixelAt > pixels.size) {
        LOG_WARN("NFS World texture pack '%s': headers claim %zu pixel bytes, block holds %zu",
                 name_.c_str(), pixelAt, pixels.size);
    }
    LOG_INFO("NFS World texture pack '%s': %zu textures, %zu animations", name_.c_str(),
             textures_.size(), animations_.size());
    return !textures_.empty();
}

const Texture* TexturePack::Find(u32 hash) const {
    const auto it = byHash_.find(hash);
    return it == byHash_.end() ? nullptr : &textures_[it->second];
}

} // namespace nfs
