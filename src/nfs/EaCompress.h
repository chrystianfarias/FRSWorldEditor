#pragma once

#include "core/Types.h"

#include <vector>

namespace nfs {

// The EA block compression of compressed texture packs (TRACKS\LOC*DYNTEX.BIN).
// Each block starts with a 16-byte header:
//   +0x0 "JDLZ" | "HUFF" | "RAWW"   +0x4 version   +0x5 header size (0x10)
//   +0x8 uncompressed size          +0xC compressed size
// Returns false when the block is not one of those or does not decode to the
// size its header promises.
bool EaDecompress(ByteSpan block, std::vector<u8>& out);

// NFS World's LZC container: 0x10-byte header (magic 0x66113388, uncompressed
// size at +0x04) followed by 0x55441122 blocks. Each block is a 24-byte header
// (uncompressed size, stored size, uncompressed size, eight zero bytes) and an
// EA block at +0x18.
bool LzcDecompress(ByteSpan file, std::vector<u8>& out);

// The same block run without the 0x10-byte container header, for compressed
// texture packs whose 0x33310003 index points directly at the first block.
// Stops once `expected` bytes are produced when it is nonzero.
bool LzcBlocksDecompress(ByteSpan blocks, std::vector<u8>& out, size_t expected = 0);

} // namespace nfs
