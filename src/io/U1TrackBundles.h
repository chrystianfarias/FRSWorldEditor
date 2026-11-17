#pragma once

#include "core/Types.h"

#include <string>

namespace nfs { class World; }

namespace io {

// Need for Speed Underground keeps the section table in every track's bundle
// (TRACKS\TRACKB<n>.LZC, JDLZ), all over the region's one stream. A section
// the editor rebuilt bigger, or moved past the end of the stream, has to be
// found where it now is by every track of the region -- so each of their
// bundles is written again with the corrected entries (offset, size, size
// without texture data, byte sum, by the section's tag) and the stream's size
// in 0x00034108, to <models>\files\TRACKS\TRACKB<n>.LZC, which the FRS World
// Loader opens in place of the game's. Compressed for real (the game unpacks
// them in place) and checked by unpacking before they are written.
//
// The editor applies the corrections to its own table from the layout spans
// it writes (World::OpenUnderground1); this hands them to the game. Run as a
// region opens -- every save reopens it. Nothing is written when the bundles
// on disk already say the same (a signature file beside them), and they are
// taken away when no section is out of place any more.
//
// Returns the number of bundles written (0 when nothing had to change).
size_t WriteU1TrackBundles(const nfs::World& world, const std::wstring& gameFolder,
                           const std::wstring& modelsFolder, std::wstring& error);

// The bundle of a track as the game is handed it: <models>\files\TRACKS\
// TRACKB<n>.LZC when the editor wrote one, else the game's own.
std::wstring U1TrackBundlePath(const std::wstring& gameFolder, const std::wstring& modelsFolder, u32 track);

} // namespace io
