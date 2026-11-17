#pragma once

#include "core/Types.h"
#include "nfs/Collision.h"

#include <array>
#include <string>
#include <vector>

namespace io {

// Need for Speed Underground's collision: one list of 32-byte polygons per
// track, in the track's bundle (TRACKB<n>.LZC: quadtree 0x34130, cells 0x34131,
// polygons 0x34132, u16 indices 0x34133, bounds 0x34134), every race its own.
//
// An edit is kept as the polygons taken out of the open track's collision and
// the polygons added, in <models>\<REGION>-collision.frsu1col. The editor lays
// it over the track as a region opens (World), and every track's bundle the
// game is handed carries it (io/U1TrackBundles.h): a polygon taken out goes
// from each track that has it, an added one is in all of them -- a wall built
// or a road laid is there whatever race is driven.
//
// Polygons are matched by what the game reads of them, not by their bytes:
// the corners at the file's precision (1/8 in plan, 1/256 in height, barrier
// ends only), the surface and the flags. Tracks encode the same street with
// other height bases.

using U1Record = std::array<u8, 32>;

struct U1CollisionEdit {
    std::vector<U1Record> removed;   // as the open track had them
    std::vector<U1Record> added;     // as they are to be written
    bool Empty() const { return removed.empty() && added.empty(); }
};

std::wstring U1CollisionEditPath(const std::wstring& modelsFolder, const std::wstring& region);
bool ReadU1CollisionEdit(const std::wstring& path, U1CollisionEdit& out);
// An empty edit deletes the file.
bool WriteU1CollisionEdit(const std::wstring& path, const U1CollisionEdit& edit);

// What identifies a polygon (see above).
using U1PolygonKey = std::array<i32, 14>;
U1PolygonKey KeyOf(const nfs::CollisionPolygon& polygon);
U1PolygonKey KeyOfRecord(const U1Record& record);

// `shipped` as `current` has it: what went and what came (multisets).
void DiffU1Collision(const std::vector<nfs::CollisionPolygon>& shipped,
                     const std::vector<nfs::CollisionPolygon>& current, U1CollisionEdit& out, std::wstring& error);

// A track's records with the edit laid over them.
std::vector<U1Record> ApplyU1CollisionEdit(const std::vector<U1Record>& records, const U1CollisionEdit& edit);

// The five collision chunks for `records`, in the game's order and alignment,
// written into a copy of `bundle` in place of its own. False (and why) past
// the file's limits (65535 polygons, cells, nodes).
bool ReplaceU1Collision(const std::vector<u8>& bundle, const std::vector<U1Record>& records,
                        std::vector<u8>& out, std::wstring& error);

// A bundle's own records (0x34132), byte 0 cleared.
std::vector<U1Record> U1CollisionRecords(const std::vector<u8>& bundle);

} // namespace io
