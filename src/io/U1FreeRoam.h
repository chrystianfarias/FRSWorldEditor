#pragma once

#include "core/Types.h"

#include <string>
#include <vector>

namespace io {

// A drive through a whole Need for Speed Underground region, for the editor's
// launches (FRSFastTest's free roam).
//
// Underground 1 has collision only along each race's course, and every
// race's own: one race closes with walls the side streets another drives.
// The free roam's track bundle is the started track's TRACKB<n>.LZC with its
// collision replaced by every track of the region together:
//   - each polygon once, by its geometry (the tracks encode the same street
//     with other height bases and corner bytes);
//   - without the race barriers: a wall (flags 0x04) with ground of some race
//     on both sides, 3 m off along its normal at a quarter, half and three
//     quarters of its length, within 2.5 m in height. A wall at the edge of
//     the drivable city has ground on one side only and stays;
//   - indexed again as the game reads it (quadtree 0x34130, cells 0x34131,
//     polygons 0x34132, indices 0x34133 as u16, bounds 0x34134).
// Written JDLZ-wrapped to <game>\models\freeroam\TRACKB<track>.LZC, which
// FRSFastTest opens in place of the track's own in a free roam.
struct U1FreeRoamReport {
    size_t tracks = 0;          // bundles read
    size_t polygons = 0;        // written
    size_t duplicates = 0;      // the same geometry from another track
    size_t raceBarriers = 0;    // left out
    size_t cityWalls = 0;       // kept
    size_t resetZonesOff = 0;   // the race's reset zones, switched off
    std::wstring path;
    std::wstring error;
};

// The editor's own walls (io/U1Collision.h: the polygons a collision edit of
// `region` added) are never taken for race barriers.
bool WriteU1FreeRoamBundle(const std::wstring& gameFolder, const std::wstring& region, u32 track,
                           const std::vector<u32>& regionTracks, U1FreeRoamReport& report);

} // namespace io
