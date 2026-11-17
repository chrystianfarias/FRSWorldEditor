#pragma once

namespace nfs {

// The games whose worlds the editor reads. Only Underground 2's can be
// written; the others open read-only (World::IsReadOnly).
//
// Most Wanted keeps Underground 2's two files per region (TRACKS\L2RA.BUN and
// STREAML2RA.BUN) and most of its chunks, with wider records in a few places:
// 0x5C-byte section table entries, 0x68-byte materials with one vertex buffer
// per run of the same effect, and 0x48-byte scenery infos.
enum class Game { Underground2, Underground1, MostWanted };

} // namespace nfs
