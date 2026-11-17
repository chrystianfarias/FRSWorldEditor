#pragma once

#include "core/Types.h"
#include "nfs/Game.h"

#include <string>
#include <vector>

// Need for Speed Most Wanted's models in Underground 2's layout.
//
// Both games store a model as the object chunk 0x80134010 -- header 0x134011,
// texture list 0x134012, mesh 0x80134100 -- but Most Wanted's mesh is built
// differently (nfs/SolidList.cpp, ParseMeshMW): 0x68-byte materials naming an
// effect, one vertex buffer per run of materials with the same effect, each
// material's indices counting from the start of its own buffer. Underground 2
// has 60-byte materials over a single buffer.
//
// So the model is read the way the editor reads it (every buffer joined into
// one 36-byte {position, normal, colour, uv} array, the indices rebased onto
// it) and written again as Underground 2 lays a world model out:
//
//   0x00134011 header, 192 bytes: Most Wanted's up to +0x90 (hash, bounds,
//              pivot matrix), version 0x16, no object flags, the box's
//              volume and triangles per unit of it at +0x94/+0x98, the name
//              at +0xA4 (cut to 27 characters; the hash stays the full
//              name's, and the game finds models by hash).
//   0x00134012 the textures the materials draw with (their diffuse slot):
//              normal, specular and the other maps have no place in
//              Underground 2's world shader.
//   0x00134900 mesh header, 68 bytes; 0x00134B01 the vertices (36 bytes);
//   0x00134B02 materials (60 bytes); 0x00134B03 indices.
//
// Dropped: light materials (0x134013), position markers (0x13401A), the
// material names (0x134C02). Wind sway, glass reflections and normal maps go
// with them; vertex colours stay as Most Wanted has them.

// `object` is a whole 0x80134010 chunk of Most Wanted that sat at
// `fileOffset` in its file (the 0x11 filler depends on it). `out` gets the
// same model as Underground 2 stores it, laid out to start on a 128-byte
// boundary.
//
// Carbon's models (`game` Carbon) go the same way: the same chunks, wider
// materials and more vertex layouts, all read into the same 36-byte vertices.
bool ConvertMWObject(ByteSpan object, u64 fileOffset, std::vector<u8>& out, std::wstring& error,
                     nfs::Game game = nfs::Game::MostWanted);

// Headless check (FRSWorldEditor.exe --mw-selftest <Most Wanted folder>):
// converts every model of L2RA, reads each back as Underground 2 and compares
// it with the model as read from Most Wanted, and tries to copy every texture
// the converted models use. Writes the result to the log; returns 0 when
// nothing failed.
// --carbon-selftest <Carbon folder>: the same over Carbon's L5RA.
int RunMWSelfTest(const std::wstring& folder, nfs::Game game = nfs::Game::MostWanted);
