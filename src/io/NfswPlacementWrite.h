#pragma once

#include "core/Types.h"

#include <string>
#include <vector>

#include "nfs/NfswCollision.h"
#include "nfs/SolidList.h"

namespace nfs { class World; struct NfswMeshUpdate; }

// Writes the NFS World section BUNs the editor changed.
//
// NFS World's sections are separate files, so the loader route is a whole-file
// replacement: the patched BUN goes to <models>\files\TRACKS\<section file> and
// FRSWorldLoaderNFSW.asi opens it in place of the shipped one. A placement that
// stays in its section patches the 96-byte 0x34103 record in place; a new
// placement, a delete or a cross-section move rebuilds the affected 0x80034100
// chunk and updates the layout's section sizes, scenery indices and streaming
// dependencies. Unsupported edits are refused before a file is written.
//
// The editor keeps its placements in memory; this function does not reopen the
// region, because the editor reads the shipped section files and would lose the
// changes. `report` lists the written overlays or explains the failure.
bool WriteNfswPlacementOverlays(const nfs::World& world, const std::wstring& modelsFolder,
                                std::wstring& report);

struct NfswMeshOverlay {
    std::wstring file;
    std::vector<u8> bytes;
    bool remove = false;
    bool modelsRoot = false;
};

// Section replacements for edited meshes and the existing placements that
// need larger visibility bounds. No section changes size.
bool PlanNfswMeshOverlays(const nfs::World& world, const std::vector<nfs::NfswMeshUpdate>& meshes,
                          std::vector<NfswMeshOverlay>& overlays, std::wstring& error);
bool WriteNfswMeshOverlays(const std::vector<NfswMeshOverlay>& overlays,
                           const std::wstring& modelsFolder, std::wstring& error);

inline std::wstring NfswBakedRoadCollisionFile(const std::wstring& region) {
    return L"road-collision-" + region + L".frsroadcol";
}

struct NfswRoadPlacement {
    size_t section;
    nfs::SolidObject model;
    Vec3 position;
};

bool PlanNfswRoadOverlays(const nfs::World& world, const std::vector<NfswRoadPlacement>& roads,
                          const std::vector<nfs::NfswRoadSegmentBuild>& routes,
                          const std::vector<nfs::NfswCollisionPlacement>& collision,
                          std::vector<NfswMeshOverlay>& output, nfs::NfswRoadBuildReport& nav,
                          std::wstring& error);
