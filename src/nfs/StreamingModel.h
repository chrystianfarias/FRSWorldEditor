#pragma once

#include "core/Math.h"
#include "nfs/ChunkReader.h"
#include "nfs/Preculler.h"

#include <vector>

namespace nfs {

// NFS World's section a point would load from. The game looks the predicted
// position up in the layout's type-6 path zones (0x3414A) first: each names up
// to four section ids, and the polygon/list lookup must return one of them.
// Without a path zone the ordinary visibility polygon decides.
//
// The model keeps the previous eye position so it can derive the prediction
// velocity the way sub_742C20 does (1.5 s ahead, clamped to 100 units).
class StreamingModel {
public:
    struct PathZone {
        Vec3 boundsMin;
        Vec3 boundsMax;
        float height = 0.0f;
        u16 candidates[4] = {};
        std::vector<Vec3> ring;

        bool Contains(float x, float y) const;
        bool HeightAllows(float z) const;
    };

    bool Parse(ByteSpan layout);
    bool Empty() const { return pathZones_.empty(); }
    size_t PathZoneCount() const { return pathZones_.size(); }

    // The zone id the game would load from standing at `eye` at `time`
    // (seconds). `predicted` receives the position used for the path-zone
    // lookup. Returns -1 when no zone/list claims the point.
    int SectionAt(const Vec3& eye, double time, const Preculler& visibility,
                  Vec3* predicted = nullptr) const;

    const std::vector<PathZone>& PathZones() const { return pathZones_; }

private:
    std::vector<PathZone> pathZones_;
    mutable Vec3          lastEye_;
    mutable double        lastTime_ = -1.0;
};

} // namespace nfs
