#pragma once

#include "nfs/Scenery.h"
#include "nfs/NfswCollision.h"

#include <array>
#include <map>
#include <set>

namespace nfs { class World; }

// Plans dynamic collision for standalone copies in the section overlays.
// The layout is staged separately because the section sizes use its original
// table offsets until all section data has been rebuilt.
class NfswSmackableWrite {
public:
    using Copies = std::map<std::pair<u32, u32>, const nfs::NfswSmackablePlacement*>;
    void ReadGeometry(ByteSpan layout);
    bool PatchBindings(std::vector<u8>& section, const std::vector<nfs::NfswSmackableLink>& links);
    bool Start(const nfs::World& world, ByteSpan stockLayout, ByteSpan editedLayout);
    bool RewriteSection(ByteSpan section, const nfs::Scenery& stock, const nfs::Scenery& baseline,
                        const nfs::Scenery& edited, const Copies& copies,
                        const std::vector<size_t>& changed, std::set<size_t>& dynamic,
                        std::vector<u8>& out, std::string& error);
    bool WriteLayout(std::vector<u8>& layout) const;

private:
    using Spawn = std::array<u8, 64>;
    using Entry = std::array<u16, 3>;
    struct Template {
        Spawn spawn;
        u16 flags;
        bool ambiguous = false;
    };

    size_t stockCount_ = 0;
    std::vector<Entry> entries_;
    std::vector<Entry> baselineEntries_;
    std::map<u32, Template> models_;
    nfs::NfswSmackableTypes types_;
    std::set<u32> stockIds_;
    std::map<u32, ByteSpan> geometries_;
    std::map<u32, std::array<u8, 80>> newBoxes_;
    u32 GeometryFor(const nfs::NfswSmackableLink& link);
};
