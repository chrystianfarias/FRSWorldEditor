#include "nfs/SceneryEdit.h"

#include "nfs/ChunkReader.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace nfs {

namespace {

constexpr u32 kSceneryPack      = 0x80034100;
constexpr u32 kSceneryHeader    = 0x00034101;
constexpr u32 kSceneryInfos     = 0x00034102;
constexpr u32 kSceneryInstances = 0x00034103;
constexpr u32 kSceneryTree      = 0x00034104;

constexpr size_t kHeaderNumber = 0x0C;

constexpr size_t kInfoSize        = 68;
constexpr size_t kInfoName        = 0x00;
constexpr size_t kInfoHash        = 0x20;
constexpr size_t kInfoHashAgain   = 0x24;
// The far (and reflected) model: XO_CATSEYEORANGE_1Z_00 for ..._1A_00. A copy
// of another section's record names a model this section may not have, and
// the game crashes resolving it (SPEED2 0x493832), so it is the placed model too.
constexpr size_t kInfoFarHash     = 0x28;
constexpr size_t kInfoTriangles   = 0x2C;   // u16, and again at 0x2E
constexpr size_t kInfoRadius      = 0x3C;

constexpr size_t kInstanceSize        = 64;
constexpr size_t kInstBoundsMin       = 0x00;
constexpr size_t kInstBoundsMax       = 0x0C;
constexpr size_t kInstInfoIndex       = 0x18;
constexpr size_t kInstFlags           = 0x1A;
constexpr size_t kInstTranslation     = 0x20;
constexpr size_t kInstRotation        = 0x2C;

constexpr float kRotationScale = 8192.0f;

// Underground 1 (nfs/Scenery.h): the same chunk with smaller records. Infos of
// 72 bytes -- six model hashes (detail levels; 4 and 5 for the reflections),
// i16 size thresholds at 0x18, eModel pointers at 0x24 filled at run time, the
// radius at 0x44 -- and no name. Instances of 48: i16 box min and max in whole
// units, the info index at 0x0C, the flags at 0x0E, the translation at 0x10,
// the rotation at 0x1C. Tree nodes of 24: i16 box, i16 child count at 12, five
// i16 children at 14 (the same references as Underground 2's). The flag of a
// mesh already in world space is 0x0200.
//
// Which one a chunk is, is told from the chunk itself (Split): the tables'
// sizes and the records' model indices. A record alone is told by its size.
// Each public function holds the answer for its own call (FormatScope), on its
// own thread: a model library may be reading Underground 1 while the open
// region is Underground 2's.
thread_local bool g_underground1 = false;

struct FormatScope {
    bool previous;
    explicit FormatScope(bool underground1) : previous(g_underground1) { g_underground1 = underground1; }
    ~FormatScope() { g_underground1 = previous; }
};

struct Layout {
    size_t info, instance, node;
    size_t infoHash, instInfo, instFlags, instTranslation, instRotation;
    size_t nodeCount, nodeChildren;
    u16 worldSpace;
};
constexpr Layout kU2Layout{ kInfoSize, kInstanceSize, 36, kInfoHash, kInstInfoIndex, kInstFlags, kInstTranslation,
                            kInstRotation, 24, 26, 0x0100 };
constexpr Layout kU1Layout{ 72, 48, 24, 0x00, 0x0C, 0x0E, 0x10, 0x1C, 12, 14, 0x0200 };
constexpr size_t kU1InfoRadius = 0x44;
const Layout& L() { return g_underground1 ? kU1Layout : kU2Layout; }

// A tree node: the box of everything under it, a child count and up to five
// children. A child below 0x8000 is an instance; one above is the node at
// 0x10000 - child. Node 0 is the root.
constexpr size_t kNodeChildren = 5;

struct Node {
    Aabb bounds;
    std::vector<u16> children;
};

bool IsNodeRef(u16 child) { return child >= 0x8000; }
u16  NodeRef(size_t index) { return (u16)(0x10000 - index); }
size_t NodeIndex(u16 child) { return 0x10000 - (size_t)child; }

// The pieces of a scenery chunk, as spans into the source.
struct Parts {
    struct Child { u32 id; ByteSpan raw; ByteSpan data; };
    std::vector<Child> children;
    ByteSpan header, infos, instances, tree;
    bool underground1 = false;
};

// Whether the tables read whole as `layout`, every record naming a listed model.
bool Fits(const Parts& parts, const Layout& layout) {
    if (parts.infos.size % layout.info || parts.instances.size % layout.instance || parts.tree.size % layout.node)
        return false;
    const size_t infos = parts.infos.size / layout.info;
    for (size_t i = 0; i < parts.instances.size / layout.instance; ++i)
        if (parts.instances.read<u16>(i * layout.instance + layout.instInfo) >= infos) return false;
    return true;
}

bool Split(ByteSpan chunk, u64 fileOffset, Parts& out) {
    if (chunk.size < 8 || chunk.read<u32>(0) != kSceneryPack) return false;
    Chunk container;
    container.id = kSceneryPack;
    container.headerOffset = fileOffset;
    container.raw = chunk.sub(8, chunk.read<u32>(4));
    container.data = container.raw;

    ChunkReader reader = ChunkReader::Children(container);
    reader.SetSkipZeroPadding(false);
    Chunk child;
    while (reader.Next(child)) {
        out.children.push_back({ child.id, child.raw, child.data });
        switch (child.id) {
            // Only the instance records carry alignment filler. The other
            // tables start straight after their header, and one shipped tree
            // legitimately begins with a 0x11 byte that a filler check would
            // eat, so they are taken raw.
            // Underground 1's header carries filler (0x11 words) before its
            // fields; read past it, the number at +0x0C is the section's.
            case kSceneryHeader: {
                out.header = child.raw;
                while (out.header.size >= 4 && out.header.read<u32>(0) == 0x11111111u)
                    out.header = out.header.sub(4, out.header.size - 4);
                break;
            }
            case kSceneryInfos:     out.infos     = child.raw;  break;
            case kSceneryInstances: out.instances = child.data; break;
            case kSceneryTree:      out.tree      = child.raw;  break;
            default: break;
        }
    }
    // Underground 1 when only its layout fits; when both do (sizes that are
    // whole in both), Underground 2's infos start with the model's name.
    const bool u1 = Fits(out, kU1Layout), u2 = Fits(out, kU2Layout);
    out.underground1 = u1 && (!u2 || out.infos.empty() ||
                              !(out.infos.data[0] >= 'A' && out.infos.data[0] <= 'Z'));
    return !out.infos.empty() || !out.instances.empty();
}

void Put16(std::vector<u8>& out, size_t at, u16 v) { std::memcpy(out.data() + at, &v, 2); }
void Put32(std::vector<u8>& out, size_t at, u32 v) { std::memcpy(out.data() + at, &v, 4); }
void PutF(std::vector<u8>& out, size_t at, float v) { std::memcpy(out.data() + at, &v, 4); }
void PutVec3(std::vector<u8>& out, size_t at, const Vec3& v) {
    PutF(out, at, v.x); PutF(out, at + 4, v.y); PutF(out, at + 8, v.z);
}

Vec3 ReadVec3(ByteSpan span, size_t at) {
    return { span.read<float>(at), span.read<float>(at + 4), span.read<float>(at + 8) };
}

// Underground 1 keeps boxes in whole units as i16.
Vec3 ReadI16Vec3(ByteSpan span, size_t at) {
    return { (float)span.read<i16>(at), (float)span.read<i16>(at + 2), (float)span.read<i16>(at + 4) };
}
i16 ClampI16(float v) { return (i16)std::clamp<long>(std::lround(v), -32768, 32767); }
void PutBoxI16(std::vector<u8>& out, size_t at, const Aabb& box) {
    const i16 v[6] = { ClampI16(std::floor(box.min.x)), ClampI16(std::floor(box.min.y)), ClampI16(std::floor(box.min.z)),
                       ClampI16(std::ceil(box.max.x)),  ClampI16(std::ceil(box.max.y)),  ClampI16(std::ceil(box.max.z)) };
    std::memcpy(out.data() + at, v, sizeof(v));
}

Aabb ReadBox(ByteSpan span, size_t at) {
    Aabb box;
    if (g_underground1) {
        box.min = ReadI16Vec3(span, at);
        box.max = ReadI16Vec3(span, at + 6);
    } else {
        box.min = ReadVec3(span, at);
        box.max = ReadVec3(span, at + 12);
    }
    return box;
}
void PutBox(std::vector<u8>& out, size_t at, const Aabb& box);

void PutBox(std::vector<u8>& out, size_t at, const Aabb& box) {
    if (g_underground1) {
        PutBoxI16(out, at, box);
    } else {
        PutVec3(out, at, box.min);
        PutVec3(out, at + 12, box.max);
    }
}

Aabb InstanceBounds(ByteSpan instances, size_t index) {
    return ReadBox(instances, index * L().instance + kInstBoundsMin);
}

// A tree node: its box (as ReadBox), the child count and the children.
void ReadNode(ByteSpan raw, Aabb& box, u16& count, size_t& childrenAt) {
    box = ReadBox(raw, 0);
    count = raw.read<u16>(L().nodeCount);
    childrenAt = L().nodeChildren;
}

float Grow(const Aabb& box, const Aabb& add) {
    Aabb both = box;
    both.Add(add);
    const Vec3 a = box.max - box.min, b = both.max - both.min;
    return b.x * b.y * b.z - a.x * a.y * a.z + (b.x + b.y + b.z) - (a.x + a.y + a.z);
}

// Puts one record into the tree, changing as little of it as it can.
void Insert(std::vector<Node>& nodes, u16 instance, const Aabb& box,
            const std::vector<Aabb>& instanceBoxes) {
    if (nodes.empty()) {
        nodes.push_back({ box, { instance } });
        return;
    }

    std::vector<size_t> path;
    size_t at = 0;
    while (true) {
        path.push_back(at);
        Node& node = nodes[at];
        if (node.children.size() < kNodeChildren) {
            node.children.push_back(instance);
            break;
        }

        // Down the child node that grows least; failing any, the instance
        // child that grows least is paired with the new one under a new node.
        size_t bestNode = SIZE_MAX, bestInstance = SIZE_MAX;
        float nodeCost = 1e30f, instanceCost = 1e30f;
        for (size_t k = 0; k < node.children.size(); ++k) {
            const u16 child = node.children[k];
            if (IsNodeRef(child)) {
                const float cost = Grow(nodes[NodeIndex(child)].bounds, box);
                if (cost < nodeCost) { nodeCost = cost; bestNode = NodeIndex(child); }
            } else {
                const float cost = Grow(instanceBoxes[child], box);
                if (cost < instanceCost) { instanceCost = cost; bestInstance = k; }
            }
        }

        if (bestNode != SIZE_MAX) { at = bestNode; continue; }

        const u16 paired = node.children[bestInstance];
        Node pair;
        pair.bounds = instanceBoxes[paired];
        pair.bounds.Add(box);
        pair.children = { paired, instance };
        nodes.push_back(pair);
        nodes[at].children[bestInstance] = NodeRef(nodes.size() - 1);
        break;
    }

    for (size_t index : path) nodes[index].bounds.Add(box);
}

} // namespace

Aabb PlacedInstance::WorldBounds() const {
    return Transform(modelBounds, transform);
}

bool ReadSceneryChunk(ByteSpan chunk, u64 fileOffset, SceneryChunkInfo& out) {
    Parts parts;
    if (!Split(chunk, fileOffset, parts)) return false;
    const FormatScope scope(parts.underground1);
    out.number = parts.header.read<u32>(kHeaderNumber);
    out.infoCount = parts.infos.size / L().info;
    out.instanceCount = parts.instances.size / L().instance;
    return true;
}

bool ReadSceneryPlacements(ByteSpan chunk, u64 fileOffset, size_t first,
                           std::vector<PlacedInstance>& out) {
    Parts parts;
    if (!Split(chunk, fileOffset, parts)) return false;
    const FormatScope scope(parts.underground1);

    const Layout& f = L();
    const size_t count = parts.instances.size / f.instance;
    for (size_t i = first; i < count; ++i) {
        const ByteSpan record = parts.instances.sub(i * f.instance, f.instance);
        const u16 infoIndex = record.read<u16>(f.instInfo);
        const ByteSpan info = parts.infos.sub((size_t)infoIndex * f.info, f.info);

        PlacedInstance placed;
        placed.modelHash = info.read<u32>(f.infoHash);
        if (!g_underground1)   // Underground 1's infos name no model
            placed.modelName.assign((const char*)info.data,
                                    ::strnlen((const char*)info.data, std::min<size_t>(info.size, 32)));
        placed.flags = record.read<u16>(f.instFlags);
        placed.infoTemplate.assign(info.data, info.data + info.size);

        placed.transform = Mat4::Identity();
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                placed.transform.m[row][col] =
                    record.read<i16>(f.instRotation + (size_t)(row * 3 + col) * 2) / kRotationScale;
        const Vec3 t = ReadVec3(record, f.instTranslation);
        placed.transform.m[3][0] = t.x;
        placed.transform.m[3][1] = t.y;
        placed.transform.m[3][2] = t.z;
        out.push_back(std::move(placed));
    }
    return true;
}

// Each node's box made the union of its children's again, leaves first.
void RecomputeBounds(std::vector<Node>& nodes, const std::vector<Aabb>& boxes, size_t at) {
    Aabb box;
    for (const u16 child : nodes[at].children) {
        if (IsNodeRef(child)) {
            RecomputeBounds(nodes, boxes, NodeIndex(child));
            box.Add(nodes[NodeIndex(child)].bounds);
        } else if (child < boxes.size()) {
            box.Add(boxes[child]);
        }
    }
    nodes[at].bounds = box;
}

bool ReadSceneryRecords(ByteSpan chunk, u64 fileOffset, std::vector<std::vector<u8>>& out) {
    Parts parts;
    if (!Split(chunk, fileOffset, parts)) return false;
    const FormatScope scope(parts.underground1);
    for (size_t i = 0; i < parts.instances.size / L().instance; ++i) {
        const ByteSpan record = parts.instances.sub(i * L().instance, L().instance);
        out.emplace_back(record.data, record.data + record.size);
    }
    return true;
}

std::string SceneryRecordModel(ByteSpan chunk, u64 fileOffset, size_t index) {
    Parts parts;
    if (!Split(chunk, fileOffset, parts)) return {};
    const FormatScope scope(parts.underground1);
    const u16 info = parts.instances.read<u16>(index * L().instance + L().instInfo);
    const ByteSpan entry = parts.infos.sub((size_t)info * L().info, L().info);
    if (entry.size < 32) return {};
    if (g_underground1) {   // no name: the model's hash
        char text[16];
        std::snprintf(text, sizeof(text), "%08X", entry.read<u32>(0));
        return text;
    }
    return std::string((const char*)entry.data, ::strnlen((const char*)entry.data, 32));
}

Vec3 RecordTranslation(const std::vector<u8>& record) {
    const FormatScope scope(record.size() == kU1Layout.instance);
    if (record.size() < L().instTranslation + 12) return {};
    return ReadVec3(ByteSpan{ record.data(), record.size() }, L().instTranslation);
}

std::vector<u8> DeletedRecord(const std::vector<u8>& record) {
    const FormatScope scope(record.size() == kU1Layout.instance);
    const Layout& f = L();
    std::vector<u8> out = record;
    if (out.size() >= f.instRotation + 18) std::memset(out.data() + f.instRotation, 0, 18);
    // Flag 0x0100 (Underground 1: 0x0200): the mesh is already in world space
    // and the game draws it WITHOUT the matrix (sub_5D6570), so a zeroed matrix
    // would not remove it. Cleared, the zero matrix applies like on any other
    // record.
    if (out.size() >= f.instFlags + 2) {
        u16 flags = 0;
        std::memcpy(&flags, out.data() + f.instFlags, 2);
        flags &= (u16)~f.worldSpace;
        std::memcpy(out.data() + f.instFlags, &flags, 2);
    }
    return out;
}

bool IsDeletedRecord(const std::vector<u8>& record) {
    const FormatScope scope(record.size() == kU1Layout.instance);
    const size_t at = L().instRotation;
    if (record.size() < at + 18) return false;
    for (size_t i = 0; i < 18; ++i)
        if (record[at + i] != 0) return false;
    return true;
}

bool AppendScenery(ByteSpan shipped, u64 fileOffset, const std::vector<PlacedInstance>& added,
                   std::vector<u8>& out, std::wstring& error,
                   const std::vector<ReplacedRecord>& replaced) {
    Parts parts;
    if (!Split(shipped, fileOffset, parts) || parts.header.empty()) {
        error = L"That is not a scenery chunk.";
        return false;
    }
    const FormatScope scope(parts.underground1);

    // --- the model table ------------------------------------------------------
    const Layout& f = L();
    std::vector<u8> infos(parts.infos.data, parts.infos.data + parts.infos.size);
    std::vector<u16> infoFor(added.size());
    for (size_t k = 0; k < added.size(); ++k) {
        const PlacedInstance& placed = added[k];
        const size_t count = infos.size() / f.info;
        size_t found = count;
        for (size_t i = 0; i < count && found == count; ++i) {
            u32 hash;
            std::memcpy(&hash, infos.data() + i * f.info + f.infoHash, 4);
            if (hash == placed.modelHash) found = i;
        }

        if (found == count && g_underground1) {
            // Underground 1: the template's levels all made the placed model
            // (another section's other levels may not be here), the radius kept.
            if (count >= 0xFFFF) { error = L"The section lists too many models already."; return false; }
            std::vector<u8> entry(f.info, 0);
            if (placed.infoTemplate.size() == f.info) {
                entry = placed.infoTemplate;
            } else {
                const i16 threshold = 17;   // drawn from a projected size of 17 on, as the game's own
                std::memcpy(entry.data() + 0x18, &threshold, 2);
                PutF(entry, kU1InfoRadius, placed.radius);
            }
            for (size_t slot = 0; slot < 6; ++slot) {
                u32 hash = 0;
                std::memcpy(&hash, entry.data() + slot * 4, 4);
                if (slot == 0 || hash != 0) Put32(entry, slot * 4, placed.modelHash);
            }
            std::memset(entry.data() + 0x24, 0, 0x20);   // the run time's model pointers
            infos.insert(infos.end(), entry.begin(), entry.end());
        } else if (found == count) {
            if (count >= 0xFFFF) { error = L"The section lists too many models already."; return false; }
            std::vector<u8> entry(kInfoSize, 0);
            if (placed.infoTemplate.size() == kInfoSize) {
                entry = placed.infoTemplate;
            } else {
                std::memcpy(entry.data() + kInfoName, placed.modelName.data(),
                            std::min<size_t>(placed.modelName.size(), 31));
                const u16 triangles = (u16)std::min<u32>(placed.triangles, 0xFFFF);
                Put16(entry, kInfoTriangles, triangles);
                Put16(entry, kInfoTriangles + 2, triangles);
                PutF(entry, kInfoRadius, placed.radius);
            }
            Put32(entry, kInfoHash, placed.modelHash);
            Put32(entry, kInfoHashAgain, placed.modelHash);
            u32 farModel = 0;
            std::memcpy(&farModel, entry.data() + kInfoFarHash, 4);
            if (farModel != 0) Put32(entry, kInfoFarHash, placed.modelHash);
            infos.insert(infos.end(), entry.begin(), entry.end());
        }
        infoFor[k] = (u16)found;
    }

    // --- the records ----------------------------------------------------------
    const size_t shippedCount = parts.instances.size / f.instance;
    if (shippedCount + added.size() >= 0x8000) {
        error = L"The section places too many instances for its tree.";
        return false;
    }

    std::vector<u8> instances(parts.instances.data, parts.instances.data + parts.instances.size);
    bool boxesMoved = false;
    const size_t boxBytes = g_underground1 ? 12 : 24;
    for (const ReplacedRecord& swap : replaced) {
        if (swap.index >= shippedCount || swap.record.size() != f.instance) {
            error = L"An edited record does not fit the section's scenery.";
            return false;
        }
        const ByteSpan before = parts.instances.sub(swap.index * f.instance, f.instance);
        if (std::memcmp(before.data, swap.record.data(), boxBytes) != 0) boxesMoved = true;
        std::memcpy(instances.data() + swap.index * f.instance, swap.record.data(), f.instance);
    }
    const ByteSpan edited{ instances.data(), instances.size() };
    std::vector<Aabb> boxes;
    for (size_t i = 0; i < shippedCount; ++i) boxes.push_back(InstanceBounds(edited, i));

    for (size_t k = 0; k < added.size(); ++k) {
        const PlacedInstance& placed = added[k];
        std::vector<u8> record(f.instance, 0);
        Aabb box = placed.WorldBounds();
        PutBox(record, kInstBoundsMin, box);
        if (g_underground1) box = ReadBox(ByteSpan{ record.data(), record.size() }, kInstBoundsMin);   // as stored
        Put16(record, f.instInfo, infoFor[k]);
        Put16(record, f.instFlags, placed.flags);
        PutVec3(record, f.instTranslation,
                { placed.transform.m[3][0], placed.transform.m[3][1], placed.transform.m[3][2] });
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const long fixed = std::lround(placed.transform.m[row][col] * kRotationScale);
                if (fixed < -32768 || fixed > 32767) {
                    error = L"A placement is scaled by more than four; the file cannot store it.";
                    return false;
                }
                Put16(record, f.instRotation + (size_t)(row * 3 + col) * 2, (u16)(i16)fixed);
            }
        }
        instances.insert(instances.end(), record.begin(), record.end());
        boxes.push_back(box);
    }

    // --- the tree -------------------------------------------------------------
    std::vector<Node> nodes;
    for (size_t n = 0; n < parts.tree.size / f.node; ++n) {
        const ByteSpan raw = parts.tree.sub(n * f.node, f.node);
        Node node;
        u16 count = 0;
        size_t childrenAt = 0;
        ReadNode(raw, node.bounds, count, childrenAt);
        count = std::min<u16>(count, (u16)kNodeChildren);
        for (u16 c = 0; c < count; ++c) node.children.push_back(raw.read<u16>(childrenAt + (size_t)c * 2));
        nodes.push_back(node);
    }
    if (boxesMoved && !nodes.empty()) RecomputeBounds(nodes, boxes, 0);
    for (size_t k = 0; k < added.size(); ++k)
        Insert(nodes, (u16)(shippedCount + k), boxes[shippedCount + k], boxes);

    std::vector<u8> tree;
    if (added.empty() && !boxesMoved) {
        tree.assign(parts.tree.data, parts.tree.data + parts.tree.size);
    } else {
        tree.assign(nodes.size() * f.node, 0);
        for (size_t n = 0; n < nodes.size(); ++n) {
            PutBox(tree, n * f.node, nodes[n].bounds);
            Put16(tree, n * f.node + f.nodeCount, (u16)nodes[n].children.size());
            for (size_t c = 0; c < nodes[n].children.size(); ++c)
                Put16(tree, n * f.node + f.nodeChildren + c * 2, nodes[n].children[c]);
        }
    }

    // --- the chunk ------------------------------------------------------------
    out.clear();
    out.resize(8);
    Put32(out, 0, kSceneryPack);

    for (const Parts::Child& child : parts.children) {
        const std::vector<u8>* replacement = nullptr;
        switch (child.id) {
            case kSceneryInfos:     replacement = &infos;     break;
            case kSceneryInstances: replacement = &instances; break;
            case kSceneryTree:      replacement = &tree;      break;
            default: break;
        }

        const size_t headerAt = out.size();
        out.resize(headerAt + 8);
        Put32(out, headerAt, child.id);

        if (!replacement) {
            out.insert(out.end(), child.raw.data, child.raw.data + child.raw.size);
        } else {
            // Only the instance records are aligned in any shipped pack: to
            // sixteen bytes by their position in the file. The rest start
            // straight after their header.
            if (child.id == kSceneryInstances) {
                const u64 payloadAt = fileOffset + out.size();
                const size_t filler = (size_t)((16 - payloadAt % 16) % 16);
                out.insert(out.end(), filler, 0x11);
            }
            out.insert(out.end(), replacement->begin(), replacement->end());
        }
        Put32(out, headerAt + 4, (u32)(out.size() - headerAt - 8));
    }
    Put32(out, 4, (u32)(out.size() - 8));
    return true;
}

bool ValidateScenery(ByteSpan chunk, u64 fileOffset, std::string& why) {
    Parts parts;
    if (!Split(chunk, fileOffset, parts)) { why = "not a scenery chunk"; return false; }
    const FormatScope scope(parts.underground1);

    const Layout& f = L();
    const size_t count = parts.instances.size / f.instance;
    const size_t infos = parts.infos.size / f.info;
    const size_t nodes = parts.tree.size / f.node;
    if (parts.instances.size % f.instance || parts.infos.size % f.info ||
        parts.tree.size % f.node) { why = "a table is not a whole number of entries"; return false; }

    std::vector<Aabb> boxes;
    for (size_t i = 0; i < count; ++i) {
        boxes.push_back(InstanceBounds(parts.instances, i));
        if (parts.instances.read<u16>(i * f.instance + f.instInfo) >= infos) {
            why = "an instance names a model the section does not list";
            return false;
        }
    }
    if (count == 0) return true;
    if (nodes == 0) { why = "instances but no tree"; return false; }

    std::vector<int> seenInstance(count, 0), seenNode(nodes, 0);
    const float slack = 1e-3f;
    for (size_t n = 0; n < nodes; ++n) {
        const ByteSpan raw = parts.tree.sub(n * f.node, f.node);
        Aabb box;
        u16 children = 0;
        size_t childrenAt = 0;
        ReadNode(raw, box, children, childrenAt);
        if (children == 0 || children > kNodeChildren) { why = "a node has no or too many children"; return false; }
        for (u16 c = 0; c < children; ++c) {
            const u16 child = raw.read<u16>(childrenAt + (size_t)c * 2);
            Aabb inner;
            if (IsNodeRef(child)) {
                const size_t k = NodeIndex(child);
                if (k == 0 || k >= nodes) { why = "a node points outside the tree"; return false; }
                ++seenNode[k];
                inner = ReadBox(parts.tree.sub(k * f.node, f.node), 0);
            } else {
                if (child >= count) { why = "a node points past the instances"; return false; }
                ++seenInstance[child];
                inner = boxes[child];
            }
            if (inner.min.x < box.min.x - slack || inner.min.y < box.min.y - slack ||
                inner.min.z < box.min.z - slack || inner.max.x > box.max.x + slack ||
                inner.max.y > box.max.y + slack || inner.max.z > box.max.z + slack) {
                why = "a node's box does not hold one of its children";
                return false;
            }
        }
    }
    for (size_t i = 0; i < count; ++i)
        if (seenInstance[i] != 1) { why = "an instance is not in the tree exactly once"; return false; }
    for (size_t n = 1; n < nodes; ++n)
        if (seenNode[n] != 1) { why = "a node is not referenced exactly once"; return false; }
    return true;
}

Mat4 ComposePlacement(const Vec3& position, float heading, float pitch, float roll, float scale) {
    const float h = Radians(heading), p = Radians(pitch), r = Radians(roll);
    const float ch = std::cos(h), sh = std::sin(h);
    const float cp = std::cos(p), sp = std::sin(p);
    const float cr = std::cos(r), sr = std::sin(r);

    // Row vectors, so the product reads in the order it applies: roll about
    // x, then pitch about y, then heading about z.
    Mat4 m = Mat4::Identity();
    m.m[0][0] = cp * ch;                 m.m[0][1] = cp * sh;                 m.m[0][2] = -sp;
    m.m[1][0] = -cr * sh + sr * sp * ch; m.m[1][1] = cr * ch + sr * sp * sh;  m.m[1][2] = sr * cp;
    m.m[2][0] = sr * sh + cr * sp * ch;  m.m[2][1] = -sr * ch + cr * sp * sh; m.m[2][2] = cr * cp;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) m.m[row][col] *= scale;
    m.m[3][0] = position.x;
    m.m[3][1] = position.y;
    m.m[3][2] = position.z;
    return m;
}

void DecomposePlacement(const Mat4& m, Vec3& position, float& heading, float& pitch,
                        float& roll, float& scale) {
    position = { m.m[3][0], m.m[3][1], m.m[3][2] };
    scale = std::sqrt(m.m[0][0] * m.m[0][0] + m.m[0][1] * m.m[0][1] + m.m[0][2] * m.m[0][2]);
    const float s = scale > 1e-6f ? scale : 1.0f;
    const float sp = std::clamp(-m.m[0][2] / s, -1.0f, 1.0f);
    pitch   = Degrees(std::asin(sp));
    heading = Degrees(std::atan2(m.m[0][1], m.m[0][0]));
    roll    = Degrees(std::atan2(m.m[1][2], m.m[2][2]));
}

} // namespace nfs
