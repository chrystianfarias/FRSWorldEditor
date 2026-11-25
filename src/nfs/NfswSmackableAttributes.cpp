#include "nfs/NfswSmackableAttributes.h"

#include <cmath>
#include <algorithm>

namespace nfs {
namespace {

// Attrib::StringHash32 keys, distinct from the geometry and model hashes.
constexpr u32 kSmackable = 0xCE70D7DB;
constexpr u32 kRigidBodySpecs = 0x7C90BB38;
constexpr u32 kCollectionName = 0x9CA1C8F9;
constexpr u32 kMass = 0x4A56503D;
constexpr u32 kBaseMaterial = 0x4164F5F9;
constexpr u32 kClassExport = 0x5E970CBC;
constexpr u32 kCollectionExport = 0x8E112EB7;
constexpr u32 kTextType = 0xA3F0C234;
constexpr u32 kFloatType = 0x3C16EC5E;
constexpr u32 kRefSpecType = 0x2B936EB7;
constexpr u32 kStringKeyType = 0xA502A824;

std::string ReadString(ByteSpan strings, size_t offset) {
    if (offset >= strings.size) return {};
    const char* start = reinterpret_cast<const char*>(strings.data + offset);
    const auto* end = static_cast<const char*>(std::memchr(start, 0, strings.size - offset));
    return end ? std::string(start, end) : std::string{};
}

struct Vault {
    ByteSpan bin, vlt, exports;
    std::unordered_map<u32, u32> vltPointers, binPointers;

    bool Open(ByteSpan pack) {
        if (pack.size < 16 || pack.read<u32>(0) != 0x4B415056) return false; // VPAK
        const size_t count = pack.read<u32>(4);
        if (count > (pack.size - 16) / 20) return false;
        const ByteSpan strings = pack.sub(pack.read<u32>(8), pack.read<u32>(12));
        for (size_t i = 0; i < count; ++i) {
            const ByteSpan entry = pack.sub(16 + i * 20, 20);
            if (ReadString(strings, entry.read<u32>(0)) != "db") continue;
            bin = pack.sub(entry.read<u32>(12), entry.read<u32>(4));
            vlt = pack.sub(entry.read<u32>(16), entry.read<u32>(8));
            if (bin.size != entry.read<u32>(4) || vlt.size != entry.read<u32>(8)) return false;
            break;
        }
        ByteSpan fixups;
        for (size_t at = 0; at + 8 <= vlt.size;) {
            const u32 id = vlt.read<u32>(at);
            const size_t length = vlt.read<u32>(at + 4);
            if (length < 8 || length > vlt.size - at) return false;
            if (id == 0x4578704E) exports = vlt.sub(at + 8, length - 8); // ExpN
            if (id == 0x5074724E) fixups = vlt.sub(at + 8, length - 8);  // PtrN
            at += length;
        }
        if (bin.read<u32>(0) != 0x53747245 || exports.size < 4 || fixups.empty() ||
            exports.read<u32>(0) > (exports.size - 4) / 16) return false;
        // PtrN selects the block containing the pointer. Both kinds point into bin.
        u16 block = 0;
        for (size_t at = 0; at + 12 <= fixups.size; at += 12) {
            const u16 type = fixups.read<u16>(at + 4);
            const u16 index = fixups.read<u16>(at + 6);
            const u32 target = fixups.read<u32>(at + 8);
            if (type == 0) break;
            if (type == 2) block = index;
            if (type == 3 && index == 1 && block <= 1 && target < bin.size)
                (block == 0 ? vltPointers : binPointers).emplace(fixups.read<u32>(at), target);
        }
        return true;
    }

    std::optional<u32> LayoutField(u32 schema, u32 key, u32 type, u16 size) const {
        if (!schema) return std::nullopt;
        const auto definitions = vltPointers.find(schema + 12);
        if (definitions == vltPointers.end()) return std::nullopt;
        const size_t count = vlt.read<u32>(schema + 8);
        const size_t layoutSize = vlt.read<u32>(schema + 24);
        if (count > (bin.size - definitions->second) / 16) return std::nullopt;
        for (size_t i = 0; i < count; ++i) {
            const auto field = bin.sub(definitions->second + i * 16, 16);
            const u32 offset = field.read<u16>(8);
            if (field.read<u32>(0) == key && field.read<u32>(4) == type &&
                field.read<u16>(10) == size && (field.read<u8>(14) & 3) == 2 && offset + size <= layoutSize)
                return offset;
        }
        return std::nullopt;
    }

    std::optional<u32> FieldAddress(u32 collection, std::optional<u32> offset, size_t size) const {
        const auto layout = vltPointers.find(collection + 28);
        if (!offset || layout == vltPointers.end() || *offset + size > bin.size - layout->second)
            return std::nullopt;
        return layout->second + *offset;
    }

    std::string Text(u32 pointer) const {
        const auto fix = binPointers.find(pointer);
        const size_t stringsEnd = bin.read<u32>(4);
        if (fix == binPointers.end() || stringsEnd < 8 || stringsEnd > bin.size ||
            fix->second < 8 || fix->second >= stringsEnd) return {};
        return ReadString(bin.sub(0, stringsEnd), fix->second);
    }
};

struct Collection {
    u32 parent = 0;
    // A null RefSpec explicitly stops the inherited lookup.
    std::optional<u32> rigidBody;
};

std::optional<u32> ReadRigidBodyRef(const Vault& vault, ByteSpan data, u32 start) {
    const size_t types = data.read<u16>(26);
    const size_t count = data.read<u32>(20);
    const size_t entriesStart = 32 + types * 4;
    if (entriesStart > data.size || count > (data.size - entriesStart) / 12) return std::nullopt;
    for (size_t i = 0; i < count; ++i) {
        const size_t at = entriesStart + i * 12;
        if (data.read<u32>(at) != kRigidBodySpecs) continue;
        const u16 type = data.read<u16>(at + 8);
        const auto fix = vault.vltPointers.find((u32)(start + at + 4));
        if (type >= data.read<u16>(24) || type >= types || data.read<u32>(32 + type * 4) != kRefSpecType ||
            (data.read<u8>(at + 10) & 0x42) || fix == vault.vltPointers.end()) return 0; // array or inline value
        const auto ref = vault.bin.sub(fix->second, 8);
        return ref.read<u32>(0) == kRigidBodySpecs ? ref.read<u32>(4) : 0;
    }
    return std::nullopt;
}


} // namespace

bool NfswReadSmackableAttributes(ByteSpan pack, NfswSmackableCatalog& catalog) {
    catalog.clear();
    Vault vault;
    if (!vault.Open(pack)) return false;
    const size_t count = vault.exports.read<u32>(0);
    u32 smackableSchema = 0, rigidBodySchema = 0;
    std::unordered_map<u32, u32> bodies;
    std::unordered_map<u32, Collection> collections;
    for (size_t i = 0; i < count; ++i) {
        const auto entry = vault.exports.sub(4 + i * 16, 16);
        const auto type = entry.read<u32>(4);
        const u32 length = entry.read<u32>(8), start = entry.read<u32>(12);
        const auto data = vault.vlt.sub(start, length);
        if (data.size != length || length < 32) continue;
        const u32 key = data.read<u32>(0);
        if (type == kClassExport) {
            if (key == kSmackable) smackableSchema = start;
            if (key == kRigidBodySpecs) rigidBodySchema = start;
        } else if (type == kCollectionExport && data.read<u32>(4) == kRigidBodySpecs) {
            bodies.emplace(key, start);
        }
    }
    if (!smackableSchema) return false;
    const auto name = vault.LayoutField(smackableSchema, kCollectionName, kTextType, 4);
    const auto mass = vault.LayoutField(smackableSchema, kMass, kFloatType, 4);
    const auto material = vault.LayoutField(rigidBodySchema, kBaseMaterial, kStringKeyType, 8);
    for (size_t i = 0; i < count; ++i) {
        const auto entry = vault.exports.sub(4 + i * 16, 16);
        if (entry.read<u32>(4) != kCollectionExport) continue;
        const u32 length = entry.read<u32>(8), start = entry.read<u32>(12);
        const auto data = vault.vlt.sub(start, length);
        if (data.size != length || length < 32 || data.read<u32>(4) != kSmackable) continue;
        const u32 key = data.read<u32>(0);
        if (!collections.emplace(key, Collection{ data.read<u32>(8), ReadRigidBodyRef(vault, data, start) }).second)
            continue;
        auto& attributes = catalog[key];
        if (const auto address = vault.FieldAddress(start, name, 4)) attributes.name = vault.Text(*address);
        if (const auto address = vault.FieldAddress(start, mass, 4)) {
            const float value = vault.bin.read<float>(*address);
            if (std::isfinite(value)) attributes.mass = value;
        }
    }

    for (auto& [key, attributes] : catalog) {
        u32 ancestor = key;
        for (size_t i = 0; ancestor && i < collections.size(); ++i) {
            const auto found = collections.find(ancestor);
            if (found == collections.end()) break;
            if (found->second.rigidBody) {
                attributes.rigidBody = *found->second.rigidBody;
                break;
            }
            ancestor = found->second.parent;
        }
        const auto body = bodies.find(attributes.rigidBody);
        if (body == bodies.end()) continue;
        if (const auto address = vault.FieldAddress(body->second, material, 8)) {
            attributes.baseMaterial = vault.bin.read<u32>(*address);
            attributes.materialName = vault.Text(*address + 4);
        }
    }
    return !catalog.empty();
}


} // namespace nfs
