//
// Created by jwscoggins on 9/19/21.
//

#ifndef SXCHIPSET_TAGGEDCACHEADDRESS_H
#define SXCHIPSET_TAGGEDCACHEADDRESS_H
#include "MCUPlatform.h"
#include "type_traits.h"
template<byte tagBits, byte offsetBits = 4, bool debugMode = false>
union TaggedAddress {
    static constexpr auto NumLowestBits = offsetBits;
    static constexpr auto NumTagBits = tagBits;
    static constexpr auto AddressSize = 32;
    static constexpr auto NumRestBits = AddressSize - (NumTagBits + NumLowestBits);
    static constexpr auto MaximumAddressSize = AddressSize;
    static_assert((NumLowestBits + NumTagBits + NumRestBits) == MaximumAddressSize, "Too many or too few bits for this given tagged address!");
    static_assert((MaximumAddressSize >= 26) && (MaximumAddressSize <= 32), "Addresses cannot be smaller than 26 bits!");
    constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
    constexpr explicit TaggedAddress(Address key, Address tag, Address offset = 0) noexcept : lowest(offset), tagIndex(tag), rest(key) { }
    void clear() noexcept { base = 0; }
    [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
    [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
    [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getOffset() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    [[nodiscard]] constexpr TaggedAddress aligned() const noexcept { return {rest, tagIndex, 0}; }
    [[nodiscard]] bool restEqual(const TaggedAddress& other) const noexcept { return getRest() == other.getRest(); }
private:
    Address base;
    struct {
        Address lowest : NumLowestBits;
        Address tagIndex : NumTagBits;
        Address rest : NumRestBits;
    };
    byte bytes_[4];
};
#endif //SXCHIPSET_TAGGEDCACHEADDRESS_H
