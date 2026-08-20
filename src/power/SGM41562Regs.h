#pragma once

#include <stdint.h>

// SGM41562 REG04 (charge-voltage regulation) bit layouts.
//
// Deliberately split out of SGM41562.h and left outside the HAS_SGM41562 guard: this is the one
// piece of arithmetic in the charger driver that can overcharge a cell, and no native, coverage or
// pr-level build environment defines HAS_SGM41562 - inside the guard it would be untestable. Keeping
// it here means the static_asserts fire on every target and test/test_charge_limit can include it.
//
// There are TWO register maps across the supported parts, and they cross-validate: the two
// post-reset bytes the driver already fingerprints on (0xA3 and 0x8D, see detectIdZeroChipType())
// each decode to exactly 4200 mV under their own different formula.
namespace SGM41562Regs
{
// Non-extended parts: SGM41562 (device id 0x04), SGM41562A (0x02), SGM41562B (0x00 with REG04
// reset 0xA3). D[7:2] = VBAT_REG[5:0], 3600 mV + 15 mV per step. The mask preserves D[1] VBAT_PRE
// and D[0] VRECH.
// Caveat: device id 0x04 has no published datasheet - its layout comes from LilyGo's decoder, which
// is why setChargeTargetVoltage() gates every write on the reset-value fingerprint below.
inline constexpr uint8_t STD_MASK = 0xFC;
inline constexpr uint8_t STD_SHIFT = 2;
inline constexpr uint8_t STD_RESET = 0xA3;
inline constexpr uint16_t STD_OFFSET_MV = 3600;
inline constexpr uint16_t STD_STEP_MV = 15;
inline constexpr uint16_t STD_MAX_FIELD = 40; // 3600 + 40 * 15 = 4200 mV

// Extended parts: SGM41562S (0x09), SGM41562SA (0x00 with REG04 reset 0x8D).
// D[7:1] = VBAT_REG[6:0], 3500 mV + 10 mV per step. The mask preserves D[0] VRECH; VBAT_PRE moved to
// REG0C D[1] on these parts.
inline constexpr uint8_t EXT_MASK = 0xFE;
inline constexpr uint8_t EXT_SHIFT = 1;
inline constexpr uint8_t EXT_RESET = 0x8D;
inline constexpr uint16_t EXT_OFFSET_MV = 3500;
inline constexpr uint16_t EXT_STEP_MV = 10;
inline constexpr uint16_t EXT_MAX_FIELD = 70; // 3500 + 70 * 10 = 4200 mV

/// Hard ceiling. Equals the OTP default on every supported variant, so a write is only ever a
/// reduction from what the silicon booted with.
inline constexpr uint16_t MAX_TARGET_MV = 4200;
/// Floor. LOAD-BEARING: encode() subtracts the map offset (3500 / 3600 mV) with no guard of its own,
/// so a caller must reject anything below this first or the subtraction underflows.
inline constexpr uint16_t MIN_TARGET_MV = 3900;

/// Highest request that survives setChargeTargetVoltage()'s map-agnostic guard.
///
/// The two layouts diverge as the field fills up: a standard-map encoding of 4140 mV is 0x90, which
/// reads as 4220 mV under the extended map. So for anything above this, the guard cannot prove the
/// byte is safe if the part's family were misidentified, and refuses the write - safe, but a refusal
/// rather than a cap. Derivation: 4139 mV encodes standard to 0x8C, which decodes extended to exactly
/// 4200 mV; 4140 mV encodes to 0x90, which decodes to 4220 mV.
///
/// The tiers the config layer offers (4000 / 4100 mV) sit below this deliberately. Anyone adding a
/// higher tier must either keep it under this ceiling or drop the map-agnostic guard, having first
/// confirmed on hardware which map the part uses.
inline constexpr uint16_t DUAL_DECODE_SAFE_MAX_MV = 4139;

inline constexpr uint16_t decode(uint8_t reg04, bool extended)
{
    return extended ? (uint16_t)(EXT_OFFSET_MV + EXT_STEP_MV * (uint16_t)((reg04 & EXT_MASK) >> EXT_SHIFT))
                    : (uint16_t)(STD_OFFSET_MV + STD_STEP_MV * (uint16_t)((reg04 & STD_MASK) >> STD_SHIFT));
}

/// Truncating division rounds DOWN, so the programmed target never exceeds the request - the
/// conservative direction for cell life. Callers MUST reject mv < MIN_TARGET_MV first.
inline constexpr uint8_t encode(uint16_t mv, bool extended)
{
    uint16_t n = extended ? (uint16_t)((mv - EXT_OFFSET_MV) / EXT_STEP_MV) : (uint16_t)((mv - STD_OFFSET_MV) / STD_STEP_MV);
    const uint16_t nMax = extended ? EXT_MAX_FIELD : STD_MAX_FIELD; // redundant ceiling, in field units
    if (n > nMax)
        n = nMax;
    return (uint8_t)(n << (extended ? EXT_SHIFT : STD_SHIFT));
}
} // namespace SGM41562Regs
