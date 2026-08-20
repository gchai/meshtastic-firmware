// Unit tests for the SGM41562 REG04 charge-voltage encoding in src/power/SGM41562Regs.h.
//
// This is the one piece of arithmetic in the charger driver that can overcharge a Li-ion cell, and no
// native, coverage or pr-level build environment defines HAS_SGM41562 - so without this suite it
// would ship exercised only on the single nRF52 board that carries the part. SGM41562Regs.h sits
// outside the HAS_SGM41562 guard specifically so it can be included here.
//
// The two register maps are trusted because they cross-validate: the post-reset bytes the driver
// already fingerprints on (0xA3, 0x8D) each decode to exactly 4200 mV under their own formula.
#include "Arduino.h"
#include "TestUtil.h"
#include "power/SGM41562Regs.h"
#include <unity.h>

using namespace SGM41562Regs;

// The only two targets the config layer will ever ask for.
static constexpr uint16_t TIER_80_PCT_MV = 4000;
static constexpr uint16_t TIER_90_PCT_MV = 4100;

void setUp(void) {}
void tearDown(void) {}

// --- The two maps cross-validate against the fingerprint bytes ---

void test_reset_values_decode_to_the_ceiling_under_their_own_map()
{
    // If either of these ever fails, the corresponding layout is wrong and no write may be trusted.
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, decode(STD_RESET, false));
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, decode(EXT_RESET, true));
}

void test_max_field_lands_exactly_on_the_ceiling()
{
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, (uint16_t)(STD_OFFSET_MV + STD_STEP_MV * STD_MAX_FIELD));
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, (uint16_t)(EXT_OFFSET_MV + EXT_STEP_MV * EXT_MAX_FIELD));
}

// --- Quantisation always rounds down ---

void test_standard_map_rounds_down()
{
    // 15 mV steps from 3600 mV: both tiers are off-grid and must land below, never above.
    TEST_ASSERT_EQUAL_UINT16(3990, decode(encode(TIER_80_PCT_MV, false), false));
    TEST_ASSERT_EQUAL_UINT16(4095, decode(encode(TIER_90_PCT_MV, false), false));
}

void test_extended_map_is_exact_on_both_tiers()
{
    // 10 mV steps from 3500 mV: both tiers land exactly.
    TEST_ASSERT_EQUAL_UINT16(TIER_80_PCT_MV, decode(encode(TIER_80_PCT_MV, true), true));
    TEST_ASSERT_EQUAL_UINT16(TIER_90_PCT_MV, decode(encode(TIER_90_PCT_MV, true), true));
}

void test_never_rounds_above_the_request()
{
    // Exhaustive over the whole legal input range, both maps. Rounding up would program a higher
    // voltage than the user asked for, which is the direction that matters.
    for (uint16_t mv = MIN_TARGET_MV; mv < MAX_TARGET_MV; mv++) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(mv, decode(encode(mv, false), false));
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(mv, decode(encode(mv, true), true));
    }
}

void test_encode_saturates_at_the_ceiling()
{
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(MAX_TARGET_MV, false), false));
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(MAX_TARGET_MV, true), true));
}

// --- The dangerous byte, and why no tier can reach it ---

void test_the_standard_4200_byte_is_an_overcharge_under_the_extended_map()
{
    // 0xA0 encodes 4200 mV non-extended but reads as 4300 mV extended. If a part's family were ever
    // misidentified, this is the byte that would overcharge the cell - so the driver's dual-decode
    // guard rejects it, and this test pins down why that guard exists.
    TEST_ASSERT_EQUAL_UINT16(4200, decode(0xA0, false));
    TEST_ASSERT_EQUAL_UINT16(4300, decode(0xA0, true));
    TEST_ASSERT_TRUE(decode(0xA0, true) > MAX_TARGET_MV);
}

void test_offered_tiers_are_safe_under_either_map()
{
    // Mirrors the static_asserts in SGM41562::setChargeTargetVoltage(): whichever map the part
    // actually uses, and whichever we guessed, neither tier can program above the ceiling.
    const uint16_t tiers[] = {TIER_80_PCT_MV, TIER_90_PCT_MV};
    for (uint16_t mv : tiers) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, false), false));
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, false), true));
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, true), false));
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, true), true));
        TEST_ASSERT_TRUE(encode(mv, false) != 0xA0);
        TEST_ASSERT_TRUE(encode(mv, true) != 0xA0);
    }
}

void test_cross_map_safety_holds_up_to_the_documented_ceiling()
{
    // The guarantee is bounded, not universal: it holds up to DUAL_DECODE_SAFE_MAX_MV and no further,
    // because the two layouts diverge as the field fills up. Pin both sides of that boundary so a
    // future tier cannot quietly cross it - above the ceiling the driver refuses the write.
    for (uint16_t mv = MIN_TARGET_MV; mv <= DUAL_DECODE_SAFE_MAX_MV; mv++) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, false), true));
        TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(mv, true), false));
    }
    // Exactly at the ceiling the standard encoding still reads as the ceiling under the extended map.
    TEST_ASSERT_EQUAL_UINT16(MAX_TARGET_MV, decode(encode(DUAL_DECODE_SAFE_MAX_MV, false), true));
    // One millivolt higher it does not, which is what the ceiling is recording.
    TEST_ASSERT_TRUE(decode(encode((uint16_t)(DUAL_DECODE_SAFE_MAX_MV + 1), false), true) > MAX_TARGET_MV);
}

void test_both_tiers_sit_below_the_dual_decode_ceiling()
{
    // The reason the driver never has to refuse a tier it was asked for.
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(DUAL_DECODE_SAFE_MAX_MV, TIER_80_PCT_MV);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(DUAL_DECODE_SAFE_MAX_MV, TIER_90_PCT_MV);
}

// --- The masks leave the adjacent register fields alone ---

void test_masks_preserve_the_neighbouring_fields()
{
    // Standard map keeps D[1] VBAT_PRE and D[0] VRECH; extended keeps D[0] VRECH only. An encoded
    // value must never carry bits into that space, or the read-modify-write would clobber them.
    TEST_ASSERT_EQUAL_UINT8(0x03, (uint8_t)~STD_MASK);
    TEST_ASSERT_EQUAL_UINT8(0x01, (uint8_t)~EXT_MASK);
    TEST_ASSERT_EQUAL_UINT8(0x00, (uint8_t)(encode(TIER_80_PCT_MV, false) & (uint8_t)~STD_MASK));
    TEST_ASSERT_EQUAL_UINT8(0x00, (uint8_t)(encode(TIER_90_PCT_MV, false) & (uint8_t)~STD_MASK));
    TEST_ASSERT_EQUAL_UINT8(0x00, (uint8_t)(encode(TIER_80_PCT_MV, true) & (uint8_t)~EXT_MASK));
    TEST_ASSERT_EQUAL_UINT8(0x00, (uint8_t)(encode(TIER_90_PCT_MV, true) & (uint8_t)~EXT_MASK));
}

void test_the_floor_is_above_both_map_offsets()
{
    // MIN_TARGET_MV is load-bearing: encode() subtracts the offset without a guard, so a request
    // below it would underflow. Both offsets must sit below the floor for that contract to hold.
    TEST_ASSERT_TRUE(MIN_TARGET_MV > STD_OFFSET_MV);
    TEST_ASSERT_TRUE(MIN_TARGET_MV > EXT_OFFSET_MV);
}

// --- Unity lifecycle ---

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_reset_values_decode_to_the_ceiling_under_their_own_map);
    RUN_TEST(test_max_field_lands_exactly_on_the_ceiling);
    RUN_TEST(test_standard_map_rounds_down);
    RUN_TEST(test_extended_map_is_exact_on_both_tiers);
    RUN_TEST(test_never_rounds_above_the_request);
    RUN_TEST(test_encode_saturates_at_the_ceiling);
    RUN_TEST(test_the_standard_4200_byte_is_an_overcharge_under_the_extended_map);
    RUN_TEST(test_offered_tiers_are_safe_under_either_map);
    RUN_TEST(test_cross_map_safety_holds_up_to_the_documented_ceiling);
    RUN_TEST(test_both_tiers_sit_below_the_dual_decode_ceiling);
    RUN_TEST(test_masks_preserve_the_neighbouring_fields);
    RUN_TEST(test_the_floor_is_above_both_map_offsets);
    exit(UNITY_END());
}

void loop() {}
