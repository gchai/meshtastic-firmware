#include "SGM41562.h"

#ifdef HAS_SGM41562

#include <Arduino.h>

#include "Throttle.h"

SGM41562 *sgm41562 = nullptr;

bool initSGM41562(TwoWire &wire)
{
    if (sgm41562)
        return true;
    sgm41562 = new SGM41562();
    if (!sgm41562->begin(wire)) {
        delete sgm41562;
        sgm41562 = nullptr;
        return false;
    }
    return true;
}

bool SGM41562::readReg(uint8_t reg, uint8_t &value)
{
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0)
        return false;
    if (wire_->requestFrom((int)address_, 1) != 1)
        return false;
    value = wire_->read();
    return true;
}

bool SGM41562::writeReg(uint8_t reg, uint8_t value)
{
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool SGM41562::updateReg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t cur;
    if (!readReg(reg, cur))
        return false;
    cur = (cur & ~mask) | (value & mask);
    return writeReg(reg, cur);
}

bool SGM41562::begin(TwoWire &wire, uint8_t address)
{
    wire_ = &wire;
    address_ = address;
    chipType_ = ChipType::Unknown;

    uint8_t id;
    if (!readReg(REG_DEVICE_ID, id)) {
        LOG_WARN("SGM41562: I2C read failed at 0x%02X", address_);
        return false;
    }

    if (!resetRegisters()) {
        LOG_WARN("SGM41562: register reset failed");
        return false;
    }

    // INVARIANT: REG04's post-reset value is part of the chip fingerprint - detectIdZeroChipType()
    // uses it to tell SGM41562B (0xA3) from SGM41562SA (0x8D), which share device ID 0x00. Sample it
    // here, right after the reset and before detection, and never write REG04 before this point.
    // setChargeTargetVoltage() refuses while reg04Default_ == 0, so a future reorder fails closed.
    // Deliberately NOT fatal: every early return in begin() leaves REG01 CEB = 1 (charging DISABLED,
    // since applyInitSequence()'s REG01 = 0xA4 never runs) and initSGM41562() then deletes the object
    // with no retry. Losing the charge limit is acceptable; losing charging is not.
    if (!readReg(REG_CHARGE_VOLTAGE, reg04Default_)) {
        reg04Default_ = 0;
        LOG_WARN("SGM41562: REG04 reset-value read failed; charge limit unavailable");
    }

    chipType_ = detectChipType(id);
    if (chipType_ == ChipType::Unknown) {
        LOG_WARN("SGM41562: unsupported device ID 0x%02X", id);
        return false;
    }

    if (!applyInitSequence()) {
        LOG_WARN("SGM41562: %s initialization failed", chipTypeName(chipType_));
        chipType_ = ChipType::Unknown;
        return false;
    }

    // REG04 and its decoded voltage are logged because they are the only way to learn which of the
    // two register maps a given board's part actually uses - device ID 0x04 does not distinguish it.
    LOG_INFO("SGM41562: detected %s at 0x%02X (id 0x%02X, REG04=0x%02X, %umV)", chipTypeName(chipType_), address_, id,
             reg04Default_, reg04Default_ ? SGM41562Regs::decode(reg04Default_, hasExtendedRegisterMap()) : 0);
    lastRefreshMs_ = 0;
    return refresh();
}

const char *SGM41562::chipTypeName(ChipType type)
{
    switch (type) {
    case ChipType::SGM41562:
        return "SGM41562";
    case ChipType::SGM41562A:
        return "SGM41562A";
    case ChipType::SGM41562B:
        return "SGM41562B";
    case ChipType::SGM41562S:
        return "SGM41562S";
    case ChipType::SGM41562SA:
        return "SGM41562SA";
    case ChipType::Unknown:
    default:
        return "unknown";
    }
}

bool SGM41562::resetRegisters()
{
    uint8_t value;
    if (!readReg(REG_CHARGE_CURRENT, value))
        return false;
    if (!writeReg(REG_CHARGE_CURRENT, value | 0x80))
        return false;
    delay(10);
    return true;
}

SGM41562::ChipType SGM41562::detectChipType(uint8_t deviceId)
{
    switch (deviceId) {
    case DEVICE_ID_SGM41562:
        return ChipType::SGM41562;
    case DEVICE_ID_SGM41562_A:
        return ChipType::SGM41562A;
    case DEVICE_ID_SGM41562_S:
        return ChipType::SGM41562S;
    case DEVICE_ID_SGM41562_B_SA:
        return detectIdZeroChipType();
    default:
        return ChipType::Unknown;
    }
}

SGM41562::ChipType SGM41562::detectIdZeroChipType()
{
    // reg04Default_ was sampled by begin() immediately after the register reset. Using the cached
    // copy rather than re-reading keeps the fingerprint byte and the byte setChargeTargetVoltage()
    // restores on failure identical by construction, and saves an I2C transaction. A failed sample
    // (0) matches neither family and yields Unknown, exactly as a failed read did before.
    uint8_t systemVoltage;
    if (reg04Default_ == 0 || !readReg(REG_SYS_VOLTAGE_REG, systemVoltage))
        return ChipType::Unknown;

    if (reg04Default_ == SGM41562Regs::STD_RESET && systemVoltage == 0x37)
        return ChipType::SGM41562B;
    if (reg04Default_ == SGM41562Regs::EXT_RESET && systemVoltage == 0x73)
        return ChipType::SGM41562SA;

    LOG_WARN("SGM41562: unknown ID 0x00 reset values (REG04=0x%02X, REG07=0x%02X)", reg04Default_, systemVoltage);
    return ChipType::Unknown;
}

bool SGM41562::applyInitSequence()
{
    if (hasExtendedRegisterMap()) {
        return writeReg(REG_SYS_VOLTAGE_REG, 0x73) && writeReg(REG_MISC_OP_CONTROL, 0x40) &&
               writeReg(REG_CHARGE_TERM_TIMER, 0x1A) && writeReg(REG_SYSTEM_STATUS, 0x40) &&
               writeReg(REG_EXT_INPUT_CURRENT, 0xCA) && writeReg(REG_POWER_ON_CFG, 0xA4);
    }

    return writeReg(REG_SYS_VOLTAGE_REG, 0xB7) && writeReg(REG_MISC_OP_CONTROL, 0x40) && writeReg(REG_CHARGE_TERM_TIMER, 0x1A) &&
           writeReg(REG_SYSTEM_STATUS, 0x40) && writeReg(REG_POWER_ON_CFG, 0xA4);
}

bool SGM41562::hasExtendedRegisterMap() const
{
    return chipType_ == ChipType::SGM41562S || chipType_ == ChipType::SGM41562SA;
}

bool SGM41562::refresh()
{
    uint32_t now = millis();
    if (lastRefreshMs_ != 0 && Throttle::isWithinTimespanMs(lastRefreshMs_, 250))
        return true; // cached
    lastRefreshMs_ = now == 0 ? 1 : now;

    uint8_t status, fault;
    if (!readReg(REG_SYSTEM_STATUS, status))
        return false;
    if (!readReg(REG_FAULT, fault))
        return false;

    chargeStatus_ = static_cast<ChargeStatus>((status >> SYS_STATUS_CHRG_SHIFT) & SYS_STATUS_CHRG_MASK);
    inputPowerGood_ = (status & SYS_STATUS_PG) != 0;
    thermalReg_ = (status & SYS_STATUS_THERM_REG) != 0;
    faultMask_ = fault & 0x3F; // bits [7:6] are enter_ship_time config, not faults

    // REG04 is cleared by REG_RST *and by a watchdog expiry*, which also clears REG01 CEB back to
    // charge-disabled. A valid input force-enables that watchdog until the host writes to the device,
    // so a boot-time-only write can silently revert tens of seconds after USB insertion. A non-zero
    // reg04Target_ means we programmed a cap worth re-checking; it also guarantees
    // lastTargetCheckMs_ is non-zero, so the 0 sentinel never reaches Throttle's arithmetic.
    if (reg04Target_ != 0 && Throttle::hasElapsed(lastTargetCheckMs_, CHARGE_VOLTAGE_RECHECK_MS))
        reapplyAfterRegisterReset();
    return true;
}

bool SGM41562::setChargeTargetVoltage(uint16_t millivolts)
{
    using namespace SGM41562Regs;

    // Proof that neither tier the config layer offers (4000 / 4100 mV) can overcharge under EITHER
    // register map - that is, even if the family were misidentified. 0xA0, the non-extended encoding
    // of 4200 mV, decodes as 4300 mV under the extended map; that is the one dangerous byte, and
    // these asserts plus the dual-decode guard below make it unreachable from the offered tiers.
    static_assert(decode(encode(4000, false), true) <= MAX_TARGET_MV, "4000mV encoded standard overcharges as extended");
    static_assert(decode(encode(4000, true), false) <= MAX_TARGET_MV, "4000mV encoded extended overcharges as standard");
    static_assert(decode(encode(4100, false), true) <= MAX_TARGET_MV, "4100mV encoded standard overcharges as extended");
    static_assert(decode(encode(4100, true), false) <= MAX_TARGET_MV, "4100mV encoded extended overcharges as standard");

    if (chipType_ == ChipType::Unknown || reg04Default_ == 0) {
        LOG_WARN("SGM41562: charge target unavailable, chip not identified");
        return false;
    }

    const bool extended = hasExtendedRegisterMap();

    // Fingerprint precondition: only touch REG04 on a part whose reset value matches the family we
    // believe it to be. Device ID 0x04 has no published datasheet, so this is what makes its layout a
    // verified precondition rather than a guess.
    if (reg04Default_ != (extended ? EXT_RESET : STD_RESET)) {
        LOG_WARN("SGM41562: REG04 default 0x%02X unexpected for %s; leaving charge target alone", reg04Default_,
                 chipTypeName(chipType_));
        return false;
    }

    if (millivolts >= MAX_TARGET_MV) {
        // 4200 mV already IS the OTP default on every variant - issue no write at all.
        reg04Target_ = 0;
        chargeTargetMv_ = 0;
        return true;
    }
    if (millivolts < MIN_TARGET_MV) {
        LOG_WARN("SGM41562: charge target %umV below floor %umV; ignored", millivolts, MIN_TARGET_MV);
        return false;
    }

    const uint8_t value = encode(millivolts, extended);
    // Map-agnostic guard: refuse any byte that reads as an overcharge under either layout, so safety
    // does not depend on having identified the family correctly. Note this refuses rather than caps,
    // and the two layouts diverge above DUAL_DECODE_SAFE_MAX_MV - a request in
    // (DUAL_DECODE_SAFE_MAX_MV, MAX_TARGET_MV) lands here and is dropped. Both offered tiers sit
    // below that ceiling on purpose; see SGM41562Regs.h before adding a higher one.
    if (decode(value, false) > MAX_TARGET_MV || decode(value, true) > MAX_TARGET_MV) {
        LOG_ERROR("SGM41562: refusing REG04=0x%02X for %umV, decodes above %umV under some map", value, millivolts,
                  MAX_TARGET_MV);
        return false;
    }

    // Read-modify-write: the masks preserve VBAT_PRE / VRECH in the low bits.
    if (!updateReg(REG_CHARGE_VOLTAGE, extended ? EXT_MASK : STD_MASK, value)) {
        LOG_WARN("SGM41562: REG04 write failed");
        return false;
    }

    uint8_t back;
    if (!readReg(REG_CHARGE_VOLTAGE, back))
        return false;
    if (decode(back, false) > MAX_TARGET_MV || decode(back, true) > MAX_TARGET_MV) {
        // Fail closed: put the silicon back the way we found it rather than leave a high target.
        writeReg(REG_CHARGE_VOLTAGE, reg04Default_);
        LOG_ERROR("SGM41562: REG04 readback 0x%02X out of range, restored 0x%02X", back, reg04Default_);
        return false;
    }

    reg04Target_ = back;
    chargeTargetMv_ = decode(back, extended);
    lastTargetCheckMs_ = millis() == 0 ? 1 : millis();
    LOG_INFO("SGM41562: charge target %umV (REG04=0x%02X)", chargeTargetMv_, back);
    return true;
}

void SGM41562::reapplyAfterRegisterReset()
{
    // Stamp first and never clear it: refresh() runs from every readPowerStatus(), so a cleared
    // timestamp would turn this recheck into an I2C storm at the 250 ms refresh throttle.
    lastTargetCheckMs_ = millis() == 0 ? 1 : millis();

    uint8_t current;
    if (!readReg(REG_CHARGE_VOLTAGE, current) || current == reg04Target_)
        return;

    LOG_INFO("SGM41562: REG04 reverted to 0x%02X (expected 0x%02X), re-applying", current, reg04Target_);

    // Re-run the FULL init sequence, not REG04 alone: whatever reset REG04 also cleared REG01 CEB, so
    // writing just the cap would faithfully program a limit into a charger that has been switched off.
    const uint16_t wanted = chargeTargetMv_;
    if (!applyInitSequence()) {
        LOG_WARN("SGM41562: re-init after register reset failed");
        return;
    }
    reg04Target_ = 0; // drop the stale byte so setChargeTargetVoltage() re-derives it
    chargeTargetMv_ = 0;
    setChargeTargetVoltage(wanted);
}

bool SGM41562::setChargeEnable(bool enable)
{
    return updateReg(REG_POWER_ON_CFG, POWER_ON_CFG_CHG_DISABLE, enable ? 0x00 : POWER_ON_CFG_CHG_DISABLE);
}

bool SGM41562::setShippingModeEnable(bool enable)
{
    return updateReg(REG_MISC_OP_CONTROL, MISC_OP_SHIPPING_MODE, enable ? MISC_OP_SHIPPING_MODE : 0x00);
}

#endif // HAS_SGM41562
