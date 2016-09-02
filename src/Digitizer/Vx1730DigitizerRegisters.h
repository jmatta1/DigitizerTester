/***************************************************************************//**
********************************************************************************
**
** @file DigitizerRegisters.h
** @author James Till Matta
** @date 27 June, 2016
** @brief
**
** @copyright Copyright (C) 2016 James Till Matta
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
**
** @details Holds enumerations for the DPP_PSD registers
**
********************************************************************************
*******************************************************************************/
#ifndef ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZERREGISTERS_H
#define ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZERREGISTERS_H

// includes for C system headers
// includes for C++ system headers
#include<map>
#include<utility>
#include<type_traits>
// includes from other libraries
#include<boost/assign/list_of.hpp>
using boost::assign::map_list_of;
// includes from ORCHID

namespace Digitizer
{

namespace LowLvl
{

typedef unsigned short ushort;

enum class Vx1730ReadRegisters : unsigned short
  { RecordLength,       InputDynamicRange,  EventsPerAggregate, PreTrg,
    CfdSettings,        ForcedDataFlush,    ShortGate,          LongGate,
    GateOffset,         TrgThreshold,       FixedBaseline,      ShapedTrgWidth,
    TrgHoldOff,         PsdCutThreshold,    DppAlgorithmCtrl,   LocalTrgManage,
    ChannelStatus,      AmcFirmwareRev,     DcOffset,           AdcTemperature,
    IndivSoftwTrig,     VetoExtension,      BoardConfig,        AggregateOrg,
    ChannelCal,         AcquisitionCtrl,    AcqusitionStatus,   GlobalTrgMask,
    TrgOutEnMask,       LvdsData,           FrontIoCtrl,        ChanEnMask,
    RocFrmwRev,         SetMonitorDac,      BoardInfo,          MonitorDacMode,
    EventSize,          TimeBombCount,      FanSpeedCtrl,       MemBuffAlmtFullLvl,
    RunStrtStpDelay,    BoardFailStatus,    DisableExtTrig,     TriggerValMask,
    FrontLvdsIoNew,     ChannelShutdown,    ReadoutCtrl,        ReadoutStatus,
    BoardId,            McstBaseAddrCtrl,   RelocationAddr,     InterruptStatID,
    InterruptEventNum,  AggregateNumPerBlt, Scratch,            ConfigRomChcksm,
    ConfigRomChcksmL2,  ConfigRomChcksmL1,  ConfigRomChcksmL0,  ConfigRomConst2,
    ConfigRomConst1,    ConfigRomConst0,    ConfigRomCcode,     ConfigRomRcode,
    ConfigRomManuId2,   ConfigRomManuId1,   ConfigRomManuId0,   ConfigRomBrdVer,
    ConfigRomBrdFf,     ConfigRomBrdNum1,   ConfigRomBrdNum0,   ConfigRomRevis3,
    ConfigRomRevis2,    ConfigRomRevis1,    ConfigRomPcbRev,    ConfigRomFlshTp,
    ConfigRomBrdSn1,    ConfigRomBrdSn0,    ConfigRomVxcoTp,    EventReadout};

template<Vx1730ReadRegisters> struct Vx1730CoupleReadRegistersAddr;
//the offsets are almost always 0x0200 for the group write registers
template<Vx1730ReadRegisters> struct Vx1730CoupleReadRegistersOffs { static constexpr ushort value = 0x0200;};
//except in the case of TriggerValMask, so specialize the template for that one case
template<> struct Vx1730CoupleReadRegistersOffs<Vx1730ReadRegisters::TriggerValMask> { static constexpr ushort value = 0x0004;};
//now define addresses
template<> struct Vx1730CoupleReadRegistersAddr<Vx1730ReadRegisters::RecordLength> : std::integral_constant<ushort, 0x1020>{};
template<> struct Vx1730CoupleReadRegistersAddr<Vx1730ReadRegisters::EventsPerAggregate> : std::integral_constant<ushort, 0x1034>{};
template<> struct Vx1730CoupleReadRegistersAddr<Vx1730ReadRegisters::LocalTrgManage> : std::integral_constant<ushort, 0x1084>{};
template<> struct Vx1730CoupleReadRegistersAddr<Vx1730ReadRegisters::TriggerValMask> : std::integral_constant<ushort, 0x8180>{};


template<Vx1730ReadRegisters> struct Vx1730CommonReadRegistersAddr;
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::BoardConfig>           : std::integral_constant<ushort, 0x8000> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::AggregateOrg>          : std::integral_constant<ushort, 0x800C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ChannelCal>            : std::integral_constant<ushort, 0x809C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::AcquisitionCtrl>       : std::integral_constant<ushort, 0x8100> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::AcqusitionStatus>      : std::integral_constant<ushort, 0x8104> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::GlobalTrgMask>         : std::integral_constant<ushort, 0x810C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::TrgOutEnMask>          : std::integral_constant<ushort, 0x8110> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::LvdsData>              : std::integral_constant<ushort, 0x8118> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::FrontIoCtrl>           : std::integral_constant<ushort, 0x811C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ChanEnMask>            : std::integral_constant<ushort, 0x8120> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::RocFrmwRev>            : std::integral_constant<ushort, 0x8124> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::SetMonitorDac>         : std::integral_constant<ushort, 0x8138> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::BoardInfo>             : std::integral_constant<ushort, 0x8140> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::MonitorDacMode>        : std::integral_constant<ushort, 0x8144> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::EventSize>             : std::integral_constant<ushort, 0x814C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::TimeBombCount>         : std::integral_constant<ushort, 0x8158> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::FanSpeedCtrl>          : std::integral_constant<ushort, 0x8168> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::MemBuffAlmtFullLvl>    : std::integral_constant<ushort, 0x816C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::RunStrtStpDelay>       : std::integral_constant<ushort, 0x8170> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::BoardFailStatus>       : std::integral_constant<ushort, 0x8178> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::DisableExtTrig>        : std::integral_constant<ushort, 0x817C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::FrontLvdsIoNew>        : std::integral_constant<ushort, 0x81A0> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ChannelShutdown>       : std::integral_constant<ushort, 0x81C0> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ReadoutCtrl>           : std::integral_constant<ushort, 0xEF00> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ReadoutStatus>         : std::integral_constant<ushort, 0xEF04> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::BoardId>               : std::integral_constant<ushort, 0xEF08> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::McstBaseAddrCtrl>      : std::integral_constant<ushort, 0xEF0C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::RelocationAddr>        : std::integral_constant<ushort, 0xEF10> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::InterruptStatID>       : std::integral_constant<ushort, 0xEF14> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::InterruptEventNum>     : std::integral_constant<ushort, 0xEF18> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::AggregateNumPerBlt>    : std::integral_constant<ushort, 0xEF1C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::Scratch>               : std::integral_constant<ushort, 0xEF20> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomChcksm>       : std::integral_constant<ushort, 0xF000> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomChcksmL2>     : std::integral_constant<ushort, 0xF004> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomChcksmL1>     : std::integral_constant<ushort, 0xF008> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomChcksmL0>     : std::integral_constant<ushort, 0xF00C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomConst2>       : std::integral_constant<ushort, 0xF010> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomConst1>       : std::integral_constant<ushort, 0xF014> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomConst0>       : std::integral_constant<ushort, 0xF018> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomCcode>        : std::integral_constant<ushort, 0xF01C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomRcode>        : std::integral_constant<ushort, 0xF020> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomManuId2>      : std::integral_constant<ushort, 0xF024> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomManuId1>      : std::integral_constant<ushort, 0xF028> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomManuId0>      : std::integral_constant<ushort, 0xF02C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdVer>       : std::integral_constant<ushort, 0xF030> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdFf>        : std::integral_constant<ushort, 0xF034> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdNum1>      : std::integral_constant<ushort, 0xF038> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdNum0>      : std::integral_constant<ushort, 0xF03C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomRevis3>       : std::integral_constant<ushort, 0xF040> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomRevis2>       : std::integral_constant<ushort, 0xF044> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomRevis1>       : std::integral_constant<ushort, 0xF048> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomPcbRev>       : std::integral_constant<ushort, 0xF04C> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomFlshTp>       : std::integral_constant<ushort, 0xF050> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdSn1>       : std::integral_constant<ushort, 0xF080> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomBrdSn0>       : std::integral_constant<ushort, 0xF084> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::ConfigRomVxcoTp>       : std::integral_constant<ushort, 0xF088> {};
template<> struct Vx1730CommonReadRegistersAddr<Vx1730ReadRegisters::EventReadout>          : std::integral_constant<ushort, 0x0000> {};


template<Vx1730ReadRegisters> struct Vx1730IndivReadRegistersAddr;
//the offsets are always 0x0100
template<Vx1730ReadRegisters> struct Vx1730IndivReadRegistersOffs { static constexpr ushort value = 0x0100;};
//now define addresses
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::InputDynamicRange>  : std::integral_constant<ushort, 0x1028> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::PreTrg>             : std::integral_constant<ushort, 0x1038> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::CfdSettings>        : std::integral_constant<ushort, 0x103C> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::ForcedDataFlush>    : std::integral_constant<ushort, 0x1040> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::ShortGate>          : std::integral_constant<ushort, 0x1054> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::LongGate>           : std::integral_constant<ushort, 0x1058> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::GateOffset>         : std::integral_constant<ushort, 0x105C> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::TrgThreshold>       : std::integral_constant<ushort, 0x1060> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::FixedBaseline>      : std::integral_constant<ushort, 0x1064> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::ShapedTrgWidth>     : std::integral_constant<ushort, 0x1070> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::TrgHoldOff>         : std::integral_constant<ushort, 0x1074> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::PsdCutThreshold>    : std::integral_constant<ushort, 0x1078> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::DppAlgorithmCtrl>   : std::integral_constant<ushort, 0x1080> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::ChannelStatus>      : std::integral_constant<ushort, 0x1088> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::AmcFirmwareRev>     : std::integral_constant<ushort, 0x108C> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::DcOffset>           : std::integral_constant<ushort, 0x1098> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::AdcTemperature>     : std::integral_constant<ushort, 0x10A8> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::IndivSoftwTrig>     : std::integral_constant<ushort, 0x10C0> {};
template<> struct Vx1730IndivReadRegistersAddr<Vx1730ReadRegisters::VetoExtension>      : std::integral_constant<ushort, 0x10D4> {};


enum class Vx1730WriteRegisters : unsigned short
  { RecordLength,       InputDynamicRange,  EventsPerAggregate, PreTrg,
    CfdSettings,        ForcedDataFlush,    ShortGate,          LongGate,
    GateOffset,         TrgThreshold,       FixedBaseline,      ShapedTrgWidth,
    TrgHoldOff,         PsdCutThreshold,    DppAlgorithmCtrl,   LocalTrgManage,
    DcOffset,           IndivSoftwTrig,     VetoExtension,      BoardConfig,
    AggregateOrg,       ChannelCal,         AcquisitionCtrl,    SoftwareTrg,
    GlobalTrgMask,      TrgOutEnMask,       LvdsData,           FrontIoCtrl,
    ChanEnMask,         SetMonitorDac,      SoftwClckSync,      MonitorDacMode,
    FanSpeedCtrl,       MemBuffAlmtFullLvl, RunStrtStpDelay,    DisableExtTrig,
    TriggerValMask,     FrontLvdsIoNew,     ChannelShutdown,    ReadoutCtrl,
    BoardId,            McstBaseAddrCtrl,   RelocationAddr,     InterruptStatID,
    InterruptEventNum,  AggregateNumPerBlt, Scratch,            SoftwReset,
    SoftwClear,         ConfigReload};

template<Vx1730WriteRegisters> struct Vx1730CommonWriteRegistersAddr;
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::BoardConfig>         : std::integral_constant<ushort, 0x8000> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AggregateOrg>        : std::integral_constant<ushort, 0x800C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ChannelCal>          : std::integral_constant<ushort, 0x809C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AcquisitionCtrl>     : std::integral_constant<ushort, 0x8100> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SoftwareTrg>         : std::integral_constant<ushort, 0x8108> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::GlobalTrgMask>       : std::integral_constant<ushort, 0x810C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::TrgOutEnMask>        : std::integral_constant<ushort, 0x8110> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::LvdsData>            : std::integral_constant<ushort, 0x8118> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::FrontIoCtrl>         : std::integral_constant<ushort, 0x811C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ChanEnMask>          : std::integral_constant<ushort, 0x8120> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SetMonitorDac>       : std::integral_constant<ushort, 0x8138> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SoftwClckSync>       : std::integral_constant<ushort, 0x813C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::MonitorDacMode>      : std::integral_constant<ushort, 0x8144> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::FanSpeedCtrl>        : std::integral_constant<ushort, 0x8168> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::MemBuffAlmtFullLvl>  : std::integral_constant<ushort, 0x816C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::RunStrtStpDelay>     : std::integral_constant<ushort, 0x8170> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::DisableExtTrig>      : std::integral_constant<ushort, 0x817C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::FrontLvdsIoNew>      : std::integral_constant<ushort, 0x81A0> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ChannelShutdown>     : std::integral_constant<ushort, 0x81C0> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ReadoutCtrl>         : std::integral_constant<ushort, 0xEF00> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::BoardId>             : std::integral_constant<ushort, 0xEF08> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::McstBaseAddrCtrl>    : std::integral_constant<ushort, 0xEF0C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::RelocationAddr>      : std::integral_constant<ushort, 0xEF10> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::InterruptStatID>     : std::integral_constant<ushort, 0xEF14> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::InterruptEventNum>   : std::integral_constant<ushort, 0xEF18> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AggregateNumPerBlt>  : std::integral_constant<ushort, 0xEF1C> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::Scratch>             : std::integral_constant<ushort, 0xEF20> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SoftwReset>          : std::integral_constant<ushort, 0xEF24> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SoftwClear>          : std::integral_constant<ushort, 0xEF28> {};
template<> struct Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ConfigReload>        : std::integral_constant<ushort, 0xEF34> {};


template<Vx1730WriteRegisters> struct Vx1730IndivWriteRegistersAddr;
//the offsets are always 0x0100
template<Vx1730WriteRegisters> struct Vx1730IndivWriteRegistersOffs { static constexpr ushort value = 0x0100;};
//now define addresses
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::InputDynamicRange>   : std::integral_constant<ushort, 0x1028> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::PreTrg>              : std::integral_constant<ushort, 0x1038> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::CfdSettings>         : std::integral_constant<ushort, 0x103C> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::ForcedDataFlush>     : std::integral_constant<ushort, 0x1040> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::ShortGate>           : std::integral_constant<ushort, 0x1054> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::LongGate>            : std::integral_constant<ushort, 0x1058> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::GateOffset>          : std::integral_constant<ushort, 0x105C> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::TrgThreshold>        : std::integral_constant<ushort, 0x1060> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::FixedBaseline>       : std::integral_constant<ushort, 0x1064> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::ShapedTrgWidth>      : std::integral_constant<ushort, 0x1070> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::TrgHoldOff>          : std::integral_constant<ushort, 0x1074> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::PsdCutThreshold>     : std::integral_constant<ushort, 0x1078> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::DppAlgorithmCtrl>    : std::integral_constant<ushort, 0x1080> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::DcOffset>            : std::integral_constant<ushort, 0x1098> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::IndivSoftwTrig>      : std::integral_constant<ushort, 0x10C0> {};
template<> struct Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::VetoExtension>       : std::integral_constant<ushort, 0x10D4> {};


template<Vx1730WriteRegisters> struct Vx1730IbcastWriteRegistersAddr;
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::InputDynamicRange>  : std::integral_constant<ushort, 0x8028> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::PreTrg>             : std::integral_constant<ushort, 0x8038> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::CfdSettings>        : std::integral_constant<ushort, 0x803C> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::ForcedDataFlush>    : std::integral_constant<ushort, 0x8040> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::ShortGate>          : std::integral_constant<ushort, 0x8054> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::LongGate>           : std::integral_constant<ushort, 0x8058> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::GateOffset>         : std::integral_constant<ushort, 0x805C> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::TrgThreshold>       : std::integral_constant<ushort, 0x8060> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::FixedBaseline>      : std::integral_constant<ushort, 0x8064> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::ShapedTrgWidth>     : std::integral_constant<ushort, 0x8070> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::TrgHoldOff>         : std::integral_constant<ushort, 0x8074> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::PsdCutThreshold>    : std::integral_constant<ushort, 0x8078> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::DppAlgorithmCtrl>   : std::integral_constant<ushort, 0x8080> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::DcOffset>           : std::integral_constant<ushort, 0x8098> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::IndivSoftwTrig>     : std::integral_constant<ushort, 0x80C0> {};
template<> struct Vx1730IbcastWriteRegistersAddr<Vx1730WriteRegisters::VetoExtension>      : std::integral_constant<ushort, 0x80D4> {};


template<Vx1730WriteRegisters> struct Vx1730GroupWriteRegistersAddr;
//the offsets are almost always 0x0200 for the group write registers
template<Vx1730WriteRegisters> struct Vx1730GroupWriteRegistersOffs { static constexpr ushort value = 0x0200;};
//except in the case of TriggerValMask, so specialize the template for that one case
template<> struct Vx1730GroupWriteRegistersOffs<Vx1730WriteRegisters::TriggerValMask> { static constexpr ushort value = 0x0004;}; 
//now define addresses
template<> struct Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::RecordLength>        : std::integral_constant<ushort, 0x1020> {};
template<> struct Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::EventsPerAggregate>  : std::integral_constant<ushort, 0x1034> {};
template<> struct Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::LocalTrgManage>      : std::integral_constant<ushort, 0x1084> {};
template<> struct Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::TriggerValMask>      : std::integral_constant<ushort, 0x8180> {};


template<Vx1730WriteRegisters> struct Vx1730GbcastWriteRegistersAddr;

template<> struct Vx1730GbcastWriteRegistersAddr<Vx1730WriteRegisters::RecordLength>        : std::integral_constant<ushort, 0x8020> {};
template<> struct Vx1730GbcastWriteRegistersAddr<Vx1730WriteRegisters::EventsPerAggregate>  : std::integral_constant<ushort, 0x8034> {};
template<> struct Vx1730GbcastWriteRegistersAddr<Vx1730WriteRegisters::LocalTrgManage>      : std::integral_constant<ushort, 0x8084> {};

}

}

#endif //ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZERREGISTERS_H
