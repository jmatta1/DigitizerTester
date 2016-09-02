/***************************************************************************//**
********************************************************************************
**
** @file Digitizer.h
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
** @details Implementation file for the Digitizer class
**
********************************************************************************
*******************************************************************************/
#include"Vx1730Digitizer.h"
// includes for C system headers
// includes for C++ system headers
#include<iomanip>
// includes from other libraries
#include<boost/chrono.hpp>
#include<boost/date_time/posix_time/posix_time.hpp>
#include<boost/thread.hpp>
// includes from ORCHID
#include"Vx1730DigitizerRegisters.h"

namespace Digitizer
{
//TODO: Make names associated with the bits we set in registers
//TODO: Maybe remove some of the error handling from CAENComm calls, it might be overkill

enum {MultiRWArraySize = 320, IrqTimeoutMs = 5000};

Vx1730Digitizer::Vx1730Digitizer() :
    moduleNumber(0), channelStartInd(0), numChannel(16), digitizerHandle(0),
    eventsPerInterrupt(0), acqRunning(false), digitizerOpen(false), addrArray(nullptr),
    dataArray(nullptr), rdbkArray(nullptr), cycleErrsArray(nullptr),
    arraySize(MultiRWArraySize), acquisitionCtrlRegBase(0), lg(OrchidLog::get())
{
    //allocate the multiread and multiwrite arrays
    addrArray = new unsigned int[arraySize];
    dataArray = new unsigned int[arraySize];
    rdbkArray = new unsigned int[arraySize];
    cycleErrsArray = new CAENComm_ErrorCode[arraySize];
    for(int i=0; i<arraySize; ++i)
    {
        addrArray[i] = 0;
        dataArray[i] = 0;
        rdbkArray[i] = 0;
        cycleErrsArray[i] = CAENComm_Success;
    }
}

Vx1730Digitizer::~Vx1730Digitizer()
{
    //delete the arrays we allocated
    delete[] addrArray;
    delete[] dataArray;
    delete[] rdbkArray;
    delete[] cycleErrsArray;
}

//log an error and throw an exception to close things
void Vx1730Digitizer::writeErrorAndThrow(CAENComm_ErrorCode errVal)
{
    switch(errVal)
    {
    case CAENComm_Success:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Success\n";
        throw std::runtime_error("Vx1730 Error - Success");
        break;
    case CAENComm_VMEBusError:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: VME Bus Error During Cycle\n";
        throw std::runtime_error("Vx1730 Error - VME Bus Error During Cycle");
        break;
    case CAENComm_CommError:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Communication Error\n";
        throw std::runtime_error("Vx1730 Error - Communication Error");
        break;
    case CAENComm_GenericError:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Unspecified Error\n";
        throw std::runtime_error("Vx1730 Error - Unspecified Error");
        break;
    case CAENComm_InvalidParam:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Invalid Parameter\n";
        throw std::runtime_error("Vx1730 Error - Invalid Parameter");
        break;
    case CAENComm_InvalidLinkType:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Invalid Link Type\n";
        throw std::runtime_error("Vx1730 Error - Invalid Link Type");
        break;
    case CAENComm_InvalidHandler:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Invalid Device Handler\n";
        throw std::runtime_error("Vx1730 Error - Invalid Device Handler");
        break;
    case CAENComm_CommTimeout:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Communication Timeout\n";
        throw std::runtime_error("Vx1730 Error - Communication Timeout");
        break;
    case CAENComm_DeviceNotFound:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Unable To Open Requested Device\n";
        throw std::runtime_error("Vx1730 Error - Unable To Open Requested Device");
        break;
    case CAENComm_MaxDevicesError:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Maximum Number of Devices Exceeded\n";
        throw std::runtime_error("Vx1730 Error - Maximum Number of Devices Exceeded");
        break;
    case CAENComm_DeviceAlreadyOpen:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Device Already Open\n";
        throw std::runtime_error("Vx1730 Error - Device Already Open");
        break;
    case CAENComm_NotSupported:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Not Supported Function\n";
        throw std::runtime_error("Vx1730 Error - Not Supported Function");
        break;
    case CAENComm_UnusedBridge:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: There Are No Boards Controlled By That Bridge\n";
        throw std::runtime_error("Vx1730 Error - There Are No Boards Controlled By That Bridge");
        break;
    case CAENComm_Terminated:
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Digitizer #" << moduleNumber << " - Code: Communication Terminated By Device\n";
        throw std::runtime_error("Vx1730 Error - Communication Terminated By Device");
        break;
    }
}

void Vx1730Digitizer::readDigitizer()
{
    using LowLvl::Vx1730WriteRegisters;
    using LowLvl::Vx1730CommonWriteRegistersAddr;
    
    //open the digitizer
    CAENComm_ErrorCode errVal;
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread: Opening VME Card Via Optical Link, Digitizer #" << moduleNumber;
    errVal = CAENComm_OpenDevice(CAENComm_OpticalLink, 0, 0, 0x00000000,&(this->digitizerHandle));
    
    if (errVal < 0)
    {
        this->writeErrorAndThrow(errVal);
    }
    else
    {
        BOOST_LOG_SEV(lg, Information) << "ACQ Thread: Successfully Opened Digitizer #" << moduleNumber;
        digitizerOpen = true;
    }
    
    //now write all the registers
    this->readCommonRegisterData();
    this->readGroupRegisterData();
    this->readIndividualRegisterData();
    
    CAENComm_CloseDevice(this->digitizerHandle);
}

void Vx1730Digitizer::readCommonRegisterData()
{
    using LowLvl::Vx1730WriteRegisters;
    using LowLvl::Vx1730CommonWriteRegistersAddr;
    int regCount=0;
    //set the components of the address and data arrays
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::BoardConfig>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AggregateOrg>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AcquisitionCtrl>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::GlobalTrgMask>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::TrgOutEnMask>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::FrontIoCtrl>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ChanEnMask>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::SetMonitorDac>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::MonitorDacMode>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::MemBuffAlmtFullLvl>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::RunStrtStpDelay>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::DisableExtTrig>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::DisableExtTrig>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::FrontLvdsIoNew>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::ReadoutCtrl>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::InterruptStatID>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::InterruptEventNum>::value;
    ++regCount;
    addrArray[regCount] = Vx1730CommonWriteRegistersAddr<Vx1730WriteRegisters::AggregateNumPerBlt>::value;
    ++regCount;

    //perform a readback to be certain of integrity
    for(int i=0; i<regCount; ++i)
    {
        rdbkArray[i] = 0x12345678UL;
    }
    CAENComm_ErrorCode overallErr = CAENComm_MultiRead32(this->digitizerHandle,
                                                         addrArray, regCount,
                                                         rdbkArray, cycleErrsArray);
    //test for errors in the individual cycles
    for(int i=0; i<regCount; ++i)
    {
        if(cycleErrsArray[i] < 0)
        {
            BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Error In Read Back From Address: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << std::dec;
            this->writeErrorAndThrow(cycleErrsArray[i]);
        }
    }
    
    //test for an overall error
    if(overallErr < 0)
    {
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Overall Error In Readback Of Common Addresses for Digitizer #" << moduleNumber;
        this->writeErrorAndThrow(overallErr);
    }
    
    //give the readback results
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:Common Register Readback for Digitizer #" << moduleNumber;
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:   Addr |   Read   ";
    for(int i=0; i<regCount; ++i)
    {
        BOOST_LOG_SEV(lg, Information) << "ACQ Thread: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << " | 0x" << std::hex << std::setw(8) << std::setfill('0') << rdbkArray[i] << std::dec;
    }
}

void Vx1730Digitizer::readGroupRegisterData()
{
    using LowLvl::Vx1730WriteRegisters;
    using LowLvl::Vx1730GroupWriteRegistersAddr;
    using LowLvl::Vx1730GroupWriteRegistersOffs;
    int regCount=0;
    for(int i=0; i<this->numChannel; i+=2)
    {
        addrArray[regCount] = (Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::RecordLength>::value +
                               (((i-channelStartInd)/2) * Vx1730GroupWriteRegistersOffs<Vx1730WriteRegisters::RecordLength>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::EventsPerAggregate>::value +
                               (((i-channelStartInd)/2) * Vx1730GroupWriteRegistersOffs<Vx1730WriteRegisters::EventsPerAggregate>::value));
        ++regCount;
        
        addrArray[regCount] = (Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::LocalTrgManage>::value +
                               (((i-channelStartInd)/2) * Vx1730GroupWriteRegistersOffs<Vx1730WriteRegisters::LocalTrgManage>::value));
        ++regCount;
        
        addrArray[regCount] = (Vx1730GroupWriteRegistersAddr<Vx1730WriteRegisters::TriggerValMask>::value +
                               (((i-channelStartInd)/2) * Vx1730GroupWriteRegistersOffs<Vx1730WriteRegisters::TriggerValMask>::value));
        ++regCount;
    }
    
    //perform a readback to be certain of integrity
    for(int i=0; i<regCount; ++i)
    {
        rdbkArray[i] = 0x12345678UL;
    }
    CAENComm_ErrorCode overallErr = CAENComm_MultiRead32(this->digitizerHandle,
                                                         addrArray, regCount,
                                                         rdbkArray, cycleErrsArray);
    //test for errors in the individual cycles
    for(int i=0; i<regCount; ++i)
    {
        if(cycleErrsArray[i] < 0)
        {
            BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Error In Read Back From Address: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << std::dec;
            this->writeErrorAndThrow(cycleErrsArray[i]);
        }
    }
    
    //test for an overall error
    if(overallErr < 0)
    {
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Overall Error In Readback Of Group Addresses for Digitizer #" << moduleNumber;
        this->writeErrorAndThrow(overallErr);
    }
    
    //give the readback results
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:Group Register Readback for Digitizer #" << moduleNumber;
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:   Addr |   Read   ";
    for(int i=0; i<regCount; ++i)
    {
        BOOST_LOG_SEV(lg, Information) << "ACQ Thread: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << " | 0x" << std::hex << std::setw(8) << std::setfill('0') << rdbkArray[i] << std::dec;
    }
}

void Vx1730Digitizer::readIndividualRegisterData()
{
    using LowLvl::Vx1730WriteRegisters;
    using LowLvl::Vx1730IndivWriteRegistersAddr;
    using LowLvl::Vx1730IndivWriteRegistersOffs;
    int regCount=0;
    int stopInd = this->channelStartInd + this->numChannel;
    for(int i=channelStartInd; i<stopInd; ++i)
    {
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::InputDynamicRange>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::InputDynamicRange>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::PreTrg>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::PreTrg>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::CfdSettings>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::CfdSettings>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::ShortGate>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::ShortGate>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::LongGate>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::LongGate>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::GateOffset>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::GateOffset>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::TrgThreshold>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::TrgThreshold>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::FixedBaseline>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::FixedBaseline>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::ShapedTrgWidth>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::ShapedTrgWidth>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::TrgHoldOff>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::TrgHoldOff>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::PsdCutThreshold>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::PsdCutThreshold>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::DppAlgorithmCtrl>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::DppAlgorithmCtrl>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::DcOffset>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::DcOffset>::value));
        ++regCount;
        addrArray[regCount] = (Vx1730IndivWriteRegistersAddr<Vx1730WriteRegisters::VetoExtension>::value +
                               ((i-channelStartInd) * Vx1730IndivWriteRegistersOffs<Vx1730WriteRegisters::VetoExtension>::value));
        ++regCount;
    }
    
    //perform a readback to be certain of integrity
    for(int i=0; i<regCount; ++i)
    {
        rdbkArray[i] = 0x12345678UL;
    }
    CAENComm_ErrorCode overallErr = CAENComm_MultiRead32(this->digitizerHandle,
                                                         addrArray, regCount,
                                                         rdbkArray, cycleErrsArray);
    //test for errors in the individual cycles
    for(int i=0; i<regCount; ++i)
    {
        if(cycleErrsArray[i] < 0)
        {
            BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Error In Read Back From Address: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << std::dec;
            this->writeErrorAndThrow(cycleErrsArray[i]);
        }
    }
    
    //test for an overall error
    if(overallErr < 0)
    {
        BOOST_LOG_SEV(lg, Error) << "ACQ Thread: Overall Error In Readback Of Group Addresses for Digitizer #" << moduleNumber;
        this->writeErrorAndThrow(overallErr);
    }
    
    //give the readback results
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:Individual Register Readback for Digitizer #" << moduleNumber;
    BOOST_LOG_SEV(lg, Information) << "ACQ Thread:   Addr |   Read   ";
    for(int i=0; i<regCount; ++i)
    {
        BOOST_LOG_SEV(lg, Information) << "ACQ Thread: 0x" << std::hex << std::setw(4) << std::setfill('0') << addrArray[i] << " | 0x" << std::hex << std::setw(8) << std::setfill('0') << rdbkArray[i] << std::dec;
    }
}

}
