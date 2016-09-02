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
** @details Definition file for the Digitizer class
**
********************************************************************************
*******************************************************************************/
#ifndef ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZER_H
#define ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZER_H
// includes for C system headers
// includes for C++ system headers
// includes from other libraries
#include<CAENComm.h>
// includes from ORCHID
#include"Utility/OrchidLogger.h"

namespace Digitizer
{
//Todo: add some mechanism to reduce the number of individual channel reads
//  while acquiring data, maybe use the number of aggregates to trigger an IRQ
//  as a bare minimum to wait for?
class Vx1730Digitizer
{
public:
    Vx1730Digitizer();
    ~Vx1730Digitizer();
    
    void readDigitizer();

private:
    void writeErrorAndThrow(CAENComm_ErrorCode errVal);
    
    void readCommonRegisterData();
    void readGroupRegisterData();
    void readIndividualRegisterData();
    
    int moduleNumber;
    int channelStartInd;
    int numChannel;
    int digitizerHandle;
    int eventsPerInterrupt;
    bool acqRunning;
    bool digitizerOpen;
    //arrays to handle multireads and multi writes
    unsigned int* addrArray;
    unsigned int* dataArray;
    unsigned int* rdbkArray;
    CAENComm_ErrorCode* cycleErrsArray;
    int arraySize;
    //variables to hold sizes of parts of the readout (in 32 bit ints
    int sizePerEvent[8];
    int sizePerChanPairAggregate[8];
    int maxSizeOfBoardAggregate;
    int maxSizeOfBoardAggregateBlock;
    int maxBufferFillForAnotherRead;
    //variables to hold persistent values for use later
    unsigned int acquisitionCtrlRegBase;
    
    boost::log::sources::severity_logger_mt<LogSeverity>& lg;
    
};

}

#endif //ORCHID_SRC_HARDWARE_DIGITIZER_DIGITIZER_H
