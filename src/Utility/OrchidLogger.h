/***************************************************************************//**
********************************************************************************
**
** @file OrchidLogger.h
** @author James Till Matta
** @date 03 June, 2016
** @brief
**
** @copyright Copyright (C) 2016 James Till Matta
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
**
** @details Holds the code to use boost::log and to make a global logger
**
********************************************************************************
*******************************************************************************/
#ifndef ORCHID_SRC_UTILITY_ORCHIDLOGGER_H
#define ORCHID_SRC_UTILITY_ORCHIDLOGGER_H

// includes for C system headers
// includes for C++ system headers
// includes from other libraries
#define BOOST_LOG_DYN_LINK 1
#include<boost/log/common.hpp>
#include<boost/log/sinks.hpp>
#include<boost/log/sources/logger.hpp>
#include<boost/log/sources/global_logger_storage.hpp>
// includes from ORCHID

enum LogSeverity
{
    Information = 0,
    Trace       = 1,
    Debug       = 2,
    Warning     = 3,
    Error       = 4,
    Critical    = 5
};

//make a global logger
BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(OrchidLog, boost::log::sources::severity_logger_mt<LogSeverity>)

#endif //ORCHID_SRC_UTILITY_ORCHIDLOGGER_H
