/*******************************************************************************
ORCHID  -  ORnl baCkground at HfIr Daq
A program for directly acquiring data from the array of detectors serving at the
HFIR background monitoring wall.
*******************************************************************************/

// includes for C system headers
// includes for C++ system headers
#include<string>
#include<algorithm>
#include<iostream>
#include<iomanip>
// includes from other libraries
#define BOOST_LOG_DYN_LINK 1
#include<boost/log/utility/setup.hpp>
#include<boost/log/utility/setup/file.hpp>
#include<boost/log/sinks/text_file_backend.hpp>
#include<boost/log/utility/setup/common_attributes.hpp>
#include <boost/core/null_deleter.hpp>
// includes from ORCHID
// ORCHID device objects
#include"Digitizer/Vx1730Digitizer.h"

int main(int argc, char* argv[])
{
    //initialize the logging file
    boost::log::register_simple_formatter_factory< LogSeverity, char >("Severity");
    boost::log::add_file_log(
                boost::log::keywords::file_name = "digitizerTester_%N.log",          //file name format
                boost::log::keywords::rotation_size = (8*1024*1024),        //rotate to a new file every 8 megabytes
                boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),  //or rotate at midnight
                boost::log::keywords::auto_flush = true,
                //boost::log::keywords::format = "[%TimeStamp%]  (%LineID%) <%Severity%>: %Message%");  //give every message a timestamp
                boost::log::keywords::format = "[%TimeStamp%] <%Severity%>: %Message%");  //give every message a timestamp and severity
    //set up to dump to the console as well
    //boost::log::add_console_log();
    boost::log::add_common_attributes();
    
    boost::shared_ptr<std::ostream> coutStream(&std::cout, boost::null_deleter());
    typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend> text_sink;
    boost::shared_ptr<text_sink> coutSink = boost::make_shared<text_sink>();
    coutSink->locked_backend()->add_stream(coutStream);
    boost::log::core::get()->add_sink(coutSink);
    
    boost::log::sources::severity_logger_mt<LogSeverity>& lg = OrchidLog::get();
    BOOST_LOG_SEV(lg, Information)  << "\n\n Starting up!\n" << std::flush;
    
    Digitizer::Vx1730Digitizer* digi = new Digitizer::Vx1730Digitizer();
    digi->readDigitizer();
    
    BOOST_LOG_SEV(lg, Information)  << "\nORCHID has successfully shut down, have a nice day! :-)\n\n" << std::flush;

    return 0;
}

