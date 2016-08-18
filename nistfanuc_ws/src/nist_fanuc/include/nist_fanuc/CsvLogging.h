#pragma once


#include <string>

#include "RCS.h"



class CsvLogging
{
	bool bHeader;
	std::string lastlogstatus;
public:
	CsvLogging() : bHeader(false) {}
	std::string CsvLogging::DumpHeader(std::string separator, size_t jointnum);
	std::string CsvLogging::Dump(std::string separator, CanonWorldModel&) ;
	void MotionLogging(CanonWorldModel & status);
};