#pragma once

#include <string>
#include "RCS.h"

namespace RCS
{
class CsvLogging
{
	bool bHeader;
	std::string lastlogstatus;
	std::fstream fs;
	bool bLogonce;
 
public:
	CsvLogging() : bHeader(false), bLogonce(false) {}
	~CsvLogging(){ fs.close(); }
	void Open(std::string filepath) ;
	std::string DumpHeader(std::string separator, size_t jointnum);
	std::string Dump(std::string separator, CanonWorldModel&) ;
	void MotionLogging(CanonWorldModel & status);
};

}
