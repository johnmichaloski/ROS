

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <boost/assign.hpp>

#include "nist_fanuc/MotionException.h"
#include "nist_fanuc/Globals.h"
#include "nist_fanuc/Config.h"

using namespace boost::assign;


static int n=-1;
std::map<int, std::string> MotionException::m;
#if 0
 = map_list_of (n--, "Singularity")
                    (n--, "Limit exceeded")
                    (n--, "Parsing Error")
                    (n--, "Conversion Error")
                    (n--, "Bad Parameter");
             (n--, "Not configured");
#endif

void MotionException::Load()
{
	std::string path = Globals._appproperties[ROSPACKAGENAME];
    // Spanish is "es", English is "en" and French is "fr" and German is "de"
    // http://www.science.co.il/Language/Locale-codes.asp
    std::locale l("");
    std::string lang = l.name();
    if (lang.compare(0, std::string("en").length(), std::string("en")) == 0) {
        Nist::Config cfg;
        cfg.load(path + "/config/English.ini");
        m = cfg.gettemplatemap<int, std::string> ("faults");

    }
    
}         
