#include "Globals.h"
#include "Config.h"
#include <QFileInfo>
#include <utility>
using namespace Nist;

CGlobals Globals;

/**
 * @brief CGlobals::parse parse a config.ini file for parameterization.
 * Parameters include "section.parameter"
 *   GLOBALS.UpdaterRate
   GLOBALS.Debug

 * @param cfgfile
 * @return
 */
bool CGlobals::parse(std::string cfgfile)
{
    QFileInfo check_file(cfgfile.c_str());
    // check if file exists and if yes: Is it really a file and no directory?
    if (!(check_file.exists() && check_file.isFile())) {
        return false;
    }

    Config config;
    config.load(cfgfile);
    mUpdateRate  = config.GetSymbolValue<int>("GLOBALS.UpdaterRate", "100");
    mDebug   = config.GetSymbolValue<int>("GLOBALS.Debug", "0");


    return true;
}
