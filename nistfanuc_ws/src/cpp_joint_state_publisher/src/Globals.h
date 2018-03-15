#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>


#define ROSPACKAGENAME "cpp_joint_state_publisher"

class CGlobals
{
public:
    CGlobals()
    {
#ifdef WIN32
        mPathSeparator='\\';
#else
        mPathSeparator='/';
#endif

    }
    bool parse(std::string inifilename);
    template<typename T>
    T convert ( std::string data) const
    {
        T result;
        try {
            std::istringstream stream(data);

            if ( stream >> result )
            {
                return result;
            }
        }
        catch(...)
        {
        }
        return T();
    }

    static const int FATAL=0;
    static const int ERROR=1;
    static const int WARNING=2;
    static const int INFO=3;
    static const int DEBUG=40;

    std::string ExeDirectory;
    char mPathSeparator;
    int mUpdateRate; // in milliseconds
    int mDebug;


};
extern CGlobals Globals;

#endif // GLOBALS_H
