
// urdfparser.h

#include "tinyxml.h"
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>


#include <QThread>
#include <QMutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>


class UrdfJointParser: public QThread
{
    Q_OBJECT

public:
    QMutex mMutex;
	TiXmlDocument doc;
	std::vector<std::string> joint_list;
	std::map<std::string, std::map<std::string, std::string> > dependent_joints;
    typedef std::map<std::string, std::map<std::string, double> > MAPLIST;


    typedef std::map<std::string, std::map<std::string, double> >::iterator jiterator;
    double delta;
    bool pub_def_positions, pub_def_vels,pub_def_efforts;
    bool use_gui;
    bool bRunFlag;
    std::map<std::string, std::map<std::string, double> > free_joints;

    ros::NodeHandle &_nh;

    // Publish to joint_state
    ros::Publisher jntpub;

    // Subscribe to sources
    std::vector<std::string> source_list;
    std::vector<ros::Subscriber> sources; /// subscribers to listen to


    UrdfJointParser(ros::NodeHandle &nh, QThread *parent=NULL);

    ~UrdfJointParser();

	std::string getmap(std::map<std::string, std::string> &m, std::string entry, std::string default_val)
	{
		if(m.count(entry)==0)
			return default_val;
		return m[entry];
	}
	template<typename T>
	T Convert(std::string data)
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
    template<typename T>
    T get_param(ros::NodeHandle &nh, std::string name, T t)
    {
        // fixme need nh
        std::string localname="~"+name;
        if (nh.hasParam(localname))
        {
            return nh.param(localname, t);
        }
        else  if (nh.hasParam(name))
        {
            return nh.param(name, t);
        }
        return t;
    }

    // TinyXML routines
    std::map<std::string, std::string> GetAttributes(TiXmlElement* pSubElem);
    std::vector<TiXmlElement *> GetAllElements(std::string name);
    std::vector<TiXmlElement *> getElementsByTagName(TiXmlElement * elem, std::string name);
    std::vector<TiXmlElement *> GetAllChildren(TiXmlElement * elem);
    void GetElements(std::vector<TiXmlElement *> &nodes, TiXmlElement* elem, std::string name);
    std::string GetText(TiXmlElement* pSubElem);
    std::string GetTagName(TiXmlElement* pElem);

    void listenSources();

    /**
     * @brief ParseUrdfJoints parse URDF joint readings given XML string
     * @param urdf_xml string containing XML
     * @return 0 sucess, -1 failure.
     */
    int ParseUrdfJoints(std::string urdf_xml);
    /**
     * @brief updateJointVal mutex update of specific joint measure (position, velocity, effort)
     * @param jointname string name of joint
     * @param measure string value of measure to update
     * @param val double containing value to save
     */
    void updateJointVal(std::string jointname, std::string measure, double val);
    /**
     * @brief update fake updating joint position given a delta offset
     * @param delta double incremental value  to add to joint position values.
     */
    void update(double delta);
    /**
     * @brief Dump return a string containing all free joint values, and all dependent joint values.
     * @return  string with dumped values
     */
    std::string Dump(std::map<std::string, std::map<std::string, double> > &);

    /**
     * @brief updatemsg create a new message from the current joint readings
     * @param msg reference to joint state message to fill
     * @return  1 success, 0 fail.
     */
    bool updatemsg(sensor_msgs::JointState &msg);

    /**
     * @brief callback is the source list subscribed callback method in which to store updated joint values.
     * @param msg the subscribed message that is delived by ROS.
     */
    void callback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief subscribeSources read the ROS param to determine what subscriber sources to listen to.
     */
    void subscribeSources();


    /**
     * @brief run QT threading method.
     */
    void run();
    /**
     * @brief safeCopy mutex copy of the current free joint values.
     * @param copy reference of free joint map
     */
    void safeCopy( std::map<std::string, std::map<std::string, double> >& copy);

signals:
    void resultUpdate(MAPLIST free_joints);
};

