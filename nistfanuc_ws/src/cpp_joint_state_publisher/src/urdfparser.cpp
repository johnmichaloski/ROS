
#include "urdfparser.h"
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <cmath>

////////////////////////////////////////////////////////////////////////////


double round(double v, int precision)
{
    double h = std::pow (10., precision);
    long long ll =  (long long)((v * h ) + .5) ;
    return ll / h;
}

/*
 * THis is code that might already be available in tinyxml. Just coded it up, very
 * simplistic.
*/


std::map<std::string, std::string> UrdfJointParser::GetAttributes(TiXmlElement* pSubElem)
{
    char* pszAttrib = NULL;
    std::map<std::string, std::string> attrs;
    //Attribute
    TiXmlAttribute * pAttrib = pSubElem->FirstAttribute();
    while (pAttrib)
    {
        char* pszAttrib = (char*)pAttrib->Name();
        char* pszText = (char*)pAttrib->Value();
        attrs[pszAttrib]=pszText;
        pAttrib = pAttrib->Next();
    }
    return attrs;
}

std::vector<TiXmlElement *> UrdfJointParser::GetAllElements(std::string name)
{
    std::vector<TiXmlElement * > elems;
    TiXmlHandle hDoc(&doc);
    TiXmlElement* elem = hDoc.FirstChildElement().Element();
    GetElements(elems, elem, name);
    return elems;
}

std::vector<TiXmlElement *> UrdfJointParser::getElementsByTagName(TiXmlElement * elem, std::string name)
{
    std::vector<TiXmlElement *> elems;

    // If null return
    if(!elem)
        return elems;

    // Loop through children
    TiXmlHandle hRoot= TiXmlHandle(elem);
    TiXmlElement * pSubElem = hRoot.FirstChildElement().Element();
    while(pSubElem)
    {
        if(!GetTagName(pSubElem).compare( name))
            elems.push_back(pSubElem);
        pSubElem = pSubElem->NextSiblingElement();
    }

    return elems;
}
std::vector<TiXmlElement *> UrdfJointParser::GetAllChildren(TiXmlElement * elem)
{
    std::vector<TiXmlElement *> elems;

    // If null return
    if(!elem)
        return elems;

    // Loop through children
    TiXmlHandle hRoot= TiXmlHandle(elem);
    TiXmlElement * pSubElem = hRoot.FirstChildElement().Element();
    while(pSubElem)
    {
        elems.push_back(pSubElem);
        pSubElem = pSubElem->NextSiblingElement();
    }

    return elems;
}

void UrdfJointParser::GetElements(std::vector<TiXmlElement *> &nodes, TiXmlElement* elem, std::string name)
{
    // If null return
    if(!elem)
        return;

    //Set current node to root node and determine childe node is exist or not
    if(!name.compare(elem->Value()))
        nodes.push_back(elem);

    // Recursively loop through children
    TiXmlHandle hRoot= TiXmlHandle(elem);
    TiXmlElement * pSubElem = hRoot.FirstChildElement().Element();
    while(pSubElem)
    {
        if(!name.compare(pSubElem->Value()))
            nodes.push_back(pSubElem);
        GetElements(nodes, pSubElem, name);
        pSubElem = pSubElem->NextSiblingElement();
    }
}
std::string UrdfJointParser::GetText(TiXmlElement* pSubElem)
{
    char * pszText = (char*) pSubElem->GetText();
    return pszText;
}
std::string UrdfJointParser::GetTagName(TiXmlElement* pElem)
{
    return pElem->Value();
}


////////////////////////////////////////////////////////////////////////////

UrdfJointParser::UrdfJointParser(ros::NodeHandle &nh, QThread *parent)  :
    QThread(parent),
    mMutex(),
    _nh(nh)
{
    pub_def_positions=true;
    pub_def_vels=false;
    pub_def_efforts=false;

    use_gui=true;
    delta=0.0;
}

UrdfJointParser::~UrdfJointParser()
{
}
std::string UrdfJointParser::Dump(std::map<std::string, std::map<std::string, double> > & free)
{

    std::stringstream tmp;
    tmp<< "FREE==========================\n";
    for(jiterator jit=free.begin(); jit!=free.end(); jit++)
    {
        tmp<<(*jit).first<<"\n";
        std::map<std::string, double> &joint = (*jit).second;
        std::map<std::string, double>::iterator mit;
        for(mit=joint.begin(); mit!=joint.end(); mit++)
        {
            tmp<<"\t"<< (*mit).first<<"="<< (*mit).second<<"\n";
        }
    }
    typedef std::map<std::string, std::map<std::string, std::string> >::iterator diterator;
    tmp<< "MIMIC DEPENDENT==========================\n";
    for(diterator jit=dependent_joints.begin(); jit!=dependent_joints.end(); jit++)
    {
        tmp<<(*jit).first<<"\n";
        std::map<std::string, std::string> &joint = (*jit).second;
        std::map<std::string, std::string>::iterator mit;
        for(mit=joint.begin(); mit!=joint.end(); mit++)
        {
            tmp<<"\t"<<  (*mit).first<<"="<< (*mit).second<<"\n";
        }

    }
    return tmp.str();

}

void  UrdfJointParser::safeCopy(std::map<std::string, std::map<std::string, double> > &copy)
{
    QMutexLocker ml(&mMutex);
    copy=free_joints;
    return;
}


bool UrdfJointParser::updatemsg(sensor_msgs::JointState &msg)
{

    std::map<std::string, std::map<std::string, double> > free;
    safeCopy(free);
    size_t num_joints;
    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    msg.effort.clear();

    msg.header.stamp = ros::Time::now();
    std::map<std::string, double>  joint;

    // Initialize msg.position, msg.velocity, and msg.effort.
    bool has_position = true;// dependent_joints.size() > 0;
    bool has_velocity = false;
    bool has_effort = false;

    //   if (delta > 0.0)
    //       update(delta);

    for(jiterator jit=free.begin(); jit != free.end(); jit++)
    {
        std::string name = (*jit).first;
        std::map<std::string, double> & joint = (*jit).second;

        if (!has_position && joint.count("position"))
            has_position = true;
        if (! has_velocity && joint.count("velocity"))
            has_velocity =  true;
        if (! has_effort && joint.count("effort"))
            has_effort =  true;

        num_joints = free.size() + dependent_joints.size();
        if (has_position)
            msg.position.resize( num_joints ,0.0);
        if (has_velocity)
            msg.velocity .resize( num_joints ,0.0);
        if (has_effort)
            msg.effort .resize( num_joints ,0.0);
    }


    for(size_t i=0; i< joint_list.size(); i++)
    {
        std::string name = joint_list[i];
        msg.name.push_back(name);
        double factor, offset;
        // Add Free Joint
        if ( free.count(name))
        {
            joint = free[name];
            factor = 1;
            offset = 0;

        }

        //Add Dependent Joint
        else if (dependent_joints.count(name))
        {
            std::map<std::string, std::string> param = dependent_joints[name];
            std::string parent = param["parent"];

            // All strings easier to deal with in this case than variants
            factor = Convert<double>(getmap(param, "factor", "1"));
            offset = Convert<double>(getmap(param, "offset", "0"));

            // Handle recursive mimic chain
            std::vector<std::string> recursive_mimic_chain_joints;
            recursive_mimic_chain_joints.push_back(name);
            while(dependent_joints.count(parent))
            {
                bool b = std::find(recursive_mimic_chain_joints.begin(), recursive_mimic_chain_joints.end(), parent) != recursive_mimic_chain_joints.end();
                if (b)
                {
                    std::string error_message = "Found an infinite recursive mimic chain";
                    //rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                    //sys.exit(-1);
                    std::cerr << "Yikes!!!!!!!" << error_message;
                    return -1;

                }
                recursive_mimic_chain_joints.push_back(parent);
                param = dependent_joints[parent];
                parent = param["parent"];
                offset += factor * Convert<double>(getmap(param, "offset", "0"));
                factor *= Convert<double>(getmap(param, "factor", "1"));
            }
            // This is a copy
            joint = free[parent];
        }

        if(has_position && joint.count("position"))
            msg.position[i] = round(joint["position"] * factor + offset, 5);
        if(has_velocity && joint.count("velocity"))
            msg.velocity[i] = joint["velocity"] * factor;
        if ( has_effort && joint.count("effort"))
            msg.effort[i] = joint["effort"];
    }
    return 0;

}


void UrdfJointParser::updateJointVal(std::string jointname, std::string measure, double val)
{
    QMutexLocker ml(&mMutex);
    free_joints[jointname][measure]=val;
}
// Parse URDF joint readings given XML string
int UrdfJointParser::ParseUrdfJoints(std::string urdf_xml)
{
    doc.Parse((const char*) urdf_xml.c_str(), 0, TIXML_ENCODING_UTF8);
    TiXmlHandle hDoc(&doc);

    TiXmlElement* robot = hDoc.FirstChildElement().Element();
    if (!robot) return -1;
    std::vector<TiXmlElement *> childNodes= GetAllChildren(robot);
    for(size_t i=0; i< childNodes.size(); i++)
    {
        double minval, maxval, zeroval=0.0;
        std::string name ;
        if(GetTagName(childNodes[i]) == "joint")
        {
            std::string jtype = childNodes[i]->Attribute("type");
            if( (jtype == "fixed") || (jtype == "floating"))
                continue;
            name = childNodes[i]->Attribute("name");
            if(name.empty())
                continue;
            if ( jtype == "continuous")
            {
                minval = -M_PI;
                maxval = M_PI;
            }
            else
            {
                try{
                    TiXmlElement* limit = getElementsByTagName(childNodes[i], "limit")[0];
                    minval = Convert<double>(limit->Attribute("lower"));
                    maxval = Convert<double>(limit->Attribute("upper"));
                    // This is in effect a fixed joint - used in gazebo
                    if(minval==maxval)
                        continue;
                }
                catch(...)
                {
                    //OutputDebugString("%s is not fixed, nor continuous, but limits are not specified!" , name.c_str());
                }
            }
            joint_list.push_back(name);

            //safety_controller element
            std::vector<TiXmlElement* > safety_tags = getElementsByTagName(childNodes[i], "safety_controller");
            if (safety_tags.size() == 1)
            {
                TiXmlElement* tag = safety_tags[0];
                if( tag->Attribute("soft_lower_limit")!=NULL)
                    minval = std::max(minval, Convert<double>(tag->Attribute("soft_lower_limit")));
                if( tag->Attribute("soft_upper_limit")!=NULL)
                    maxval = std::min(maxval, Convert<double>(tag->Attribute("'soft_upper_limit")));
            }

            //mimic joint element
            std::vector<TiXmlElement* > mimic_tags = getElementsByTagName(childNodes[i], "mimic");
            if ( mimic_tags.size() == 1)
            {
                TiXmlElement* tag = mimic_tags[0];
                std::map<std::string, std::string> entry;
                entry["parent"] = tag->Attribute("joint");
                if ( tag->Attribute("multiplier")!=NULL)
                    entry["factor"] = tag->Attribute("multiplier");
                if( tag->Attribute("offset")!=NULL)
                    entry["offset"] = tag->Attribute("offset");

                dependent_joints[name] = entry;
                continue;
            }

            if ( dependent_joints.count(name) >0)
                continue;

            // FIXME: zeroval not correct

            std::map<std::string, double> joint;
            joint["min"] = minval;
            joint["max"] = maxval;
            joint["zero"] = zeroval;
            joint["position"] = 0.0;
            if (pub_def_positions)
                joint["position"] = 0.0;
            if (pub_def_vels)
                joint["velocity"] = 0.0;
            if(pub_def_efforts)
                joint["effort"] = 0.0;

            // In python code ...
            if (jtype == "continuous")
                joint["continuous"] = 1.0;

            free_joints[name] = joint;
        }

    }
    return 0;
}
void UrdfJointParser::update(double delta)
{
    for(jiterator jit=free_joints.begin(); jit!=free_joints.end(); jit++)
    {
        std::map<std::string, double> &joint = (*jit).second;
        double forward = joint.count("forward") > 0 ? joint["forward"] : 1.0;
        if ( forward>0)
            joint["position"] += delta;
        if ( joint["position"] > joint["max"])
        {
            if ( joint.count("continuous") == 0)
                joint["position"] = joint["min"];
            else
            {
                joint["position"] = joint["max"];
                joint["forward"] = -1.0;
            }
        }
        else
        {
            joint["position"] -= delta;
            if ( joint["position"] < joint["min"])
            {
                joint["position"] = joint["min"];
                joint["forward"] = 1.0;
            }
        }
    }
}

void UrdfJointParser::callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::string name;
    qDebug("UrdfJointParser::callback(const sensor_msgs::JointState::ConstPtr& msg)");
    for(size_t i=0; i< msg->name.size(); i++)
    {
        name = msg->name[i];
        if(free_joints.count(name) == 0)
            continue;

        mMutex.lock();
        std::map<std::string, double> &joint(free_joints[name]);
        if(msg->position.size() > i)
            joint["position"] = msg->position[i];
        if(msg->velocity.size() > i)
            joint["velocity"] = msg->velocity[i];
        if(msg->effort.size() > i)
            joint["effort"] = msg->effort[i];
        mMutex.unlock();
    }

    if(use_gui)
    {
        std::map<std::string, std::map<std::string, double> > copy_free_joints;
        safeCopy(copy_free_joints);
        // signal instead of directly calling the update_sliders method, to switch to the QThread
        emit resultUpdate(copy_free_joints);
    }
}
void UrdfJointParser::subscribeSources()
{
    _nh.getParam("source_list", source_list);
    sources.clear();
    for(size_t i=0; i< source_list.size(); i++)
    {
        // careful if allocated on stack will be deleted
        qDebug("Add source %s", source_list[i].c_str());
        sources.push_back( _nh.subscribe("nist_controller/robot/joint_states", 1, &UrdfJointParser::callback, this));
    }
}
void UrdfJointParser::listenSources()
{
    subscribeSources();
   // ros::spin();

    double hz=100;
    ros::Rate r(hz);
    while (ros::ok() && bRunFlag )
    {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        r.sleep();
    }
}

void UrdfJointParser::run()
{
    //double hz = get_param<double> (_nh,"rate", 10.); // 10 times a second = 10
    double hz=10;
    ros::Rate r(hz);
    bRunFlag=true;
    jntpub = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);


    boost::thread thread1( boost::bind( &UrdfJointParser::listenSources, this ) );

    QThread::sleep(2);
    try {
        while (ros::ok() && bRunFlag)
        {

            sensor_msgs::JointState msg;
            updatemsg(msg);

            if ( msg.name.size() ||  msg.position.size() || msg.velocity.size() | msg.effort.size())
            {
                jntpub.publish(msg);
            }

            r.sleep();
        }
    }
    catch(...)
    {
        std::cerr<< "Aborted Joint State UPdater\n";
    }
    thread1.join();
}
