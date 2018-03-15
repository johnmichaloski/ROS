

#include <ros/ros.h>



#define RANGE = 10000

template<typename T>
T get_param(ros::NodeHandle &nh, std::string name, T t)
{
	// fixme need nh
	std::string localname="~"+name;
	if (nh.hasParam(localname))
	{
		return nh.aram(localname, t);
	}
	else  if (nh.hasParam(name))
	{
		return nh.param(name, t);
	}
	return t;
}


class JointStatePublisher
{
	std::map<std::string, std::map<std::string, double> > free_joints;
	typedef std::map<std::string, std::map<std::string, double> >::iterator jiterator;
	std::vector<std::string> joint_list;
	bool use_mimic;
	bool use_small;
	bool use_gui;
	bool pub_def_positions, pub_def_vels, pub_def_efforts;
	std::vector<std::string> source_list;
	std::vector<ros::Subscriber> sources; /// subscribers to listen to
	ros::Publisher jntpub;
	// wrong needs to be a recursive 
	typedef boost::variant<double, std::map<std::string, double> *> mimic_var;

public:
	JointStatePublisher()
	{
		std::string urdf_xml;
		std::map<std::string, double> joint;
		std::string jtype;

		if (!_nh.getParam("robot_description", urdf_xml)) {
			ROS_FATAL_NAMED("JointStateUpdater", "Could not load the xml from parameter server: %s", urdf_xml.c_str());

		}
		//robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
		free_joints.clear();
		self.joint_list.clear(); // for maintaining the original order of the joints

		if(nh.hasParam("dependent_joints"))
		{
			// array of dependent joints
			// self.dependent_joints = get_param("dependent_joints", {})
		}
		use_mimic = get_param<bool> ("use_mimic_tags", true);
		use_small = get_param<bool>("use_smallest_joint_limits", true);

		//self.zeros = get_param("zeros")

		pub_def_positions = get_param<bool>("publish_default_positions", true);
		pub_def_vels = get_param<bool>("publish_default_velocities", false);
		pub_def_efforts = get_param<bool>("publish_default_efforts", false);
		/*
		# Find all non-fixed joints
		for child in robot.childNodes:
		if child.nodeType is child.TEXT_NODE:
		continue
		if child.localName == 'joint':
		jtype = child.getAttribute('type')
		if jtype == 'fixed' or jtype == 'floating':
		continue
		name = child.getAttribute('name')
		self.joint_list.append(name)
		if jtype == 'continuous':
		minval = -pi
		maxval = pi
		else:
		try:
		limit = child.getElementsByTagName('limit')[0]
		minval = float(limit.getAttribute('lower'))
		maxval = float(limit.getAttribute('upper'))
		except:
		rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
		continue

		safety_tags = child.getElementsByTagName('safety_controller')
		if use_small and len(safety_tags) == 1:
		tag = safety_tags[0]
		if tag.hasAttribute('soft_lower_limit'):
		minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
		if tag.hasAttribute('soft_upper_limit'):
		maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

		mimic_tags = child.getElementsByTagName('mimic')
		if use_mimic and len(mimic_tags) == 1:
		tag = mimic_tags[0]
		entry = {'parent': tag.getAttribute('joint')}
		if tag.hasAttribute('multiplier'):
		entry['factor'] = float(tag.getAttribute('multiplier'))
		if tag.hasAttribute('offset'):
		entry['offset'] = float(tag.getAttribute('offset'))

		self.dependent_joints[name] = entry
		continue

		if name in self.dependent_joints:
		continue

		if self.zeros and name in self.zeros:
		zeroval = self.zeros[name]
		elif minval > 0 or maxval < 0:
		zeroval = (maxval + minval)/2
		else:
		zeroval = 0
		*/
		joint["min"] = minval;
		joint["max"] = maxval;
		joint["zero"] = zeroval;
		if (pub_def_positions)
			joint["position"] = zeroval;
		if (pub_def_vels)
			joint["velocity"] = 0.0;
		if pub_def_efforts:
		joint["effort"] = 0.0;

		if (jtype == "continuous")
			joint["continuous"] = 1.0;
		this->free_joints[name] = joint;

		use_gui = get_param<bool>("use_gui", false);

		if ( use_gui)
		{
			num_rows = get_param<int>("num_rows", 0);
			//self.app = QApplication(sys.argv);
			//self.gui = JointStatePublisherGui("Joint State Publisher", self, num_rows);
			//self.gui.show()
		}
		else
		{
			//self.gui = None;
		}

		nh.getParam("source_list", source_list);
		sources.clear();
		for(size_t i=0; i< source_list.size(); i++)
		{
			// careful if allocated on stack will be deleted
			sources.push_back( rosAdapter->nh->subscribe(source_list[i].c_str(), 10, &JointStatePublisher::callback, this));
		}

		jntpub = nh.advertise<sensor_msgs::JointState>("joint_states", 5);
	}
	void JointStatePublisher::callback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		std::string name;
		for(size_t i=0; i< msg->name.size(); i++)
		{
			name = msg->name[i];
			if(free_joints.count(name) == 0)
				continue;

			std::map<std::string, double> &joint(free_joints[name]);
			if(msg->position.size() >= i)
				joint["position"] = position;
			if(msg->velocity.size() >= i)
				joint["velocity"] = velocity;
			if(msg->effort.size() >= i)
				joint["effort"] = effort;
		}


		if(use_gui)
		{
			// signal instead of directly calling the update_sliders method, to switch to the QThread
			sliderUpdateTrigger.emit()
		}
	}
	void Run()
	{

		double hz = get_param<double> ("rate", 10.); // 10 times a second = 10Hz
		ros::Rate r(hz); 

		// Publish Joint States
		bool bFlag=true;
		while (ros::ok() && bFlag)
		{
			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();

			// Initialize msg.position, msg.velocity, and msg.effort.
			bool has_position = dependent_joints.size() > 0;
			bool has_velocity = false;
			bool has_effort = false;

			for(jiterator jit=free_joints.begin(); jit!=free_joints.end(); jit++)
			{
				if((*jit).second.count("position") >0)
					has_position=true;
				if((*jit).second.count("velocity") >0)
					has_velocity=true;
				if((*jit).second.count("effort") >0)
					has_effort=true;

			}

			size_t num_joints = (len(self.free_joints.items()) +
				len(self.dependent_joints.items()));

			if ( has_position)
				msg.position.resize(num_joints ,0.0);

			if ( has_velocity)
				msg.velocity.resize(num_joints ,0.0);
			if (has_effort)
				msg.effort.resize(num_joints ,0.0);

			for(size_t i=0; i< joint_list.size(); i++)
			{
				std::string name = joint_list[i];
				msg.name[i]=name; // or append?
				std::map<std::string, double> joint;
				double factor=1., offset=0.0;
				// Add Free Joint
				if (free_joints.count(name) !=0)
				{
					joint = free_joints[name]
				}
#if 0
				// Add Dependent Joint
				else if( dependent_joints.count(name)!= 0)
				{
					std::map<std::string, double> param = dependent_joints[name];
					parent = param["parent"];
					factor = param.get("factor", 1.);
					offset = param.get("offset", 0.);

					//Handle recursive mimic chain
					recursive_mimic_chain_joints = [name]
					while parent in self.dependent_joints:
					if parent in recursive_mimic_chain_joints:
					error_message = "Found an infinite recursive mimic chain"
						rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
						sys.exit(-1)
						recursive_mimic_chain_joints.append(parent);
					param = dependent_joints[parent];
					parent = param['parent'];
					offset += factor * param.get('offset', 0);
					factor *= param.get('factor', 1);
					joint = free_joints[parent];
#endif
					if (has_position && joint.count("position"))
						msg.position[i] = joint["position"] * factor + offset;
					if (has_velocity && joint.count("velocity"))
						msg.velocity[i] = joint["velocity"] * factor;
					if (has_effort && joint.count("effort"))
						msg.effort[i] = joint["effort"];
				}
			}
			// Only publish non-empty messages
			if ( msg.name.size() || msg.position || msg.velocity || msg.effort)
				jntpub.publish(msg);
			try{

				r.sleep();
			}
			catch(...) // rospy.exceptions.ROSTimeMovedBackwardsException:
			{
			}
		}
	}


#if 0
	if __name__ == '__main__':
	try:
	rospy.init_node('joint_state_publisher')
		rospy.set_param('/source_list', ['nist_controller/robot/joint_states'])
		rospy.set_param('/use_gui','true')

		jsp = JointStatePublisher()

		if jsp.gui is None:
	jsp.loop()
		else:
	Thread(target=jsp.loop).start()
		signal.signal(signal.SIGINT, signal.SIG_DFL)
		sys.exit(jsp.app.exec_())

		except rospy.ROSInterruptException:
	pass
#endif