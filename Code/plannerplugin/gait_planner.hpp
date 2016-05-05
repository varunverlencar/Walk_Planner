/*Authors: Varun Verlencar
Worcester Polytechnic Insitute
Last update: April 23, 2016

Gait Planner
*/

#pragma once

#include <vector>
#include <algorithm>

const double pi = 3.14;
bool goalflag = false;

///////////////////// RRTNode class with Node configuration and its parent /////////////////////

class RRTNode {
private:
	std::vector<float> _config;
	RRTNode* _parent;

public:
	RRTNode(const std::vector<float> &q, RRTNode* prevnode = NULL){
		_config.assign(q.begin(),q.end());
		this->_parent = prevnode;
	}

	RRTNode(RRTNode* Node)	{
		_config.assign(Node->getConfig().begin(),Node->getConfig().end());
		this->_parent = Node->getParent();
	}

	RRTNode(){}
	
	virtual ~RRTNode() {
		// delete _config;
		// delete _parent;
	}

	RRTNode* getParent() {return _parent;}

	const std::vector<float> &getConfig() {return _config;}

};

///////////////////// NodeTree class with all Nodes /////////////////////

class NodeTree {
private:
	std::vector<RRTNode*> _nodes;
	std::vector<float> _goalconfig;
	float _stepSize;
	float _goalBias;
	int _maxIterations;
	
	float _goal_threshold;
	bool _goalflag;

public:
	unsigned int _numNodes;

	NodeTree()
	{
		//default values
		// _numNodes = 1;
		// setStepSize(0.01);
		// setGoalBias(0.26); // value between 0 and 1
		setMaxIterations(10000);
		// setgoalthreshold(1);
	}

	NodeTree(RRTNode* start)
	{
		_nodes.push_back(start);
		_numNodes = 1;
		// setStepSize(0.01);
		// setGoalBias(0.26); // value between 0 and 1
		setMaxIterations(10000);
		// setgoalthreshold(1);
	}

	virtual ~NodeTree() {
		// delete _nodes;
		// delete _goalconfig;
		// delete _stepSize;
		// delete _goalBias;
		// delete _maxIterations;
		// delete _numNodes;
		// delete _errorfactor;
		// delete _goalflag;
	}

	//add nodes to the tree
	void addNode(RRTNode* _newNode){
		_nodes.push_back(_newNode);
		++_numNodes;
	}

	//delete nodes from the tree
	void deleteNode(unsigned int index){
		_nodes.pop_back();
		--_numNodes;
	}

	//get path from root to node at index
	NodeTree* getPath(unsigned int index){
		RRTNode* node = _nodes[index];
		NodeTree* path = new NodeTree(node);

		while(node->getParent() != NULL)
		{
			path->addNode(node); // insert 
			node = node->getParent();
		}
		path->addNode(node); 
		// std::reverse(path->_nodes.begin(),path->_nodes.end());
		return path;
	}


	RRTNode* getNodes(unsigned int index) {return _nodes[index];}

	//set goal bias
	void setGoalBias(float goalb){
		if (goalb < 0 || goalb > 1) {
			throw std::invalid_argument("The goal bias must be a number between 0.0 and 1.0");
		}
		_goalBias = goalb;
	}


	void addNodeTree(NodeTree* intermediateTree)
	{
		for(unsigned int i=0;i<intermediateTree->sizeNodes();i++)
		{
			_nodes.push_back(intermediateTree->getNodes(i));
			++_numNodes;
		}
	}

	///////////////////// Find the nearest node from the tree to current node /////////////////////

	RRTNode* nearestNeighbour(RRTNode* randomNode,NodeTree *tree)
	{
		// std::cout<<"Searching Nearest Neighbour...:"<<std::endl;
		int dist = -1;
		int nearestindex = 0;
		
		for (unsigned int i = 0; i<tree->_nodes.size();++i) {
			float currentdist = getNearestDistance(tree->_nodes[i]->getConfig(), randomNode->getConfig());
			// std::cout<<"checking Node...:"<<i<<std::endl;
			if (dist < 0 || currentdist < dist) {
				dist = currentdist;
				nearestindex = i;
			}
		}

		
		return tree->_nodes[nearestindex];
	}

	///////////////////// Find euclidean distance between given configuration and tree nodes /////////////////////

	float getDistance(const std::vector<float> &existingNodeConfig, const std::vector<float> &newconfig)
	{
		//euclidean distance
		// std::cout<<"Entered getDistance..."<<std::endl;
		float tempdist = 0;
		for (unsigned int i = 0;i<newconfig.size();++i)
		{
			tempdist += pow(newconfig[i] - existingNodeConfig[i],2);
		}
		tempdist = std::sqrt(tempdist);
		return tempdist;
	}

	////////////// Incremental update of shortest distance between two trees ///////////////////////

	float approach_dist(const std::vector<float>& child, NodeTree *test)
	{
		float distan=0.0, best = -1;
		for(unsigned int i=0; i<test->_nodes.size(); i++){
			distan = getNearestDistance(test->_nodes[i]->getConfig(), child);
			if(best < 0 || distan < best){
				best=distan;
			}
		}
		return best;
	}

	///////////////////// Find weighted euclidean distance between given configuration and tree nodes /////////////////////

	float getNearestDistance(const std::vector<float> &existingNodeConfig, const std::vector<float> &newconfig)
	{
		//euclidean distance wieghted
		// std::cout<<"Entered getNearestDistance..."<<std::endl;
		float tempdist = 0;
		// float w[6] = {600.0, 400.0, 120.0, 100, 200.0, 600.0};
		// float w[6] = {3.0, 1.0, 1.0, 1, 1.0, 5.0};
		float w[6] = {5.0, 1.0, 1.0, 1, 1.0, 5.0};
		// float w[6] = {1,1,1,1,1,1};
		for (unsigned int i = 0;i<newconfig.size();++i)
		{
			tempdist += w[i]*pow((newconfig[i] - existingNodeConfig[i]),2);
		}
		tempdist = std::sqrt(tempdist);
		return tempdist;
	}

	///////////////////// Sample a new random node /////////////////////

	RRTNode* getRamdomSample(std::vector<float> upper,std::vector<float> lower,RRTNode* goalNode,std::vector<float> baseleg)
	{	
		// std::cout<<"Entered Sampling..."<<std::endl;
		std::vector<float> sampleconfig;

		for (int i=0;i<6;i++){		
			sampleconfig.push_back(0);
		}

		float r = rand()/(float)RAND_MAX; //  r is between 0 and 1 since we normalize it
		// std::cout<<"random r:"<<r<<std::endl;

		if (r < .1){//p->goalBias()
			// std::cout<<"Goal as Sample...:"<<std::endl<<std::endl;
			// p->setgoalflag(true);
			goalflag=true;
			return goalNode;
		}

		else {
			// std::cout<<"Getting Random Sample...:"<<std::endl<<std::endl;
			// p->setgoalflag(false);
			goalflag =false;
			bool t = 0;
			// while(!t){
				for (int i = 0; i < 6; ++i){
				sampleconfig[i]=(((rand() * (upper[i]-lower[i]))/(float)RAND_MAX) + lower[i]);
				}
				// int sum  = sampleconfig[baseleg[0]]+sampleconfig[baseleg[1]]+sampleconfig[baseleg[2]];
				// if (std::abs(sum) < 0.35){t =1;}
			// }

		RRTNode* RandNode = new RRTNode(sampleconfig);
		return RandNode;
		}
	}

	///////////////////// Check for collision /////////////////////

			
	bool checkifCollision(const std::vector<float> &sampleconfig,OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot)
	{
		std::vector<OpenRAVE::dReal> config(sampleconfig.begin(),sampleconfig.end());
		robot->SetActiveDOFValues(config);
		bool collision = env->CheckCollision(robot);
		bool selfcollision = robot->CheckSelfCollision();
		
		// if(collision)
		// 	std::cout<<"Collision!"<<std::endl;
		// else if (selfcollision)
		// 	std::cout<<"Self Collision!"<<std::endl;
		return (collision||selfcollision);
	}

	///////////////////// Grow RRT tree /////////////////////


	std::vector< std::vector<float> > rrtgrow(const std::vector<float> &start,const std::vector<float> &goal,float goalbias,float stepsize,std::vector<float> upper,std::vector<float> lower,OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot, std::vector<float> baseleg, int Bi)
	{
		setgoalConfig(goal);
		setGoalBias(goalbias);
		setStepSize(stepsize);
		setgoalthreshold(1);
		// srand (static_cast <unsigned> (time(0)));
		float dist=100,ndist =10;
		RRTNode* startNode = new RRTNode(start);
		RRTNode* goalNode = new RRTNode(goal);
		NodeTree* initPath = new NodeTree(startNode);
		// initPath->_numNodes =1;
		
		// NodeTree* path= new NodeTree();
		NodeTree* finalPath = new NodeTree();
		NodeTree* source_path = new NodeTree();
		NodeTree* dest_path = new NodeTree();
		RRTNode* currentNode = NULL;
		RRTNode* prevNode = NULL;
		RRTNode* nearestNode = NULL;

		NodeTree *goal_tree = new NodeTree(goalNode);
		RRTNode *goal_near = new RRTNode;

		std::vector<std::vector<float> > _final_path,finalpathconfig,smoothened;
		
		// initPath->setgoalflag(false);
		// goalflag = false; //not needed

		std::vector<float>::const_iterator it;
		std::cout<<std::endl<<"Given:"<<std::endl<<"Goal:"<<goal[0]<<","<<goal[1]<<","<<goal[2]<<","<<goal[3]<<","<<goal[4]<<","<<goal[5]<<std::endl;
		std::cout<<std::endl<<"Start:"<<start[0]<<","<<start[1]<<","<<start[2]<<","<<start[3]<<","<<start[4]<<","<<start[5]<<std::endl;

		if (Bi == 0){
			dist = getNearestDistance(start,goal);
			std::vector<float> node;	//get disance from goal
			nearestNode = initPath->nearestNeighbour(startNode,initPath);
			std::cout<<std::endl<<"Goal Distance:"<<dist<<std::endl;

			int i=0; int threshold = 1;
			// float ndist = getNearestDistance(start,goal);
			while(dist > threshold){	//if distance from goal is greater than set threshold
			
				std::cout<<std::endl<<"Distance:"<<dist<<"threshold-"<<goalthreshold()<<std::endl;
				i++;
				RRTNode* sampledNode = initPath->getRamdomSample(upper,lower,goalNode,baseleg);

				if(!checkifCollision(sampledNode->getConfig(),env,robot)){	//check for collision
					nearestNode = initPath->nearestNeighbour(sampledNode,initPath);
					std::vector<float> nearestNodeConfig(nearestNode->getConfig().begin(),nearestNode->getConfig().end());
					// std::cout<<"Nearest found:["<<nn[0]<<","<<nn[1]<<","<<nn[2]<<","<<nn[3]<<","<<nn[4]<<","<<nn[5]<<"]"<<std::endl;
					// std::cout<<"Nearest Node found..."<<std::endl;
					
					// path = initPath->connectNodes(sampledNode, nearestNode, goalNode,initPath->stepSize(),env,robot);
					std::vector<float> targetNodeConfig(sampledNode->getConfig().begin(),sampledNode->getConfig().end());
					prevNode = nearestNode;	//nearest node becomes parent for next node
					ndist = getNearestDistance(targetNodeConfig,nearestNodeConfig);	//get distance of nearest node from sampled target

					if(ndist <= threshold){		//if within stepsize add to the initial tree
						currentNode  = new RRTNode(targetNodeConfig, prevNode);
						initPath->addNode(currentNode);

						if (getNearestDistance(goal,targetNodeConfig) < dist){	//find targets distance from goal
							dist = getNearestDistance(goal,targetNodeConfig);
						}
					}
					else{
						while(ndist > threshold){
							std::cout<<std::endl<<"Distance:"<<dist<<std::endl;
							std::cout<<std::endl<<"NearestDistance:"<<ndist<<std::endl;
							node = nearestNodeConfig;
							node = connect(nearestNodeConfig,targetNodeConfig,stepsize); //updated step from nearest node of tree towards target
							if(!getNearestDistance(node,nearestNodeConfig)){
								if(!checkifCollision(node,env,robot)){
									currentNode  = new RRTNode(node, prevNode);
									initPath->addNode(currentNode);	
									prevNode = currentNode;		// make current as parent

									if (getNearestDistance(goal,node) < dist){
										dist = getNearestDistance(goal,node);
									}
									ndist = getNearestDistance(targetNodeConfig,node);	//find new distance from sampled target
								}
								else
									break;	//if collision

								if(ndist <= threshold){	// if close to the sampled target add
									currentNode  = new RRTNode(targetNodeConfig, prevNode);
									initPath->addNode(currentNode);

									if (getNearestDistance(goal,targetNodeConfig) < dist){
										dist = getNearestDistance(goal,targetNodeConfig);
									}
								}
							}
							else 
								break;
						}
						
					}

				}
				else
					continue;					
			}

			std::cout<<"...Planning Complete,Path found..."<<std::endl;

			prevNode = currentNode;	
			RRTNode* finalNode = new RRTNode(goal,prevNode);
			initPath->addNode(finalNode);		//adding goal at the last
			finalPath = initPath->getPath(initPath->sizeNodes()-1);		//generating the path form start to goal
			std::cout<<"...FinalPath Size..."<<finalPath->sizeNodes()<<std::endl;

			std::vector<float> temp;

			for( int i=finalPath->sizeNodes()-1;i>-1;i--){			//getting reversed configurations
				temp.assign(finalPath->getNodes(i)->getConfig().begin(),finalPath->getNodes(i)->getConfig().end());
				finalpathconfig.push_back(temp);
			}
	}
	//////////////////////BiRRT Planning//////////////////////////////////////////////////
	else{
		dist = approach_dist(start, goal_tree);
		std::vector<float> node,gnode;	//get disance from goal
		std::cout<<std::endl<<"Bi-Goal Distance:"<<dist<<std::endl;

		int i=0; int threshold = 2;
		// float ndist = getNearestDistance(start,goal);
		while(dist > threshold){	//if distance from goal is greater than set threshold
		// while (ndist > .5){
			// std::cout<<std::endl<<"RRT-iteration"<<i<<std::endl;
			std::cout<<std::endl<<"Bi-Distance:"<<dist<<"threshold-"<<goalthreshold()<<std::endl;
			i++;
			RRTNode* sampledNode = initPath->getRamdomSample(upper,lower,goalNode,baseleg);

			if(!checkifCollision(sampledNode->getConfig(),env,robot)){	//check for collision
				nearestNode = initPath->nearestNeighbour(sampledNode,initPath);
				std::vector<float> nearestNodeConfig(nearestNode->getConfig().begin(),nearestNode->getConfig().end());
				std::vector<float> targetNodeConfig(sampledNode->getConfig().begin(),sampledNode->getConfig().end());
				prevNode = nearestNode;											//nearest node becomes parent for next node
				ndist = getNearestDistance(targetNodeConfig,nearestNodeConfig);	//get distance of nearest node from sampled target

				if(ndist <= threshold){											//if within stepsize add to the initial tree
					currentNode  = new RRTNode(targetNodeConfig, prevNode);
					initPath->addNode(currentNode);

					if (approach_dist(targetNodeConfig, goal_tree) < dist){		//find targets distance from goal
						dist = approach_dist(targetNodeConfig, goal_tree);
					}
				}
				else{
					while(ndist > threshold){
						std::cout<<std::endl<<"Bi-Distance:"<<dist<<std::endl;
						std::cout<<std::endl<<"Bi-NearestDistance:"<<ndist<<std::endl;
						node = nearestNodeConfig;
						node = connect(nearestNodeConfig,targetNodeConfig,stepsize); //updated step from nearest node of tree towards target
						if(!getNearestDistance(node,nearestNodeConfig)){
							if(!checkifCollision(node,env,robot)){
								currentNode  = new RRTNode(node, prevNode);
								initPath->addNode(currentNode);	
								prevNode = currentNode;								// make current as parent

								if (approach_dist(node, goal_tree) < dist){
									dist = approach_dist(node, goal_tree);
								}
								ndist = getNearestDistance(targetNodeConfig,node);	//find new distance from sampled target
							}
							else
								break;	//if collision

							if(ndist <= threshold){	// if close to the sampled target add
								currentNode  = new RRTNode(targetNodeConfig, prevNode);
								initPath->addNode(currentNode);

								if (approach_dist(targetNodeConfig, goal_tree) < dist){
									dist = approach_dist(targetNodeConfig, goal_tree);
								}
							}
						}
						else 
							break;
					}
					
				}

			}
			else
				continue;	

		//////////////////Goal tree Side//////////////////////

			goal_near = new RRTNode;
			RRTNode* prevNodeg = new RRTNode;
			if(!checkifCollision(sampledNode->getConfig(),env,robot)){
				goal_near = goal_tree->nearestNeighbour(sampledNode,goal_tree);
				prevNodeg = goal_near;
				std::vector<float> gnearestNodeConfig(goal_near->getConfig().begin(),goal_near->getConfig().end());
				std::vector<float> gtargetNodeConfig(sampledNode->getConfig().begin(),sampledNode->getConfig().end());
				ndist = getNearestDistance(gtargetNodeConfig,gnearestNodeConfig);

				if(ndist <= threshold){											//if within stepsize add to the initial tree
					break;
				}
				else{
					while(ndist > threshold){
						std::cout<<std::endl<<"G-Distance:"<<dist<<std::endl;
						std::cout<<std::endl<<"G-NearestDistance:"<<ndist<<std::endl;
						gnode = gnearestNodeConfig;
						gnode = connect(gnearestNodeConfig,gtargetNodeConfig,stepsize); //updated step from nearest node of tree towards target
						if(!getNearestDistance(gnode,gnearestNodeConfig)){
							if(!checkifCollision(gnode,env,robot)){
								goalNode  = new RRTNode(gnode, prevNode);
								goal_tree->addNode(goalNode);	
								prevNodeg = goalNode;								// make current as parent

								if (approach_dist(gnode, initPath) < dist){
									dist = approach_dist(gnode, initPath);
								}
								ndist = getNearestDistance(gtargetNodeConfig,gnode);	//find new distance from sampled target
							}
							else
								break;	//if collision

						}
						else 
							break;
					}
					
				}

			}
		}	

		std::cout<<"...BiRRT-Planning Complete,Path found..."<<std::endl;

		
		source_path = initPath->getPath(initPath->sizeNodes()-1);
		dest_path = goal_tree->getPath(goal_tree->sizeNodes()-1);
		
		std::vector<float> temp;

		for( int i=source_path->sizeNodes()-1;i>-1;i--){			//getting reversed configurations
			temp.assign(source_path->getNodes(i)->getConfig().begin(),source_path->getNodes(i)->getConfig().end());
			finalpathconfig.push_back(temp);
		}

		for( unsigned int i=0; i<dest_path->sizeNodes()-1;++i){			//getting reversed configurations
			temp.assign(dest_path->getNodes(i)->getConfig().begin(),dest_path->getNodes(i)->getConfig().end());
			finalpathconfig.push_back(temp);
		}

	}

	///////////////////////completed planning////////////////////
		

		for(it=finalpathconfig.back().begin(); it!=finalpathconfig.back().end(); ++it){
			std::cout<<"final Node:"<<(*it)<<std::endl;
		}

		if (finalpathconfig.size() > 5){
			std::cout<<" ...Smoothing.."<<std::endl;
			smoothened = smoothpath(finalpathconfig,stepsize,env,robot);		//shortcut smoothened path
			std::cout<<" ...Smoothing Complete..."<<std::endl;
			_final_path.reserve(finalpathconfig.size() + smoothened.size()); // preallocate memory
			// _final_path.insert( _final_path.end(), finalpathconfig.begin(), finalpathconfig.end());	//if req, unsmoothened path
			_final_path.insert(_final_path.end(), smoothened.begin(), smoothened.end());
		}
		else{
			_final_path.reserve(finalpathconfig.size()); // preallocate memory
			_final_path.insert( _final_path.end(), finalpathconfig.begin(), finalpathconfig.end());	//if req, unsmoothened path
			// _final_path.insert(_final_path.end(), smoothened.begin(), smoothened.end());
		}
		std::cout<<"...Final Size..."<<finalpathconfig.size()<<std::endl;
		
		return _final_path;
	}

	std::vector<float> connect(std::vector<float>& parent, std::vector<float>& rand_config, float stepsize)
	{
		std::vector<float> nNode;
		float leng = getNearestDistance(parent, rand_config);
		for(unsigned int i=0; i<rand_config.size(); i++){
			nNode.push_back(parent[i] + ((rand_config[i] - parent[i])*(stepsize/leng)));
		}
		return nNode;
	}


	//////////////////////// Shortcut smoothing to smooth the rrt path ////////////////////

	std::vector< std::vector<float> > smoothpath(std::vector< std::vector<float> > path_smoothened, float stepsize,OpenRAVE::EnvironmentBasePtr& env,OpenRAVE::RobotBasePtr& robot)
	{
		int hi, ran1, ran2, p1, p2, collided;
		std::vector<float>  v1, v2, inter;
		std::vector< std::vector<float> > bypass;
		std::vector< std::vector<float> >::iterator it;
		int n = 5;
		int xx = (path_smoothened.size())/n;

		for(int i=0; i<xx; i++){
			hi = path_smoothened.size();
			ran1 = rand()%hi;
			ran2 = rand()%hi;
			p1 = std::min(ran1,ran2);
			p2 = std::max(ran1,ran2);
			if(p1 != p2){
				v1 = path_smoothened[p1];	// sample random 2 points
				v2 = path_smoothened[p2];
				inter = v1;
				bypass.push_back(v1);
				it = path_smoothened.begin();

				while(1){
					inter = connect(inter,v2,stepsize);	//step towards the sample node
					collided = checkifCollision(inter,env,robot);
					if(collided == 1){
						break;
					}
					else{
						bypass.push_back(inter);
						if(getNearestDistance(inter, v2) < stepsize){
							break;
						}
					}
				}
				if(collided == 0){
					path_smoothened.erase(path_smoothened.begin()+p1, path_smoothened.begin()+p2); //erase old and insert smooth path
					path_smoothened.insert(it+p1, bypass.begin(), bypass.end());
				}
				bypass.clear();
			}
		}	
		return path_smoothened;
	}

	///////////////////// Useful finctions /////////////////////

	const std::vector<float> goalConfig(){return _goalconfig;}

	const std::vector<RRTNode*> allNodes() {return _nodes;}

	unsigned int sizeNodes(){return _numNodes;}

	void setgoalConfig(const std::vector<float> &goalConfig) {_goalconfig = goalConfig;}

	void setgoalflag(bool f=false) {_goalflag = f;}

	bool goalFlag() {return _goalflag;}

	float stepSize() const {return _stepSize;}

	void setStepSize(float stepSize = 0.25) {_stepSize = stepSize;}

	float goalthreshold() const {return _goal_threshold;}

	void setgoalthreshold(float err = 1) {_goal_threshold = err;}

	float goalBias() const {return _goalBias;}

	int maxIterations() const {return _maxIterations;}

	void setMaxIterations(int itr = 1000) {_maxIterations = itr;}

	RRTNode* rootNode() const {
		if (_nodes.empty()) 
			return NULL;

	return _nodes.front();
	}


	RRTNode* lastNode() const {
		if (_nodes.empty()) 
			return NULL;
		return _nodes.back();
	}

};

