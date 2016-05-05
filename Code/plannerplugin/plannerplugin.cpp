#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "gait_planner.hpp"

using namespace OpenRAVE;

class plannermodule : public ModuleBase
{
public:
    plannermodule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("gaitplanner",boost::bind(&plannermodule::gaitplannercommand,this,_1,_2),
                        "This is an example command");
    }
    virtual ~plannermodule() {}
    
    bool gaitplannercommand(std::ostream& sout, std::istream& sinput)
    {   

        EnvironmentBasePtr env;
        // bool rflag = true;
        env = GetEnv();
        RobotBasePtr robot;
        RobotBasePtr robot1;
        std::vector<RobotBasePtr> pr2;

        env->GetRobots(pr2);


        robot = pr2[0];
        std::cout<<" Gait Planning1..."<<std::endl;
               
        

        float start, goal, goalbias = .26, stepsize;
        std::vector<float> startconfig;
        std::vector<float> goalconfig;
        std::vector<float> baseleg;

        for(int i=0; i<6; i++){
            sinput >> start;
            startconfig.push_back(start);
        }
        
        for(int i=0; i<6; i++){
            sinput >> goal;
            goalconfig.push_back(goal);
        }

        sinput >> goalbias;
        sinput >> stepsize;

        for(int i=0; i<3; i++){
            sinput >> goal;
            baseleg.push_back(goal);
        }

        int biflag=0;
        sinput >> biflag;
                

        std::vector<int> indices;
        std::vector<OpenRAVE::dReal> upperlimit,lowerlimit;
        
        indices = robot->GetActiveDOFIndices();        
        robot->GetDOFLimits(lowerlimit,upperlimit,indices);

        lowerlimit[0]= -3.1416/4; upperlimit[0]= 3.1416/4;
        lowerlimit[1]= -3.1416/2; upperlimit[1]= 0.0;
        lowerlimit[2]= -3.1416/2; upperlimit[2]= 3.1416/2;
        lowerlimit[3]= 3.1416/2; upperlimit[3]= -3.1416/2;
        lowerlimit[4]= 0.0; upperlimit[4]= -3.1416/2;
        lowerlimit[5]= 3.1416/2; upperlimit[5]= -3.1416/4;

        // for(int i=0; i<6; ++i){

        //     std::cout<<"indices"<<" "<<indices[i]<<std::endl;
        //     std::cout<<"lowerlimit"<<i<<" "<<lowerlimit[i]<<std::endl;
        //     std::cout<<"upperlimit"<<i<<" "<<upperlimit[i]<<std::endl;
        // }

        std::vector<float> lower(lowerlimit.begin(), lowerlimit.end());
        std::vector<float> upper(upperlimit.begin(), upperlimit.end());
        std::vector< std::vector<float> >  pathConfigs; 
        
        // NodeTree *Final_Path= new NodeTree();
        // Final_Path = Final_Path->rrtgrow(startconfig,goalconfig,goalbias,upper,lower,env,robot);
        NodeTree a;
        pathConfigs  = a.rrtgrow(startconfig,goalconfig,goalbias,stepsize,upper,lower,env,robot,baseleg,biflag);
        
        std::vector<std::vector<float> > temp;
        // std::vector<float>::const_iterator it;
        // std::vector<float>::const_iterator iit;

                
        for (unsigned int j=0; j<pathConfigs.size(); j++){
            for(int k=0; k<6; k++){
                sout << pathConfigs[j][k];
                sout << " ";

            }
            sout << ";";
            
        }



        // for(it=pathConfigs.begin(); it!=pathConfigs.end(); it++){
        //     temp.assign(pathConfigs.begin(),pathConfigs.end());
        //     // pathConfigs.push_back(temp[i]);
        //     std::cout<<(*it)<<std::endl;
        //     // std::cout<<"Nodes"<<Final_Path->sizeNodes()-i-1<<":";
        //     // for(it =temp.begin(); it !=temp.end(); it++){
        //     //    sout<<(*it);
        //     //    sout<<" "; 
        //     //    std::cout<<(*it)<<" ";
        //     // } 
        //     // sout<<";";
        //     // std::cout<<std::endl; 

        // }
        return true;
    }

};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "plannermodule" ) {
        return InterfaceBasePtr(new plannermodule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("plannermodule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

