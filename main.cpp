#include <iostream>
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include <fstream>
#include <algorithm>

/****************************************/
/****************************************/

/* Definition of functions to assimilate the different feature-vectors and perform abnormality detection */

#include "featurevectorsinrobotagent.h"
#include "crminrobotagent_optimised.h"

/****************************************/
/****************************************/

#define NUM_ROBOTS_EXPT 7 // Number of robots in the experiment
#define NUMBER_FEATURES 6 // Number of features in feature vector

std::string SWARM_BEHAV("SWARM_HOMING_MOVING_BEACON");
std::string ERROR_BEHAV("FAULT_ACTUATOR_RWHEEL_SETZERO");
std::string RAND_SEED("777");

/****************************************/
/****************************************/

using namespace std;

int main()
{
    t_listFVsSensed               listFVsSensed;
    t_listMapFVsToRobotIds        listMapFVsToRobotIds; // ids and fvs of observed neighbours, including ids and fvs the neighbours have relayed to you

    CRMinRobotAgentOptimised*     crminAgent[NUM_ROBOTS_EXPT];


    /****************************************************/
    std::map<size_t, size_t> RobotIndex_ARGoSID_Map;
    RobotIndex_ARGoSID_Map[0] = 201u;
    RobotIndex_ARGoSID_Map[1] = 202u;
    RobotIndex_ARGoSID_Map[2] = 206u;
    RobotIndex_ARGoSID_Map[3] = 207u;
    RobotIndex_ARGoSID_Map[4] = 208u;
    RobotIndex_ARGoSID_Map[5] = 209u;
    RobotIndex_ARGoSID_Map[6] = 210u;
    /****************************************************/

    std::vector< std::vector<unsigned> > m_vec2dPropioceptiveFV(NUM_ROBOTS_EXPT);

    for(size_t i = 0; i < NUM_ROBOTS_EXPT; ++i)
    {
        std::string fvlog_filename;
        fvlog_filename = "_" + SWARM_BEHAV + "_" + ERROR_BEHAV + "_" + RAND_SEED + ".explog";
        std::ostringstream os;
        os << RobotIndex_ARGoSID_Map[i] << fvlog_filename;

        std::cout << "Reading file " << os.str() << std::endl;

        ifstream myfile(fvlog_filename.c_str());
        if (myfile.is_open())
        {
            std::vector <unsigned> m_vecTimeSeriesSingleRobotFV;
            while(!myfile.eof())
            {
                int step, fv;
                myfile >> step >> fv;
                std::cout << step << fv << '\n';
                m_vecTimeSeriesSingleRobotFV.push_back(fv);
            }
            myfile.close();
            m_vec2dPropioceptiveFV[i] = m_vecTimeSeriesSingleRobotFV;
            std::cout << "Number of steps " << m_vec2dPropioceptiveFV[i].size();
        }
        else cout << "Unable to open file " << os.str();
    }

    // crude check that all robots have recorded FVs for same time-length
    for(size_t i = 0; i < NUM_ROBOTS_EXPT - 1; ++i)
        assert(m_vec2dPropioceptiveFV[i].size() ==  m_vec2dPropioceptiveFV[i+1].size());

    std::vector< std::vector<unsigned> > m_vec2dAttackingRobots(NUM_ROBOTS_EXPT,  std::vector<unsigned>(m_vec2dPropioceptiveFV[0].size(), 0));
    std::vector< std::vector<unsigned> > m_vec2dToleratingRobots(NUM_ROBOTS_EXPT, std::vector<unsigned>(m_vec2dPropioceptiveFV[0].size(), 0));

    for(size_t observer_robot = 0; observer_robot < NUM_ROBOTS_EXPT; ++observer_robot)
    {
        crminAgent[observer_robot] =  new CRMinRobotAgentOptimised(RobotIndex_ARGoSID_Map[observer_robot], NUMBER_FEATURES);
        listFVsSensed.clear();        assert(listFVsSensed.size() == 0u);
        listMapFVsToRobotIds.clear(); assert(listMapFVsToRobotIds.size() == 0u);

        for(size_t time_step = 0; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
        {
            for(size_t observed_robot = 0; observed_robot < NUM_ROBOTS_EXPT; ++observed_robot)
            {
                if(observer_robot == observed_robot)
                    continue;

                UpdateFvToRobotIdMap(listMapFVsToRobotIds, m_vec2dPropioceptiveFV[observed_robot][time_step], observed_robot);
            }
            UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, NUM_ROBOTS_EXPT-1);
            crminAgent[observer_robot]->SimulationStepUpdatePosition((double)time_step, &listFVsSensed);

            for (t_listFVsSensed::iterator it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
            {
                if((*it_dist).uMostWantedState == 1)
                {
                    for(size_t tmp_observed_robots = 0; tmp_observed_robots < NUM_ROBOTS_EXPT; ++tmp_observed_robots)
                    {
                        if(observer_robot == tmp_observed_robots)
                            continue;
                        if(m_vec2dPropioceptiveFV[tmp_observed_robots][time_step] == (*it_dist).uFV)
                            m_vec2dAttackingRobots[tmp_observed_robots][time_step]++;
                    }
                }
                else if((*it_dist).uMostWantedState == 2)
                {
                    for(size_t tmp_observed_robots = 0; tmp_observed_robots < NUM_ROBOTS_EXPT; ++tmp_observed_robots)
                    {
                        if(observer_robot == tmp_observed_robots)
                            continue;
                        if(m_vec2dPropioceptiveFV[tmp_observed_robots][time_step] == (*it_dist).uFV)
                            m_vec2dToleratingRobots[tmp_observed_robots][time_step]++;
                    }
                }
            }
        }
    }


    std::string m_strOutput;
    m_strOutput = "crm_" + SWARM_BEHAV + "_" + ERROR_BEHAV + "_" + RAND_SEED + ".result";
    std::ofstream m_cOutput; m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);

    for(size_t time_step = 0; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
    {
        for(size_t observed_robot = 0; observed_robot < NUM_ROBOTS_EXPT; ++observed_robot)
        {
            m_cOutput << time_step << "\t" << RobotIndex_ARGoSID_Map[observed_robot] << "\t" <<
                         m_vec2dAttackingRobots[observed_robot][time_step] << "\t" << m_vec2dToleratingRobots[observed_robot][time_step] << std::endl;
        }
        m_cOutput << std::endl;
    }
    m_cOutput.close();

    return 0;
}

