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

#define FILTER_LENGTH 1 // CRM decisions accumulated over FILTER_LENGTH/10 second window
#define FILTER_THRESHOLD 0.5 // Robot treated as normal if it has been detected as such for more than FILTER_THRESHOLD proportion of FILTER_LENGTH time

#define MODEL_START_TIME 0 // FV comprises 45s of history

std::string SWARM_BEHAV;//("SWARM_DISPERSION");
std::string ERROR_BEHAV;//("FAULT_ACTUATOR_BWHEELS_SETZERO");
std::string RAND_SEED;//("111");
std::string AFFINITY_VALUE;

/****************************************/
/****************************************/

using namespace std;

int main(int argc, char**argv)
{
    assert(argc == 5);
    SWARM_BEHAV = argv[1];
    ERROR_BEHAV = argv[2];
    RAND_SEED   = argv[3];
    AFFINITY_VALUE = argv[4];
    double affinity_value = atof(argv[4]);
    //printf("\n affinity_value %f", affinity_value);

    t_listFVsSensed               listFVsSensed;
    t_listMapFVsToRobotIds        listMapFVsToRobotIds; // ids and fvs of observed neighbours, including ids and fvs the neighbours have relayed to you

    CRMinRobotAgentOptimised*     crminAgent[NUM_ROBOTS_EXPT];
    int fv[NUM_ROBOTS_EXPT], id[NUM_ROBOTS_EXPT];
    

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

    std::vector< std::list<unsigned> >   m_vecFilterCRMDecision(NUM_ROBOTS_EXPT);

    for(size_t i = 0; i < NUM_ROBOTS_EXPT; ++i)
    {
        std::string fvlog_filename;
        fvlog_filename = SWARM_BEHAV + "_" + ERROR_BEHAV + "_" + RAND_SEED + ".fvlog_PropFV"; // for behav trans expt ERROR_BEHAV = DISPERSION_0.1, DISPERSION_0.2 and DISPERSION_1.0
        std::ostringstream os;
        os << fvlog_filename;

        std::cout << "Reading file " << os.str() << std::endl;

        ifstream myfile(os.str());
        if (myfile.is_open())
        {
            std::vector <unsigned> m_vecTimeSeriesSingleRobotFV;
            while(!myfile.eof())
            {
                int step;
                myfile >> step >> id[0] >> fv[0] >> id[1] >> fv[1] >> id[2] >> fv[2] >> id[3] >> fv[3] >> id[4] >> fv[4] >> id[5] >> fv[5] >> id[6] >> fv[6];
                //std::cout << step << " " << fv << std::endl;
                m_vecTimeSeriesSingleRobotFV.push_back(fv[i]);
            }
            myfile.close();
            m_vec2dPropioceptiveFV[i] = m_vecTimeSeriesSingleRobotFV;
            std::cout << "Number of steps " << m_vec2dPropioceptiveFV[i].size() << std::endl;
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
        crminAgent[observer_robot] =  new CRMinRobotAgentOptimised(RobotIndex_ARGoSID_Map[observer_robot], NUMBER_FEATURES, affinity_value);


        //for(size_t time_step = 0; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
        for(size_t time_step = MODEL_START_TIME; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
        {
	  //printf("\n robot id %d; time_step %d \n\n", observer_robot, time_step);
            listFVsSensed.clear();        assert(listFVsSensed.size() == 0u);
            listMapFVsToRobotIds.clear(); assert(listMapFVsToRobotIds.size() == 0u);

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
                        {
//                            m_vecFilterCRMDecision[tmp_observed_robots].push_back(1u);
//                            if(m_vecFilterCRMDecision[tmp_observed_robots].size() > FILTER_LENGTH)
//                                m_vecFilterCRMDecision[tmp_observed_robots].pop_front();
                            m_vec2dAttackingRobots[tmp_observed_robots][time_step]++;
                        }
                    }
                }
                else if((*it_dist).uMostWantedState == 2)
                {
                    for(size_t tmp_observed_robots = 0; tmp_observed_robots < NUM_ROBOTS_EXPT; ++tmp_observed_robots)
                    {
                        if(observer_robot == tmp_observed_robots)
                            continue;
                        if(m_vec2dPropioceptiveFV[tmp_observed_robots][time_step] == (*it_dist).uFV)
                        {
//                            m_vecFilterCRMDecision[tmp_observed_robots].push_back(0u);
//                            if(m_vecFilterCRMDecision[tmp_observed_robots].size() > FILTER_LENGTH)
//                                m_vecFilterCRMDecision[tmp_observed_robots].pop_front();
                            m_vec2dToleratingRobots[tmp_observed_robots][time_step]++;
                        }
                    }
                }
            }
        }
    }


    std::string m_strOutput;
    m_strOutput = "crm_affinity_" + AFFINITY_VALUE + "_" + SWARM_BEHAV + "_" + ERROR_BEHAV + "_" + RAND_SEED + ".result";
    std::ofstream m_cOutput; m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);

    std::vector<double> m_vecSummary_Attack  (NUM_ROBOTS_EXPT, 0);
    std::vector<double> m_vecSummary_Tolerate(NUM_ROBOTS_EXPT, 0);
    std::vector<unsigned> m_vecLatency_Attack(NUM_ROBOTS_EXPT, 100000u);


    //for(size_t time_step = 0; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
    for(size_t time_step = MODEL_START_TIME; time_step < m_vec2dPropioceptiveFV[0].size(); ++time_step)
    {
        for(size_t observed_robot = 0; observed_robot < NUM_ROBOTS_EXPT; ++observed_robot)
        {
            m_cOutput << time_step << "\t" << RobotIndex_ARGoSID_Map[observed_robot] << "\t" <<
                         m_vec2dPropioceptiveFV[observed_robot][time_step] << "\t" <<
                         m_vec2dAttackingRobots[observed_robot][time_step] << "\t" << m_vec2dToleratingRobots[observed_robot][time_step] << std::endl;


            if(m_vec2dToleratingRobots[observed_robot][time_step]  >= m_vec2dAttackingRobots[observed_robot][time_step])
            {
                m_vecFilterCRMDecision[observed_robot].push_back(1u);
                if(m_vecFilterCRMDecision[observed_robot].size() > FILTER_LENGTH)
                    m_vecFilterCRMDecision[observed_robot].pop_front();
            }
            else
            {
                m_vecFilterCRMDecision[observed_robot].push_back(0u);
                if(m_vecFilterCRMDecision[observed_robot].size() > FILTER_LENGTH)
                    m_vecFilterCRMDecision[observed_robot].pop_front();
            }

            if(m_vecFilterCRMDecision[observed_robot].size() >= FILTER_LENGTH)
            {
	      	   double norm_sum_of_elems = (double)(std::accumulate(m_vecFilterCRMDecision[observed_robot].begin(), m_vecFilterCRMDecision[observed_robot].end(), 0)) /
		   (double) FILTER_LENGTH;

		   //if(m_vec2dToleratingRobots[observed_robot][time_step]  >= m_vec2dAttackingRobots[observed_robot][time_step])
		   if(norm_sum_of_elems >= FILTER_THRESHOLD)
		   {
		       m_vecSummary_Tolerate[observed_robot]++;
		   }
		   else
		   {
		       m_vecSummary_Attack[observed_robot]++;
		       if(m_vecLatency_Attack[observed_robot] == 100000u)
		       {
			   m_vecLatency_Attack[observed_robot] = time_step;
		       }
		   }
		   
		   /*{
		  if(m_vec2dToleratingRobots[observed_robot][time_step]  >= m_vec2dAttackingRobots[observed_robot][time_step])
		    m_vecSummary_Tolerate[observed_robot]++;
		  else
		    m_vecSummary_Attack[observed_robot]++;

		    if(m_vecLatency_Attack[observed_robot] == 100000u)
		    {
		      m_vecLatency_Attack[observed_robot] = time_step;
		    }
		    }*/
            }

        }
        m_cOutput << std::endl;
    }

    m_cOutput << std::endl << std::endl << std::endl << std::endl << std::endl;

    for(size_t observed_robot = 0; observed_robot < NUM_ROBOTS_EXPT; ++observed_robot)
    {
        m_vecSummary_Tolerate[observed_robot] = (double)m_vecSummary_Tolerate[observed_robot] / (double)(m_vec2dPropioceptiveFV[0].size() - FILTER_LENGTH+1 - MODEL_START_TIME);
        m_vecSummary_Attack[observed_robot]   = (double)m_vecSummary_Attack[observed_robot]   / (double)(m_vec2dPropioceptiveFV[0].size() - FILTER_LENGTH+1 - MODEL_START_TIME);

        m_cOutput << 100000u << "\t" << RobotIndex_ARGoSID_Map[observed_robot] << "\t" <<
                     m_vecSummary_Attack[observed_robot] << "\t" << m_vecSummary_Tolerate[observed_robot] << "\t" << m_vecLatency_Attack[observed_robot] << std::endl;

    }
    m_cOutput.close();

    return 0;
}

