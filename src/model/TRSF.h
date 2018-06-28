#ifndef TRSF_H
#define TRSF_H

#include "../proNet.h"
/*****
 * TRSF
 * **************************************************************/

class TRSF{

    public:

        TRSF();
        ~TRSF();
        
		proNet pnet;

        // parameters
        int dim;                // representation dimensions

        vector< vector<double> > w_vertex;
		vector< vector<double> > w_context;
		vector< vector<double> > w_context_srcUI;
		vector< vector<double> > w_context_srcUSim;
        vector< vector<double> > w_context_srcISim;

        vector< vector<double> > w_context_tarUI;
        vector< vector<double> > w_context_tarUSim;
        vector< vector<double> > w_context_tarISim;

        vector< vector<double> > w_context_CDUtSim;
        vector< vector<double> > w_context_CDUsSim;
        vector< vector<double> > w_context_CDIsSim;
        vector< vector<double> > w_context_CDItSim;

        vector< vector<double> > w_context_SimBridge;
        vector< vector<double> > w_context_tranCF_s;
        vector< vector<double> > w_context_tranCF_t;

		//Alias Tables
		vector< AliasTable > dist_vertex_AT;
		vector< AliasTable > dist_context_AT;
		vector< AliasTable > dist_negative_AT;

		vector< AliasTable > angle_vertex_AT;
		vector< AliasTable > angle_context_AT;
		vector< AliasTable > angle_negative_AT;
		vector<long> non_joint_vid;

        // data function
        void LoadEdgeList(string, string, bool, bool);
		void GetNodeStatus();
        void LoadEdgeList(string, bool);
        void LoadEdgeListTransfer(string, bool);
		void LoadNonJointNodeList(string);
        void SaveWeights(string);
		long SearchVid(HashTable& hash_table, char *key);
//		int *vid_status;

        // model function
        void Init(int);
        void Train(double, int, double, int, double);

};


#endif
