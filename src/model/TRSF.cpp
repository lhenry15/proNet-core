#include "TRSF.h"
#include <omp.h>

TRSF::TRSF() {
}
TRSF::~TRSF() {
}

long TRSF::SearchVid(HashTable& hash_table, char *key) {
    unsigned int pos = pnet.BKDRHash(key);
    while (1)
    {
        if (hash_table.table[pos] == -1)
            return -1;
        if ( !strcmp(key, hash_table.keys[ hash_table.table[pos] ]) )
            return hash_table.table[pos];
        pos = (pos + 1) % HASH_TABLE_SIZE;
    }
}

void TRSF::GetNodeStatus(){
	//vid_status --> 1:src , 2:tar
	//joint_node --> overlap nodes

	//Get unique Uid and Iid for both domain
	sort(pnet.src_uid.begin(), pnet.src_uid.end());
	pnet.src_uid.erase(unique(pnet.src_uid.begin(), pnet.src_uid.end()), pnet.src_uid.end());
	sort(pnet.tar_uid.begin(), pnet.tar_uid.end());
	pnet.tar_uid.erase(unique(pnet.tar_uid.begin(), pnet.tar_uid.end()), pnet.tar_uid.end());
	sort(pnet.src_iid.begin(), pnet.src_iid.end());
	pnet.src_iid.erase(unique(pnet.src_iid.begin(), pnet.src_iid.end()), pnet.src_iid.end());
	sort(pnet.tar_iid.begin(), pnet.tar_iid.end());
	pnet.tar_iid.erase(unique(pnet.tar_iid.begin(), pnet.tar_iid.end()), pnet.tar_iid.end());

	pnet.vid_status = new int[pnet.MAX_vid];
	pnet.joint_node = new int[pnet.MAX_vid];

	//0 for initialize
	for (long i=0; i<pnet.MAX_vid; i++){
		pnet.vid_status[i] = 0;
	}

	// 1-->Us, 2-->Ut, 3-->Ins, 4-->Int, 5-->Io
	// s-->src, t-->tar, ns --> non-overlap src, nt --> non-overlap tar, o --> overlap
	for (unsigned int i = 0; i < pnet.src_uid.size(); i++){
		long vid = pnet.src_uid[i];
		pnet.vid_status[vid] = 1;
	}
	for (unsigned int i = 0; i < pnet.tar_uid.size(); i++){
		long vid = pnet.tar_uid[i];
		pnet.vid_status[vid] = 2;
	}
	for (unsigned int i = 0; i < pnet.src_iid.size(); i++){
		long vid = pnet.src_iid[i];
		pnet.vid_status[vid] = 3;
	}
	for (unsigned int i = 0; i < pnet.tar_iid.size(); i++){
		long vid = pnet.tar_iid[i];
		if (pnet.vid_status[vid] == 3)
			pnet.vid_status[vid] = 5;
		else
			pnet.vid_status[vid] = 4;
	}

}

void TRSF::LoadEdgeList(string filedir, bool undirect) {
	pnet.Load2EdgeList(filedir, undirect);
}

void TRSF::LoadNonJointNodeList(string filename) {
	//pnet.GetJoint(filedir);
    FILE *fin;
    char c_line[1000];
    unsigned long long max_line=0;
    
    cout << "Joint Nodes Preview:" << endl;
    fin = fopen(filename.c_str(), "rb");
    while (fgets(c_line, sizeof(c_line), fin))
    {
        if (max_line % MONITOR == 0)
        {
            printf("\t# of non-joint nodes:\t\t%llu%c", max_line, 13);
        }
        ++max_line;
    }
    fclose(fin);
    cout << "\t# of non-joint nodes:\t\t" << max_line << endl;
    
    char v[160], meta[160];
    long vid;

    cout << "Non-Joint Nodes Loading:" << endl;
    fin = fopen(filename.c_str(), "rb");
    for (unsigned long long line = 0; line != max_line; line++)
    {
        if ( fscanf(fin, "%s", v)!=1 )
        {
            cout << "line " << line << " contains wrong number of data" << endl; 
            continue;
        }
        vid = pnet.SearchHashTable(pnet.vertex_hash, v);
		non_joint_vid.push_back(vid);
        if (line % MONITOR == 0)
        {
            printf("\tProgress:\t\t%.2f %%%c", (double)(line)/(max_line+1) * 100, 13);
            fflush(stdout);
        }
	}	
    cout << "\tProgress:\t\t100.00 %\r" << endl;
}


void TRSF::SaveWeights(string model_name){
    cout << "Save Model:" << endl;
    ofstream model(model_name);
    if (model)
    {
        model << pnet.MAX_vid << " " << dim << endl;

		for (long vid=0; vid!=pnet.MAX_vid; vid++)
		{
			model << pnet.vertex_hash.keys[vid];
			for (int d=0; d<dim; ++d)
				model << " " << w_vertex[vid][d];
			model << endl;
		}
        cout << "\tSave to <" << model_name << ">" << endl;
    }
    else
    {
        cout << "\tfail to open file" << endl;
    }
}
void TRSF::Init(int dimension) {
   
    cout << "Model Setting:" << endl;
    cout << "\tdimension:\t\t" << dimension << endl;
    this->dim = (int)(dimension);

	w_vertex.resize(pnet.MAX_vid);
	w_context.resize(pnet.MAX_vid);

	w_context_srcUI.resize(pnet.MAX_vid);
	w_context_srcUSim.resize(pnet.MAX_vid);
	w_context_srcISim.resize(pnet.MAX_vid);

	w_context_tarUI.resize(pnet.MAX_vid);
	w_context_tarUSim.resize(pnet.MAX_vid);
	w_context_tarISim.resize(pnet.MAX_vid);

	w_context_CDUtSim.resize(pnet.MAX_vid);
	w_context_CDUsSim.resize(pnet.MAX_vid);

	w_context_CDIsSim.resize(pnet.MAX_vid);
	w_context_CDItSim.resize(pnet.MAX_vid);

	w_context_SimBridge.resize(pnet.MAX_vid);
	w_context_tranCF_s.resize(pnet.MAX_vid);
	w_context_tranCF_t.resize(pnet.MAX_vid);


	for (long vid=0; vid<pnet.MAX_vid; ++vid){
		w_vertex[vid].resize(dim);
		for (int d=0; d<dim;++d)
		{
			//w_vertex[vid][d] = (rand()/(double)RAND_MAX - 0.5 );
			w_vertex[vid][d] = (rand()/(double)RAND_MAX - 0.5) / dim;
		}
	}
	for (long vid=0; vid<pnet.MAX_vid; ++vid){
		w_context[vid].resize(dim);

		w_context_srcUI[vid].resize(dim);
		w_context_srcUSim[vid].resize(dim);
		w_context_srcISim[vid].resize(dim);

		w_context_tarUI[vid].resize(dim);
		w_context_tarUSim[vid].resize(dim);
		w_context_tarISim[vid].resize(dim);

		w_context_CDUtSim[vid].resize(dim);
		w_context_CDUsSim[vid].resize(dim);

		w_context_CDIsSim[vid].resize(dim);
		w_context_CDItSim[vid].resize(dim);

		w_context_SimBridge[vid].resize(dim);
		w_context_tranCF_s[vid].resize(dim);
		w_context_tranCF_t[vid].resize(dim);
		for (int d=0; d<dim;++d){
			w_context[vid][d] = 0.0;

			w_context_srcUI[vid][d] = 0.0;
			w_context_srcUSim[vid][d] = 0.0;
			w_context_srcISim[vid][d] = 0.0;

			w_context_tarUI[vid][d] = 0.0;
			w_context_tarUSim[vid][d] = 0.0;
			w_context_tarISim[vid][d] = 0.0;

			w_context_CDUtSim[vid][d] = 0.0;
			w_context_CDUsSim[vid][d] = 0.0;

 			w_context_CDIsSim[vid][d] = 0.0;
 			w_context_CDItSim[vid][d] = 0.0;

			w_context_SimBridge[vid][d] = 0.0;
			w_context_tranCF_s[vid][d] = 0.0;
			w_context_tranCF_t[vid][d] = 0.0;
		}
	}
}
void TRSF::Train(double sample_times, int negative_samples, double alpha, int workers, double reg){

    omp_set_num_threads(workers);

    cout << "Model:" << endl;
    cout << "\t[TRSF]" << endl;

    cout << "Learning Parameters:" << endl;
    cout << "\tsample_times:\t\t" << sample_times << endl;
    cout << "\tregularization:\t\t" << reg << endl;
    cout << "\tnegative_samples:\t" << negative_samples << endl;
    cout << "\talpha:\t\t\t" << alpha << endl;
    cout << "\tworkers:\t\t" << workers << endl;

    unsigned long long total_sample_times = sample_times*1000000;
    double alpha_min = alpha * 0.000001;
    double alpha_last;
    unsigned long long current_sample = 0;
    unsigned long long jobs = total_sample_times/workers;

	#pragma omp parallel for
	for (int worker=0; worker<workers; ++worker){
		unsigned long long count = 1;
		double _alpha = alpha;
		long v1=0,v2=0,v3=0;
		while (count<jobs)
		{            
			//sample 2-hop walk
			v1 = pnet.SourceSample();
			v2 = pnet.TargetSample(v1);
			v3 = pnet.TargetSample(v2);	
			if (pnet.vid_status[v1]==1 && pnet.vid_status[v2]==3 && pnet.vid_status[v3]==1){//Us-Is-Us
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg);	

			}else if(pnet.vid_status[v1]==3 && pnet.vid_status[v2]==1 && pnet.vid_status[v3]==3){//Is-Us-Is
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg);	

			}else if (pnet.vid_status[v1]==2 && pnet.vid_status[v2]==4 && pnet.vid_status[v3]==2){//Ut-It-Ut
				pnet.UpdateTPair(w_vertex,w_context_tarUI,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context_tarUI,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg);	

			}else if (pnet.vid_status[v1]==4 && pnet.vid_status[v2]==2 && pnet.vid_status[v3]==4){//It-Ut-It
				pnet.UpdateTPair(w_vertex,w_context_tarUI,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context_tarUI,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg);	

			}else if (pnet.vid_status[v1]==1 && pnet.vid_status[v2]==5 && pnet.vid_status[v3]==2){//Us-Io-Ut
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context_CDUsSim,v1,v3,dim,negative_samples,alpha,reg);	

			}else if (pnet.vid_status[v1]==2 && pnet.vid_status[v2]==5 && pnet.vid_status[v3]==1){//Ut-Io-Us
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg);	
				pnet.UpdateTPair(w_vertex,w_context_CDUtSim,v1,v3,dim,negative_samples,alpha,reg);	
			//up to here: CDUSim Separation
			}else if (pnet.vid_status[v1]==3 && pnet.vid_status[v2]==1 && pnet.vid_status[v3]==5){//Is-Us-Io
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii

			}else if (pnet.vid_status[v1]==5 && pnet.vid_status[v2]==1 && pnet.vid_status[v3]==3){//Io-Us-Is
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii

			}else if (pnet.vid_status[v1]==4 && pnet.vid_status[v2]==2 && pnet.vid_status[v3]==5){//It-Ut-Io
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii

			}else if (pnet.vid_status[v1]==5 && pnet.vid_status[v2]==2 && pnet.vid_status[v3]==4){//Io-Ut-It
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii
			//up to here: CDISim Separation	
			}else if (pnet.vid_status[v1]==5 && pnet.vid_status[v2]==2 && pnet.vid_status[v3]==5){//Io-Ut-Io
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii
			//up to here: Sim Bridge
			}else{
				pnet.UpdateTPair(w_vertex,w_context,v1,v2,dim,negative_samples,alpha,reg); //ui
				pnet.UpdateTPair(w_vertex,w_context,v2,v1,dim,negative_samples,alpha,reg); //iu
				pnet.UpdateTPair(w_vertex,w_context,v1,v3,dim,negative_samples,alpha,reg); //ii
			}

			count++;
			if (count % MONITOR == 0)
			{
				_alpha = alpha* ( 1.0 - (double)(current_sample)/total_sample_times ); // learning rate decade
				if (_alpha < alpha_min) _alpha = alpha_min;
				alpha_last = _alpha;
				current_sample += MONITOR;
				printf("\tAlpha: %.6f\tProgress: %.3f %%%c", _alpha, (double)(current_sample)/total_sample_times * 100, 13);
				fflush(stdout);
			}
		}
	}
	printf("\tAlpha: %.6f\tProgress: 100.00 %%\n", alpha_last);
}
