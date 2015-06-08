////O(nk^2mlogm+nk^2)
////n为轨迹点数
////m为路段数
////k为某个轨迹点的候选点数
//
#include "stdafx.h"
#include "IVMM.h"
using namespace std;

string dbname,dbport,dbaddr,roadTN;
double R,Sigma;
int K;

bool readConfig(){
	ifstream fin;
	fin.open("config.ini");
	if(!fin.is_open()){
		cerr <<"can't open config.ini"<<endl;
		return false;
	}
	int cnt = 0;
	string line,prop;
	while(fin >> line){
		if(line == "" || line[0] == '#')
			continue;
		prop = line;
		fin>>line;
		fin>>line;
		cnt++;
		if(prop == "dbname")
			dbname = line;
		else if(prop == "dbport"){
			dbport = line;
		}
		else if(prop == "dbaddr"){
			dbaddr = line;
		}
		else if(prop == "roadTN"){
			roadTN = line;
		}
		else if(prop == "threadnum"){
			int tmp;
			sscanf_s(line.c_str(),"%d",&tmp);
			//threadNum = tmp;
		}
		else if(prop == "K"){
			int tmp;
			sscanf_s(line.c_str(),"%d",&tmp);
			K = tmp;
		}
		else if(prop == "R"){
			double tmp;
			sscanf_s(line.c_str(),"%lf",&tmp);
			R = tmp;
		}
		else if(prop == "sigma"){
			double tmp;
			sscanf_s(line.c_str(),"%lf",&tmp);
			Sigma = tmp;
		}
	}
	fin.close();
	if(cnt == 8) 
		return true;
	else{
		cerr<<"config.ini corrupted！"<<endl;
		return false;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	IVMM MapMatching;
	MapMatching.batchMatching("E:/research/data/out_20131017.csv");
	return 0;
}