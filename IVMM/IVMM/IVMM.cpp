#include "stdafx.h"
#include "IVMM.h"
using namespace std;

#define BUFFSIZE 5000000
char buffer[BUFFSIZE];

mutex IVMM::lock_it;
mutex IVMM::lockVote;
int IVMM::it;
double IVMM::R;
double IVMM::Sigma;
double IVMM::miu;
int IVMM::K;
double IVMM::beta;
double IVMM::Coef;

IVMM::IVMM(string _dbname,string _dbport,string _dbaddr,string _roadTN,
		   double _R,double _sigma,double _miu,int _K,double _beta,
		   int _threadNum,int _candiThreadNum):
	dbname(_dbname),
	dbport(_dbport),
	dbaddr(_dbaddr),
	roadTN(_roadTN),
	threadNum(_threadNum),
	getCandiPointThreadNum(_candiThreadNum)
{
	R = _R;
	Sigma = _sigma;
	miu = _miu;
	K = _K;
	beta = _beta;
	DB = new Database();
}

IVMM::~IVMM()
{
	delete DB;
}

//初始化
//读入轨迹，建立点的映射
void IVMM::init(){
	Coef = 1/(sqrt(2*PI)*Sigma);
	data.reset();
}

void IVMM::calc_candiPoint(
	DATA &data
	){
	int upd = (int)data.P.size();
	int cur;
	while(true){
		{
			lock_guard<mutex> _(lock_it);
			if(it >= upd) return ;
			cur = it;
			++it;
		}
		try{
			vector <Point> tmp = data.network->getCandidate(data.P[cur],R,K);
			data.candiPoint[cur] = tmp;
		}
		catch (exception e){
			printf("%d error!",cur);
		}
	}
}


void IVMM::FindMatchedSequence(
	vector <Point>& res,
	int i,
	int k,
	vector <MatrixXd>& fi_i,
	vector <double>& Wi,
	DATA &data
	)
{
	int totCandiNum = data.network->totCandiPoint;
	double *fmx = new double[totCandiNum];
	int *pre = new int[totCandiNum];
	
	for(int t=0;t<totCandiNum;++t)
		fmx[t] = -1-1e300;
	for(int t=0;t<totCandiNum;++t)
		pre[t] = -1;

	for(int t=0,sz=(int)data.candiPoint[0].size();t<sz;++t)
		fmx[data.candiPoint[0][t].id] = Wi[0]*N(0,t,data);
	
	if(i == 0){//i = 0,set init value be -max because there is no fi[i][0],only have fi[i][1]
		for(int t=0,sz=(int)data.candiPoint[0].size();t<sz;++t){
			if(t == k) continue;
			fmx[data.candiPoint[0][t].id] = -1e300;
		}
	}
	
	int pNum = (int)data.candiPoint.size();
	for(int j=1;j<pNum;++j){
		for(int s = 0,ssz=(int)data.candiPoint[j].size();s<ssz;++s){
			for(int t=0,tsz=(int)data.candiPoint[j-1].size();t<tsz;++t){
				double fs = fmx[data.candiPoint[j][s].id];
				double ft = fmx[data.candiPoint[j-1][t].id];
				double fijts = fi_i[j](t,s);
				if(i == j)
				{
					if(s != k)
					{
						fijts = -1e300;
					}
				}
				if(fs < ft+fijts){
					fmx[data.candiPoint[j][s].id] = ft+fijts;
					pre[data.candiPoint[j][s].id] = data.candiPoint[j-1][t].id;
				}
			}
		}
	}
	
	
	double mx = fmx[data.candiPoint[pNum-1][0].id];
	int c = data.candiPoint[pNum-1][0].id;
	for(int s=0,sz=(int)data.candiPoint[pNum-1].size();s<sz;++s){
		if(mx < fmx[data.candiPoint[pNum-1][s].id]){
			mx = fmx[data.candiPoint[pNum-1][s].id];
			c = data.candiPoint[pNum-1][s].id;
		}
	}
	
	res.resize(pNum);
	data.fvalue[data.candiPoint[i][k].id] = mx;
	for(int s = pNum-1;s>0;--s){
		if(c == -1)
		{// error
			delete[] pre;
			delete[] fmx;
			res.clear();
			return ;
		}
		res[s] = data.network->getCandiPointById(c);
		c = pre[c];
	}
	
	if(c == -1)
	{
		delete[] pre;
		delete[] fmx;
		res.clear();
		return ;
	}
	res[0] = data.network->getCandiPointById(c);
	delete[] pre;
	delete[] fmx;
	return ;
}

void IVMM::interactiveVoting(
	DATA& data
	){
	int upbound = (int)data.P.size();
	int cur = 0;
	while(true)
	{
		{
			lock_guard<mutex> _(lock_it);
			if(it >= upbound) return ;
			cur = it;
			++it;
		}
		//fprintf(stderr,"%d start voting...\n",cur);
		vector <double> W;
		W.resize(upbound);
		for(int j=0;j<upbound;++j){
			W[j] = f(Point(data.P[cur]).EucDisTo(data.P[j]));
		}
		vector <MatrixXd> fi;
		fi.resize(upbound);
		for(int j=1;j<upbound;++j){//j indicate M^j , not exist M^1
			if(j-1 < cur){
				fi[j] = W[j-1]*data.M[j];
			}
			else{
				fi[j] = W[j]*data.M[j];
			}
		}

		for(int j=0,candiSize=(int)data.candiPoint[cur].size();j<candiSize;++j){
			vector <Point> Seq;
			FindMatchedSequence(Seq,cur,j,fi,W,data);
			
			lock_guard<mutex> _(lockVote);
			for(int k=0,sz=(int)Seq.size();k<sz;++k)
			{
				++ data.vote[Seq[k].id];
			}
		}
	}
}

void IVMM::solve(vector <Point>& res){
	//O(nm) for every point in Trajectory find the candipoint set
	std::cerr<<"start getCandiPoint"<<endl;
	tm = clock();

	//////////并行计算candipoint，初始化it
	it = 0;
	thread *threadGetCandi = new thread[getCandiPointThreadNum];
	for(int i=0;i<getCandiPointThreadNum;++i)
		threadGetCandi[i] = thread(calc_candiPoint,ref(data));
	for(int i=0;i<getCandiPointThreadNum;++i)
		threadGetCandi[i].join();
	delete []threadGetCandi;
	std::cerr<<"getCandidate cost = "<<clock()-tm<<"ms"<<endl;

	int pNum = (int)data.P.size();

	//删去为候选点为0的点
	try{
		vector <int> noCandiPointIdx;
		for(int i=0;i<pNum;++i){
			if(data.candiPoint[i].empty()){
				noCandiPointIdx.push_back(i);
			}
		}
		if(noCandiPointIdx.size() == pNum)
			exit(-1);
		for(int i=(int)noCandiPointIdx.size()-1;i >= 0;--i){
			data.P.erase(data.P.begin()+noCandiPointIdx[i]);
			data.candiPoint.erase(data.candiPoint.begin()+noCandiPointIdx[i]);
		}
	}
	catch (exception e){
		fprintf(stderr,"erase point error! %s\n",e);
	}

	//DB->loadInitPoint(data.P);
	//DB->loadCandiPoint(data.candiPoint);

	//更新pNum
	pNum = (int)data.P.size();

	//计算M矩阵,不计算Ft
	//M[i][j]表示只考虑两个候选点i,j时,当j是由i转移过来并且为正确点的概率
	data.M.resize(pNum);
	//特殊处理第一个矩阵为单位阵表示从自己到自己为正确点的概率为1
	int candi0size = (int)data.candiPoint[0].size();
	MatrixXd mat = MatrixXd::Identity(candi0size,candi0size);

	data.M[0] = mat;

	std::cerr<<"start calc Matrix M"<<endl;
	for(int i=1;i<pNum;++i){
		int nPre = (int)data.candiPoint[i-1].size();
		int nCur = (int)data.candiPoint[i].size();
		MatrixXd tMat(nPre,nCur);
		for(int t = 0;t<nPre;++t){
			for(int s=0;s<nCur;++s){
				tMat(t,s) = N(i,s,data)
					*V(Point(data.P[i-1]).EucDisTo(Point(data.P[i])),data.candiPoint[i-1][t],data.candiPoint[i][s]);
			}
		}
		data.M[i] = tMat;
	}
	std::cerr << "calc Matrix M cost = "<<clock() - tm<<"ms"<<endl;

	//calc seq
	int totCandiNum = data.network->totCandiPoint;
	data.fvalue.resize(totCandiNum);
	data.vote.resize(totCandiNum);

	/////////////并行计算，初始化it
	std::cerr<<"start interactive voting"<<endl;
	tm = clock();
	it = 0;
	thread *threadVote = new thread[threadNum];
	for(int i=0;i<threadNum;++i)
		threadVote[i] = thread(interactiveVoting,ref(data));
	for(int i=0;i<threadNum;++i)
		threadVote[i].join();
	delete []threadVote;
	std::cerr<<"interactive voting cost = "<<clock()-tm<<"ms"<<endl;

	for(int i=0;i<pNum;++i){
		int mx = 0;
		double mxFval = 0;
		int pos = 0;
		for(int j=0,candiSize=(int)data.candiPoint[i].size();j<candiSize;++j){
			if(mx < data.vote[data.candiPoint[i][j].id] 
			|| (mx == data.vote[data.candiPoint[i][j].id] && mxFval < data.fvalue[data.candiPoint[i][j].id])
				){
				mx = data.vote[data.candiPoint[i][j].id];
				pos = j;
				mxFval = data.fvalue[data.candiPoint[i][j].id];
			}
		}
		res.push_back(data.candiPoint[i][pos]);
	}
	return ;
}

void IVMM::dealFlyPoint(vector <Point>& Ori,vector <Point>& res){
	res.push_back(Ori[0]);
	const double LimitV = 25;
	int sz = (int)Ori.size();
	for(int i=1;i<sz;++i){
		double dis = data.network->getCandiShortest(Ori[i-1],Ori[i]);
		double span = data.P[i].date - data.P[i-1].date;
		if(dis / span > LimitV){
			if(i-2>=0 && ( data.network->getCandiShortest(Ori[i-2],Ori[i])/(data.P[i].date - data.P[i-2].date) ) < LimitV){
				res.pop_back();
				res.push_back(Ori[i]);
				continue;
			}
			else 
				continue;
		}
		res.push_back(Ori[i]);
	}
	return ;
}

void IVMM::batchMatching(string filepath)
{
	double lat,lon;
	int y,mon,d,h,min,s;
	char buff[100];

	init();
	//ifstream fin("E:/research/data/out_20131017GPS/13311300594.csv");
	//ifstream fin("E:/research/data/day.csv");
	//ifstream fin("input.txt");
	ifstream fin(filepath.c_str());
	fin.getline(buff,100);
	sscanf_s(buff,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&min,&s,&lon,&lat);
	data.P.push_back(GeoPoint(lat,lon,Date(y,mon,d,h,min,s)));
	long long pre = carID;
	while(fin.getline(buff,100)){
		sscanf_s(buff,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&min,&s,&lon,&lat);
		if(pre == carID)
		{
			data.P.push_back(GeoPoint(lat,lon,Date(y,mon,d,h,min,s)));
		}
		else
		{
			data.candiPoint.resize(data.P.size());
			vector <Point> tmp,res;
			solve(tmp);
			puts("start deal fly point");
			dealFlyPoint(tmp,res);
			puts("start write to db");
			writeToDB(res,::to_string(pre));
			printf("tra %lld done!\n",pre);
			init();
			data.P.push_back(GeoPoint(lat,lon,Date(y,mon,d,h,min,s)));
			pre = carID;
		}
	}
	data.candiPoint.resize(data.P.size());
	vector <Point> tmp,res;
	solve(tmp);
	puts("start deal fly point");
	dealFlyPoint(tmp,res);
	puts("start write to db");
	writeToDB(res,::to_string(carID));
	init();

	fin.close();
}

bool IVMM::getTaxiTrajectory(string filePath){
	ifstream fin;
	fin.open(filePath);
	if(!fin) {
		std::cerr<<"File open error!"<<endl;
		fin.close();
		return false;
	}
	else{
		double lat,lon;
		int y,mon,d,h,min,s;
		char buff[100];
		while(fin.getline(buff,100)){
			sscanf_s(buff,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&min,&s,&lon,&lat);
			data.P.push_back(GeoPoint(lat,lon,Date(y,mon,d,h,min,s)));
		}
	}
	fin.close();
	data.candiPoint.resize(data.P.size());
	return true;
}

//把匹配选中的点，路径写入数据库
void IVMM::writeToDB(vector <Point>& Traj,string tbname){
	//写入选中的候选点
	string SQL = "select * from pg_class where relname = 'point_" + tbname + "'";
	PGresult* res = DB->execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from point_" + tbname;
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table point_" + tbname + " (id integer primary key,way geometry(Point,4326))";
		DB->execUpdate(SQL);
	}
	int sz = (int)Traj.size();

	for(int i=0;i<sz;++i){
		sprintf_s(buffer,"insert into point_%s values(%d,ST_GeomFromText('Point(%lf %lf)',4326))",
			tbname.c_str(),
			i+1,
			Traj[i].getLon(),
			Traj[i].getLat() );
		SQL = buffer;
		DB->execUpdate(SQL);
	}

	//写入匹配轨迹的折线
	SQL = "select * from pg_class where relname = 'polyline_" + tbname + "'";
	res = DB->execQuery(SQL);
	num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from polyline_" + tbname;
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table polyline_" + tbname + " (id integer primary key,way geometry(LineString,4326))";
		DB->execUpdate(SQL);
	}
	for(int i=1;i<sz;++i)
	{
		sprintf_s(buffer,"insert into polyline_%s values(%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",
			tbname.c_str(),
			i,
			Traj[i-1].getLon(),
			Traj[i-1].getLat(),
			Traj[i].getLon(),
			Traj[i].getLat() );
		SQL = buffer;
		DB->execUpdate(SQL);
	}

	//写入匹配的轨迹
	SQL = "select * from pg_class where relname = 'line_" + tbname + "'";
	res = DB->execQuery(SQL);
	num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from line_"+tbname;
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table line_" + tbname + " (id integer primary key,way geometry(LineString,4326))";
		DB->execUpdate(SQL);
	}
	sz = (int)Traj.size();
	int ID = 0;
	int cnt = 0;
	sprintf_s(buffer,"insert into line_%s values(%d,ST_GeomFromText('LineString(%.6f %.6f",
		tbname.c_str(),
		ID++,
		Traj[0].getLon(),
		Traj[0].getLat()
		);
	int baseLen = strlen(buffer);
	for(int i=1;i<sz;++i)
	{
		//if(network->isInSameSeg(Traj[i-1].id,Traj[i].id))
		//{
		//	//in same seg
		//	sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%lf %lf",Traj[i].x,Traj[i].y);
		//}
		//else
		{
			vector <int> path = data.network->getPath(Traj[i-1],Traj[i]);

			if( ! path.empty()) {
				//Point cur(network->getPointById(path[1]));
				//sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%lf %lf",cur.x,cur.y);
				size_t psz = path.size();
				for(int j=psz-2;j>0;--j){
					Point cur = data.network->getPointById(path[j]);
					cnt = strlen(buffer);
					sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",
						cur.getLon(),
						cur.getLat()
						);
				}
				cnt = strlen(buffer);
				//cout<<cnt<<endl;
				sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",
					Traj[i].getLon(),
					Traj[i].getLat()
					);
			}
			cnt = strlen(buffer);
			if(cnt == baseLen){
				sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",
					Traj[i].getLon(),
					Traj[i].getLat()
					);
			}
			sprintf_s(buffer+cnt,BUFFSIZE-cnt,")',4326))");
			SQL = buffer;
			DB->execUpdate(buffer);
			memset(buffer,0,sizeof(buffer));
			sprintf_s(buffer,"insert into line_%s values(%d,ST_GeomFromText('LineString(%.6f %.6f",
				tbname.c_str(),
				ID++,
				Traj[i].getLon(),
				Traj[i].getLat()
				);
		}
	}
}

void IVMM::writeToFile(vector <Point>& Traj,string path)
{
	ofstream fout(path.c_str());
	for(int i=0,sz=Traj.size();i<sz;++i)
	{
		sprintf_s(buffer,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",
			carID,
			data.P[i].date.year,
			data.P[i].date.month,
			data.P[i].date.day,
			data.P[i].date.hour,
			data.P[i].date.minute,
			data.P[i].date.second,
			Traj[i].getLon(),
			Traj[i].getLat());
		fout<<buffer<<endl;
	}
	fout.close();
}

double IVMM::f(double x){
	return exp(-SQR(x/beta));
}

//正态分布
double IVMM::N(int i,int t,DATA& data){
	double x = data.candiPoint[i][t].EucDisTo(data.P[i]);
	return Coef*exp(-SQR(x-miu)/(2*SQR(Sigma)));
}

//Transmission Probability
//if t == s then return 1; because t must transmit s
double IVMM::V(double d,Point& t,Point& s){
	if(t == s) return 1;
	double tmp = data.network->getCandiShortest(t,s);
	return d/tmp;
}