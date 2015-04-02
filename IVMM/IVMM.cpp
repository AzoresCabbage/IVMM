//O(nk^2mlogm+nk^2)
//n为轨迹点数
//m为路段数
//k为某个轨迹点的候选点数

#include "stdafx.h"
#include "geometry.h"
#include "Graph.h"
#include "database.h"
using namespace std;

//#define MYDEBUG
#define BUFFSIZE 5000000
class MAT{
public:
	double **mat;
	int n,m;
	MAT():n(0),m(0),mat(NULL){}
	MAT(int _n,int _m):n(_n),m(_m),mat(NULL){
		mat = new double*[n];
		for(int i=0;i<n;++i){
			mat[i] = new double[m];
			memset(mat[i],0,sizeof(double)*m);
		}
	}
	MAT& operator = (const MAT& x){
		double** matOri = mat;
		mat = new double*[x.n];
		for(int i=0;i<x.n;++i){
			mat[i] = new double[x.m];
			for(int j=0;j<x.m;++j)
				mat[i][j] = x.mat[i][j];
		}
		if(n && m && matOri != NULL){
			for(int i=0;i<n;++i)
				delete[] matOri[i];
			delete[] matOri;
		}
		n = x.n;
		m = x.m;
		return *this;
	}
	~MAT(){
		if(n && m && mat!=NULL){
			for(int i=0;i<n;++i)
				delete[] mat[i];
			delete[] mat;
		}
	}
};

////////////////////config////////////////////////
string dbname = "osm";//数据库名称
string dbport = "5432";//数据库端口号
string dbaddr = "127.0.0.1";//数据库地址
string roadTN = "network";//道路表名称
int threadNum = 3;//用于计算的线程数量
int getCandiPointThreadNum = 1;
double R = 50;//选取某轨迹点的候选点的范围，单位为m,对数据预处理时删去间距小于50的点
double Sigma = 10;//正态分布，单位为m
double miu = 5;
int K = 5;//候选点最多的数量
double beta = 7000;//m
char buffer[BUFFSIZE];
//////////////////变量定义/////////////////////////
//variable
time_t tm;//计时变量
double Coef;//norm distribution coef
vector <GeoPoint> P;//轨迹点
vector < vector <Point> > candiPoint;//每个轨迹点的候选点集合
vector <MAT> M;
vector <int> vote;
vector <double> fvalue;
//F是总代价:F = Fs*Ft
map < pair<int,int> , double > F;

Database DB;
Graph *network;
//////////////////End/////////////////////////

//////////not main logic func/////////////////
bool readConfig();
bool getTaxiTrajectory(string filePath);
void writeToDB(const vector <Point>& Traj);
//////////////////////////////////////////////

//初始化
//读入轨迹，建立点的映射
bool init(string basePath){
	//network.reset();
	Coef = 1/(sqrt(2*PI)*Sigma);
	P.clear();
	candiPoint.clear();
	F.clear();
	network->reset();
	//splitTrajectory(basePath);
	return getTaxiTrajectory(basePath);
}

double f(double x){
	return exp(-SQR(x/beta));
}

//正态分布
double N(int i,int t){
	double x = getGeoDis(P[i],candiPoint[i][t]);
	return Coef*exp(-SQR(x-miu)/(2*SQR(Sigma)));
}

//Transmission Probability
//if t == s then return 1; because t must transmit s
double V(double d,Point t,Point s){
	if(t == s) return 1;
	double tmp = network->getCandiShortest(t,s);
	return d/tmp;
}

mutex lock_it;
mutex lockVote;
int it;
DWORD WINAPI calc_candiPoint(LPVOID ptr){
	int upd = (int)P.size();
	int cur;
	while(true){
		lock_it.lock();
		if(it >= upd) {
			lock_it.unlock();
			return 0;
		}
		cur = it;
		++it;
		lock_it.unlock();
		try{
			vector <Point> tmp = network->getCandidate(P[cur],R,K);
			candiPoint[cur] = tmp;
		}
		catch (exception e){
			printf("%d error!",cur);
		}
		
	}
}

//传入轨迹上每个点的候选点集合
//计算F,Fs,Ft
vector <Point> FindMatchedSequence(int i,int k,vector <MAT>& fi_i,vector <double>& Wi){
	tm = clock();
	//std::cerr<<"start FindMatchedSequence "<<i<<" "<<k<<endl;
	vector <Point> res;

	int totCandiNum = network->totCandiPoint;
	double *fmx = new double[totCandiNum];
	int *pre = new int[totCandiNum];
	
	for(int t=0;t<totCandiNum;++t)
		fmx[t] = -1-1e300;

	for(int t=0;t<candiPoint[0].size();++t)
		fmx[candiPoint[0][t].id] = Wi[0]*N(0,t);

	if(i == 0){//i = 0,set init value be -max because there is no fi[i][0],only have fi[i][1]
		for(int t=0;t<candiPoint[0].size();++t){
			if(t == k) continue;
			fmx[candiPoint[0][t].id] = -1e300;
		}
	}
	else{
		for(int s = 0;s<candiPoint[i].size();++s){
			if(s == k) continue;
			int tSz = 0;
			if(i == 0) tSz = (int)candiPoint[i].size();
			else tSz = (int)candiPoint[i-1].size();
			for(int t=0;t<tSz;++t){
				fi_i[i].mat[t][s] = -1e300;
			}
		}
	}

	int pNum = (int)P.size();
	for(int j=1;j<pNum;++j){
		/*cout<<j<<" ";
		cout<<fi[i][j-1].n<<" "<<fi[i][j-1].m<<" ";
		cout<<candiPoint[j].size()<<" "<<candiPoint[j-1].size()<<endl;*/
		for(int s = 0;s<candiPoint[j].size();++s){
			for(int t=0;t<candiPoint[j-1].size();++t){

				//assert(candiPoint[j][s].id < totCandiNum);
				//assert(candiPoint[j-1][t].id < totCandiNum);
				//assert(t < fi[i][j].n);
				//assert(s < fi[i][j].m);
				double fs = fmx[candiPoint[j][s].id];
				double ft = fmx[candiPoint[j-1][t].id];
				/*assert(t < fi[i][j].n);
				assert(s < fi[i][j].m);
				assert(i < fi.size());
				assert(j < fi[i].size());
				cout<<"n = "<<fi[i][j].n<<" m = "<<fi[i][j].m<<endl;
				cout<<"i = "<<i<<" j = "<<j<<endl;*/
				double fijts = fi_i[j].mat[t][s];
				if(fs < ft+fijts){
					fmx[candiPoint[j][s].id] = ft+fijts;
					pre[candiPoint[j][s].id] = candiPoint[j-1][t].id;
				}
			}
		}
	}

	double mx = fmx[candiPoint[pNum-1][0].id];
	int c = candiPoint[pNum-1][0].id;
	for(int s=0;s<candiPoint[pNum-1].size();++s){
		if(mx < fmx[candiPoint[pNum-1][s].id]){
			mx = fmx[candiPoint[pNum-1][s].id];
			c = candiPoint[pNum-1][s].id;
		}
	}
	fvalue[candiPoint[i][k].id] = mx;
	for(int s = pNum-1;s>0;--s){
		res.push_back(network->getCandiPointById(c));
		c = pre[c];
	}
	res.push_back(network->getCandiPointById(c));
	delete[] pre;
	delete[] fmx;

	reverse(res.begin(),res.end());
	//std::cerr<<"FindMatchedSequence cost = "<<clock()-tm<<"ms"<<endl;
	return res;
}

DWORD WINAPI interactiveVoting(LPVOID ptr){
	int upbound = (int)P.size();
	int cur = 0;
	while(true)
	{
		lock_it.lock();
		if(it >= upbound) {
			lock_it.unlock();
			return 0;
		}
		cur = it;
		++it;
		lock_it.unlock();
#ifdef MYDEBUG
		fprintf(stderr,"W[%d] = ",cur);
#endif
		//fprintf(stderr,"%d start voting...\n",cur);
		vector <double> W;
		W.resize(upbound);
		for(int j=0;j<upbound;++j){
			W[j] = f(getGeoDis(P[cur],P[j]));
#ifdef MYDEBUG
			fprintf(stderr,"%lf ",W[j]);
#endif
		}
#ifdef MYDEBUG
		fprintf(stderr,"\n");

		fprintf(stderr,"fi[%d] = ",cur);
#endif
		vector <MAT> fi;
		fi.resize(upbound);
		for(int j=1;j<upbound;++j){//j indicate M^j , not exist M^1
			MAT tMat(M[j].n,M[j].m);
#ifdef MYDEBUG
			fprintf(stderr,"j = %d\n",j);
#endif
			//assert(j < M.size());
			if(j-1 < cur){
				for(int t = 0;t<tMat.n;++t){
					for(int s=0;s<tMat.m;++s){
						tMat.mat[t][s] = W[j-1]*M[j].mat[t][s];
#ifdef MYDEBUG
						fprintf(stderr,"%lf ",tMat.mat[t][s]);
#endif
					}
#ifdef MYDEBUG
					fprintf(stderr,"\n");
#endif
				}
			}
			else{
				for(int t = 0;t<tMat.n;++t){
					for(int s=0;s<tMat.m;++s){
						tMat.mat[t][s] = W[j]*M[j].mat[t][s];
#ifdef MYDEBUG
						fprintf(stderr,"%lf ",tMat.mat[t][s]);
#endif
					}
#ifdef MYDEBUG
					fprintf(stderr,"\n");
#endif
				}
			}
			fi[j] = tMat;
		}
		
		for(int j=0;j<candiPoint[cur].size();++j){
			vector <MAT> tFi;
			tFi.resize(fi.size());
			for(int i=0,num=(int)fi.size();i<num;++i)
				tFi[i] = fi[i];
			vector <Point> Seq = FindMatchedSequence(cur,j,tFi,W);
			lockVote.lock();
#ifdef MYDEBUG
			cerr<<"candi id = "<<candiPoint[cur][j].id<<endl;
#endif
			for(int k=0;k<Seq.size();++k)
			{
#ifdef MYDEBUG
				cerr<<Seq[k].id<<" ";
#endif
				++ vote[Seq[k].id];
			}
#ifdef MYDEBUG
			cerr<<endl;
#endif
			lockVote.unlock();
		}
	}
}

vector <Point> IVMM(){
	//O(nm) for every point in Trajectory find the candipoint set
	std::cerr<<"start getCandiPoint"<<endl;
	tm = clock();

	//////////并行计算candipoint，初始化it
	it= 0;
	HANDLE *handle = new HANDLE[getCandiPointThreadNum];
	for(int i=0;i<getCandiPointThreadNum;++i){
		handle[i] = CreateThread(NULL,0,calc_candiPoint,NULL,0,NULL);
	}
	for(int i=0;i<getCandiPointThreadNum;++i){
		WaitForSingleObject(handle[i],INFINITE);
	}
	delete[] handle;
	std::cerr<<"getCandidate cost = "<<clock()-tm<<"ms"<<endl;
	//删去为候选点为0的点
	try{
		vector <int> noCandiPointIdx;
		for(int i=0,pNum = (int)P.size();i<pNum;++i){
			if(candiPoint[i].empty()){
				noCandiPointIdx.push_back(i);
			}
		}
		for(int i=0,num = (int)noCandiPointIdx.size();i < num;++i){
			P.erase(P.begin()+noCandiPointIdx[i]-i);
			candiPoint.erase(candiPoint.begin()+noCandiPointIdx[i]-i);
		}
	}
	catch (exception e){
		fprintf(stderr,"erase point error! %s\n",e);
	}

	DB.loadInitPoint(P);
	DB.loadCandiPoint(candiPoint);

	int pNum = (int)P.size();

	//计算M矩阵,不计算Ft
	//M[i][j]表示只考虑两个候选点i,j时,当j是由i转移过来并且为正确点的概率
	M.resize(pNum);
	//特殊处理第一个矩阵为单位阵表示从自己到自己为正确点的概率为1
	MAT mat((int)candiPoint[0].size(),(int)candiPoint[0].size());
	for(int i=0;i<mat.n;++i)
		for(int j=0;j<mat.m;++j)
			mat.mat[i][j] = i==j?1:0;
	M[0] = mat;

	std::cerr<<"start calc Matrix M"<<endl;
	for(int i=1;i<pNum;++i){
		//std::cerr<<"start calc Matrix M %d\n"<<i<<endl;
		int nPre = (int)candiPoint[i-1].size();
		int nCur = (int)candiPoint[i].size();
		MAT tMat(nPre,nCur);
#ifdef MYDEBUG
		fprintf(stderr,"M[%d] = \n",i);
#endif
		for(int t = 0;t<nPre;++t){
			for(int s=0;s<nCur;++s){
				tMat.mat[t][s] = N(i,s)
					*V(getGeoDis(P[i-1],P[i]),candiPoint[i-1][t],candiPoint[i][s]);
#ifdef MYDEBUG
				fprintf(stderr,"%lf ",tMat.mat[t][s]);
#endif
			}
#ifdef MYDEBUG
			fprintf(stderr,"\n");
#endif
		}
		M[i] = tMat;
	}
	std::cerr << "calc Matrix M cost = "<<clock() - tm<<"ms"<<endl;

	//calc seq
	int totCandiNum = network->totCandiPoint;
	fvalue.resize(totCandiNum);
	vote.resize(totCandiNum);

	/////////////并行计算，初始化it
	std::cerr<<"start interactive voting"<<endl;
	tm = clock();
	it = 0;
	handle = new HANDLE[threadNum];
	for(int i=0;i<threadNum;++i){
		handle[i] = CreateThread(NULL,0,interactiveVoting,NULL,0,NULL);
	}
	for(int i=0;i<threadNum;++i){
		WaitForSingleObject(handle[i],INFINITE);
	}
	delete[] handle;
	std::cerr<<"interactive voting cost = "<<clock()-tm<<"ms"<<endl;

	vector <Point> res;
	for(int i=0;i<pNum;++i){
		int mx = 0;
		double mxFval = 0;
		int pos = 0;
		for(int j=0;j<candiPoint[i].size();++j){
			if(mx < vote[candiPoint[i][j].id] 
			|| (mx == vote[candiPoint[i][j].id] && mxFval < fvalue[candiPoint[i][j].id])
				){
				mx = vote[candiPoint[i][j].id];
				pos = j;
				mxFval = fvalue[candiPoint[i][j].id];
			}
		}
		res.push_back(candiPoint[i][pos]);
	}
	/*ofstream fout("vote.txt");
	for(int i=0;i<vote.size();++i)
		fout<<vote[i]<<endl;
	fout.close();*/
	return res;
}

vector <Point> dealFlyPoint(vector <Point> Ori){
	vector <Point> res;
	res.push_back(Ori[0]);
	const double LimitV = 25;
	int sz = (int)Ori.size();
	for(int i=1;i<sz;++i){
		double dis = network->getCandiShortest(Ori[i-1],Ori[i]);
		double span = P[i].date - P[i-1].date;
		if(dis / span > LimitV){
			if(i-2>=0 && ( network->getCandiShortest(Ori[i-2],Ori[i])/(P[i].date - P[i-2].date) ) < LimitV){
				res.pop_back();
				res.push_back(Ori[i]);
				continue;
			}
			else 
				continue;
		}
		res.push_back(Ori[i]);
	}
	return res;
}

int _tmain(int argc, _TCHAR* argv[]){
	/*if(!readConfig()){
		cerr << "read config.ini error!" <<endl;
		return 0;
	}*/
	//DB.preProcData();
	/*DB.reOrder();
	system("pause");*/

	network = new Graph(roadTN);

	

	string basePath;
	cerr<<"please input file path:";
	//while(cin>>basePath)
	basePath = "input.txt";
	{
		cerr<<"start init..."<<endl;
		tm = clock();
		if(init(basePath) == false){
			cerr << "can not open file "<< basePath<<endl;
			cerr<<"please input file path:";
			//continue;
		}
		cerr<<"init cost "<<clock()-tm<<"ms"<<endl;

		vector <Point> res = IVMM();
		//res = dealFlyPoint(res);
		cerr<<"start writeToDB->.."<<endl;
		tm = clock();
		writeToDB(res);
		cerr<<"writeToDB cost "<<clock()-tm<<"ms"<<endl;
		cerr<<"please input file path:";
	}
	return 0;
}

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
			threadNum = tmp;
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

bool getTaxiTrajectory(string filePath){
	ifstream fin;
	fin.open(filePath);
	if(!fin) {
		std::cerr<<"File open error!"<<endl;
		fin.close();
		return false;
	}
	else{
		double lat,lon;
		long long carID;
		int y,mon,d,h,min,s;
		char buff[100];
		while(fin.getline(buff,100)){
			sscanf_s(buff,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&min,&s,&lon,&lat);
			P.push_back(GeoPoint(lat,lon,Date(y,mon,d,h,min,s)));
		}
	}
	fin.close();
	candiPoint.resize(P.size());
	return true;
}

//把匹配选中的点，路径写入数据库
void writeToDB(const vector <Point>& Traj){
	//写入选中的候选点
	string SQL = "select * from pg_class where relname = 'trajectory_point'";
	PGresult* res = DB.execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from trajectory_point";
		DB.execUpdate(SQL);
	}
	else{
		SQL = "create table trajectory_point (id integer primary key,way geometry(Point,4326))";
		DB.execUpdate(SQL);
	}
	size_t sz = Traj.size();

	for(int i=0;i<sz;++i){
		sprintf_s(buffer,"insert into trajectory_point values(%d,ST_GeomFromText('Point(%lf %lf)',4326))",i+1,Traj[i].x,Traj[i].y);
		SQL = buffer;
		DB.execUpdate(SQL);
	}

	//写入匹配的轨迹
	SQL = "select * from pg_class where relname = 'trajectory_line'";
	res = DB.execQuery(SQL);
	num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from trajectory_line";
		DB.execUpdate(SQL);
	}
	else{
		SQL = "create table trajectory_line (id integer primary key,way geometry(LineString,4326))";
		DB.execUpdate(SQL);
	}
	sz = Traj.size();
	int ID = 0;
	int cnt = 0;
	sprintf_s(buffer,"insert into trajectory_line values(%d,ST_GeomFromText('LineString(%.6f %.6f"
		,ID++,Traj[0].x,Traj[0].y);
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
			vector <int> path = network->getPath(Traj[i-1],Traj[i]);

			if( ! path.empty()) {
				//Point cur(network->getPointById(path[1]));
				//sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%lf %lf",cur.x,cur.y);
				size_t psz = path.size();
				for(int j=psz-2;j>0;--j){
					Point cur = network->getPointById(path[j]);
					cnt = strlen(buffer);
					sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",cur.x,cur.y);
				}
				cnt = strlen(buffer);
				//cout<<cnt<<endl;
				sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",Traj[i].x,Traj[i].y);
			}
			cnt = strlen(buffer);
			if(cnt == baseLen){
				sprintf_s(buffer+cnt,BUFFSIZE-cnt,",%.6f %.6f",Traj[i].x,Traj[i].y);
			}
			sprintf_s(buffer+cnt,BUFFSIZE-cnt,")',4326))");
			SQL = buffer;
			DB.execUpdate(buffer);
			memset(buffer,0,sizeof(buffer));
			sprintf_s(buffer,"insert into trajectory_line values(%d,ST_GeomFromText('LineString(%.6f %.6f"
				,ID++,Traj[i].x,Traj[i].y);
		}
	}
	//cout<<cnt<<endl;
	//cnt = strlen(buffer);
	//sprintf_s(buffer+cnt,BUFFSIZE-cnt,")',4326))");
	//SQL = buffer;
	//DB.execUpdate(SQL);
}