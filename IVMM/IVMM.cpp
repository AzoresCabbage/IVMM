//O(nk^2mlogm+nk^2)
//n为轨迹点数
//m为路段数
//k为某个轨迹点的候选点数

#include "stdafx.h"
#include "database.h"
#include "geometry.h"
#include "Graph.h"
using namespace std;

class MAT{
public:
	double **mat;
	int n,m;
	MAT(){}
	MAT(int _n,int _m):n(_n),m(_m){
		mat = new double*[n];
		for(int i=0;i<n;++i){
			mat[i] = new double[m];
			memset(mat[i],0,sizeof(double)*m);
		}
	}
	/*~MAT(){
		for(int i=0;i<n;++i)
			delete[] mat[i];
		delete[] mat;
	}*/
};

////////////////////config////////////////////////
string dbname = "osm";//数据库名称
string dbport = "5432";//数据库端口号
string dbaddr = "127.0.0.1";//数据库地址
string roadTN = "network";//道路表名称
int threadNum = 3;//用于计算的线程数量
double R = 0.01;//选取某轨迹点的候选点的范围，单位为Km
double Sigma = 0.01;//正态分布，单位为Km
double miu = 0.005;
int K = 5;//候选点最多的数量
double beta = 7;//km

//////////////////变量定义/////////////////////////
//variable
time_t tm;//计时变量
double Coef;//norm distribution coef
vector <GeoPoint> P;//轨迹点
vector < vector <Point> > candiPoint;//每个轨迹点的候选点集合
vector <MAT> M;
vector < vector <double> > W;
vector < vector <MAT> > fi;
vector <int> vote;
vector <double> fvalue;
//F是总代价:F = Fs*Ft
map < pair<int,int> , double > F;


Database *DB;//数据库连接实例
Graph *network;

//////////////////End/////////////////////////
//////////not main logic func/////////////////
vector < pair<double,double> > parseString(string str);
bool readConfig();
void loadInitPoint();
void loadCandiPoint();
//////////////////////////////////////////////

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

double f(double x)
{
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
	return d/network->getCandiShortest(t,s);
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

		vector <Point> tmp = network->getCandidate(Point(P[cur].longitude,P[cur].latitude),R,K);
		
		candiPoint[cur] = tmp;
	}
}

//传入轨迹上每个点的候选点集合
//计算F,Fs,Ft
//#define FT
vector <Point> FindMatchedSequence(int i,int k){
	tm = clock();
	std::cerr<<"start FindMatchedSequence "<<i<<" "<<k<<endl;
	vector <Point> res;

	int totCandiNum = network->totCandiPoint;
	double *fmx = new double[totCandiNum];
	int *pre = new int[totCandiNum];

	for(int t=0;t<candiPoint[0].size();++t)
		fmx[candiPoint[0][t].id] = W[i][0]*N(0,t);

	for(int s = 0;s<candiPoint[i].size();++s){
		if(s == k) continue;
		int tSz = 0;
		if(i == 0) tSz = (int)candiPoint[i].size();
		else tSz = (int)candiPoint[i-1].size();
		for(int t=0;t<tSz;++t){
			fi[i][i].mat[t][s] = -DBL_MAX_10_EXP;
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

				if(fmx[candiPoint[j][s].id] < fmx[candiPoint[j-1][t].id]+fi[i][j].mat[t][s]){
					fmx[candiPoint[j][s].id] = fmx[candiPoint[j-1][t].id]+fi[i][j].mat[t][s];
					pre[candiPoint[j][s].id] = candiPoint[j-1][t].id;
				}
			}
		}
	}

	double mx = 0;
	int c = 0;
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
	std::cerr<<"FindMatchedSequence cost = "<<clock()-tm<<"ms"<<endl;
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

		W[cur].resize(upbound);
		for(int j=0;j<upbound;++j){
			W[cur][j] = f(getGeoDis(P[cur],P[j]));
		}

		for(int j=0;j<upbound;++j){
			MAT tMat(M[j].n,M[j].m);
			assert(j < M.size());
			for(int t = 0;t<tMat.n;++t){
				for(int s=0;s<tMat.m;++s){
					tMat.mat[t][s] = W[cur][j]*M[j].mat[t][s];
				}
			}
			fi[cur].push_back(tMat);
		}

		for(int j=0;j<candiPoint[cur].size();++j){
			vector <Point> Seq = FindMatchedSequence(cur,j);
			lockVote.lock();
			for(int k=0;k<Seq.size();++k)
				++ vote[Seq[k].id];
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
	HANDLE *handle = new HANDLE[threadNum];
	for(int i=0;i<threadNum;++i){
		handle[i] = CreateThread(NULL,0,calc_candiPoint,NULL,0,NULL);
	}
	for(int i=0;i<threadNum;++i){
		WaitForSingleObject(handle[i],INFINITE);
	}
	std::cerr<<"getCandidate cost = "<<clock()-tm<<"ms"<<endl;
	loadCandiPoint();


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
		int nPre = (int)candiPoint[i-1].size();
		int nCur = (int)candiPoint[i].size();
		MAT tMat(nPre,nCur);
		for(int t = 0;t<nPre;++t){
			for(int s=0;s<nCur;++s){
				tMat.mat[t][s] = N(i-1,t)
					*V(getGeoDis(P[i-1],P[i]),candiPoint[i-1][t],candiPoint[i][s]);
			}
		}
		M[i] = tMat;
	}
	std::cerr << "calc Matrix M cost = "<<clock() - tm<<"ms"<<endl;

	//计算W矩阵
	//W[i][j]表示p[j]对p[i]的影响
	W.resize(pNum);
	//calc fi
	fi.resize(pNum);
	//calc seq
	int totCandiNum = network->totCandiPoint;
	fvalue.resize(totCandiNum);
	vote.resize(totCandiNum);

	/////////////并行计算，初始化it
	std::cerr<<"start interactive voting"<<endl;
	tm = clock();
	it = 0;
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
	return res;
}

//把匹配选中的点，路径写入数据库
void writeToDB(vector <Point> Traj){
	//写入选中的候选点
	string SQL = "select * from pg_class where relname = 'trajectory_point'";
	PGresult* res = DB->execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from trajectory_point";
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table trajectory_point (id integer primary key,way geometry(Point,4326))";
		DB->execUpdate(SQL);
	}
	size_t sz = Traj.size();
	char buffer[500];

	for(int i=0;i<sz;++i){
		sprintf_s(buffer,"insert into trajectory_point values(%d,ST_GeomFromText('Point(%lf %lf)',4326))",i+1,Traj[i].x,Traj[i].y);
		SQL = buffer;
		DB->execUpdate(SQL);
	}

	//写入匹配的轨迹
	SQL = "select * from pg_class where relname = 'trajectory_line'";
	res = DB->execQuery(SQL);
	num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from trajectory_line";
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table trajectory_line (id integer primary key,src integer,des integer,way geometry(LineString,4326))";
		DB->execUpdate(SQL);
	}
	sz = Traj.size();
	buffer[500];

	int ID = 0;
	for(int i=1;i<sz;++i){
		if(network->isInSameSeg(Traj[i-1].id,Traj[i].id)){
			//in same seg
			sprintf_s(buffer,"insert into trajectory_line values(%d,%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",ID++,Traj[i-1].id,Traj[i].id,Traj[i-1].x,Traj[i-1].y,Traj[i].x,Traj[i].y);
			SQL = buffer;
			DB->execUpdate(SQL);
		}
		else{
			vector <int> path = network->getPath(Traj[i-1],Traj[i]);
			path.pop_back();//path中包含S,T两个点位于一头一尾
			if(path.empty() || path.size() < 2) {
				sprintf_s(buffer,"insert into trajectory_line values(%d,%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",ID++,Traj[i-1].id,Traj[i].id,Traj[i-1].x,Traj[i-1].y,Traj[i].x,Traj[i].y);
				SQL = buffer;
				DB->execUpdate(SQL);
			}
			else{
				Point cur(network->getPointById(path[1]));
				sprintf_s(buffer,"insert into trajectory_line values(%d,%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",ID++,Traj[i-1].id,Traj[i].id,cur.x,cur.y,Traj[i].x,Traj[i].y);
				SQL = buffer;
				DB->execUpdate(SQL);
				size_t psz = path.size();
				for(int j=2;j<psz;++j){
					cur = network->getPointById(path[j]);
					Point pre(network->getPointById(path[j-1]));
					sprintf_s(buffer,"insert into trajectory_line values(%d,%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",ID++,Traj[i-1].id,Traj[i].id,cur.x,cur.y,pre.x,pre.y);
					SQL = buffer;
					DB->execUpdate(SQL);
				}
				cur = network->getPointById(path[psz-1]);
				sprintf_s(buffer,"insert into trajectory_line values(%d,%d,%d,ST_GeomFromText('LineString(%lf %lf,%lf %lf)',4326))",ID++,Traj[i-1].id,Traj[i].id,Traj[i-1].x,Traj[i-1].y,cur.x,cur.y);
				SQL = buffer;
				DB->execUpdate(SQL);
			}

		}
	}
}

int _tmain(int argc, _TCHAR* argv[]){

	//preProcData();
	//cerr<<"over"<<endl;
	/*if(!readConfig()){
		cerr << "read config.ini error!" <<endl;
		return 0;
	}*/

	DB = new Database(dbname,dbport,dbaddr);
	network = new Graph(roadTN);

	//system("pause");

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

		cerr<<"start loadInitPoint..."<<endl;
		tm = clock();
		loadInitPoint();
		cerr<<"loadInitPoint cost "<<clock()-tm<<"ms"<<endl;

		vector <Point> res = IVMM();

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

//把轨迹点存入数据库
void loadInitPoint(){
	string SQL = "select * from pg_class where relname = 'init_point'";
	PGresult* res = DB->execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from init_point";
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table init_point (id integer primary key,year integer,month integer,day integer,hour integer,minute integer,second integer,way geometry(Point,4326))";
		DB->execUpdate(SQL);
	}
	size_t sz = P.size();
	char buffer[500];

	for(int i=0;i<sz;++i){
		sprintf_s(buffer,"insert into init_point values(%d,%d,%d,%d,%d,%d,%d,ST_GeomFromText('Point(%lf %lf)',4326))",i+1,P[i].date.year,P[i].date.month,P[i].date.day,P[i].date.hour,P[i].date.minute,P[i].date.second,P[i].longitude,P[i].latitude);
		SQL = buffer;
		DB->execUpdate(SQL);
	}
}

//把候选点存入数据库
void loadCandiPoint(){
	string SQL = "select * from pg_class where relname = 'candi_point'";
	PGresult* res = DB->execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from candi_point";
		DB->execUpdate(SQL);
	}
	else{
		SQL = "create table candi_point (id integer primary key,belong integer,way geometry(Point,4326))";
		DB->execUpdate(SQL);
	}
	size_t sz = candiPoint.size();
	char buffer[500];

	for(int i=0;i<sz;i++){
		size_t candisz = candiPoint[i].size();
		for(int j=0;j<candisz;++j){
			sprintf_s(buffer,"insert into candi_point values(%d,%d,ST_GeomFromText('Point(%lf %lf)',4326))",candiPoint[i][j].id,i+1,candiPoint[i][j].x,candiPoint[i][j].y);
			SQL = buffer;
			DB->execUpdate(SQL);
		}
	}
}

//把数据库中读出的LINESTRING字符串转化为点坐标
//返回点的vector
vector < pair<double,double> > parseString(string str){
	vector < pair<double,double> > res;
	size_t len = str.size();
	int pre = 11;
	double x,y;
	for(int i=0;i<len;++i){
		if(str[i] == ',' || str[i] == ')'){
			string tmp = str.substr(pre,i-pre);
			//cerr<<tmp<<endl;
			sscanf_s(tmp.c_str(),"%lf %lf",&x,&y);
			res.push_back(make_pair(x,y));
			pre = i+1;
		}
	}
	return res;
}

//处理原始数据，把LineString中含有多于两个点的部分拆分后存入数据库
//只需要运行一次
int Insert(string SQL,int id){
	PGresult* res = DB->execQuery(SQL);

	int tupleNum = PQntuples(res);
	int fieldNum = PQnfields(res);

	for(int i=0;i<tupleNum;i++){
		string content = PQgetvalue(res,i,fieldNum-1);
		vector < pair<double,double> > pts = parseString(content);
		size_t sz = pts.size();
		for(int j=1;j<sz;j++){

			SQL = "insert into network values(";
			SQL += std::to_string(id++)+",0,0,";//id,source,target
			SQL += PQgetvalue(res,i,0);//osm_id
			SQL += ",";
			for(int k=1;k<fieldNum;k++){

				char * ss = PQgetvalue(res,i,k);
				if(strlen(ss) == 0) 
					SQL+="NULL";
				else{
					char * name = PQfname(res,k);
					if(strcmp(name,"name") == 0 || strcmp(name,"tags") == 0){
						SQL += "$$";
						SQL += ss;
						SQL += "$$";
					}
					else if(strcmp(name,"way") == 0){
						SQL += "ST_geomFromText('LINESTRING(";
						SQL += std::to_string(pts[j-1].first);
						SQL += " ";
						SQL += std::to_string(pts[j-1].second);
						SQL += ",";
						SQL += std::to_string(pts[j].first);
						SQL += " ";
						SQL += std::to_string(pts[j].second);
						SQL += ")',900913)";
					}
					else{
						SQL += "'";
						SQL += ss;
						SQL += "'";
					}
				}
				if(k+1 == fieldNum) break;
				SQL += ",";
			}
			SQL += ")";
			//cerr<<SQL<<endl;
			DB->execUpdate(SQL);
		}
	}

	PQclear(res);
	return id;
}

void preProcData(){

	string SQL = "select * from pg_class where relname = 'network'";
	PGresult* res = DB->execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from network";
		DB->execUpdate(SQL);
	}
	else{
		SQL = "";
		ifstream fin;
		fin.open("createNetwork.txt");
		string tmp;
		while(fin>>tmp) SQL += tmp;
		fin.close();
		DB->execUpdate(SQL);
	}

	string Field = "";
	SQL = "select * from allroads";
	res = DB->execQuery(SQL);

	int tupleNum = PQntuples(res);
	int fieldNum = PQnfields(res);

	for(int i=0;i<fieldNum-1;i++){
		Field += "allroads.";
		char * name = PQfname(res,i);
		if(strstr(name,":") == NULL) 
			Field += name;
		else{
			Field += "\"";
			Field += name;
			Field += "\"";
		}
		Field += ",";
	}
	Field += "ST_AsText(way) as way";
	PQclear(res);
	int id = 1;
	id = Insert("select "+ Field + " from allroads",id);
}