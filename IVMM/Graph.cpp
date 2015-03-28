#include "stdafx.h"
#include "Graph.h"
using namespace std;

Graph::Graph(string RTN){
	roadTN = RTN;
	constructGraph();
	divideRegion();
}

Graph::~Graph(){
	delete []vis;
	delete []dis;
	delete []pre;
	delete []edge;
	delete []preV;
	n = 0;
	roadTN = "";
}

void Graph::constructGraph(){
	cerr<<"start construct graph"<<endl;
	time_t tm = clock();
	
	char buff[500];
	sprintf_s(buff,"select id,ST_AsText(ST_Transform(the_geom,4326)) from %s_vertices_pgr",roadTN.c_str());
	string SQL = buff;
	PGresult*  res = DB->execQuery(SQL);
	int tupleNum = n = PQntuples(res);
	int id;

	n += 2;//求最短路时最多加两个点：S,T
	edge = new vector < EDGE >[n+1];
	vis = new bool[n+1];
	dis = new double[n+1];
	pre = new int[n+1];
	preV = new double[n+1];

	double x,y;
	for(int i=0;i<tupleNum;i++){
		string tmp = PQgetvalue(res,i,0);
		sscanf_s(tmp.c_str(),"%d",&id);
		tmp = PQgetvalue(res,i,1);
		sscanf_s(tmp.c_str(),"POINT(%lf %lf)",&x,&y);
		idToPoint[id] = Point(x,y);
	}
	PQclear(res);
	
	sprintf_s(buff,"select source,target,ST_length(way),highway,gid,oneway from %s where highway <> '';",roadTN.c_str());
	SQL = buff;
	res = DB->execQuery(SQL);
	tupleNum = PQntuples(res);
	int st,ed;
	double cost;
	double v;
	for(int i=0;i<tupleNum;++i){
		string tmp = PQgetvalue(res,i,0);
		sscanf_s(tmp.c_str(),"%d",&st);

		tmp = PQgetvalue(res,i,1);
		sscanf_s(tmp.c_str(),"%d",&ed);

		tmp = PQgetvalue(res,i,2);
		sscanf_s(tmp.c_str(),"%lf",&cost);
		cost /= 1000.0;//转化为千米

		tmp = PQgetvalue(res,i,3);
		v = judgeV(tmp.c_str())*1000/3600;

		tmp = PQgetvalue(res,i,4);
		sscanf_s(tmp.c_str(),"%d",&id);
		//加边
		//cerr<<st<<" "<<ed<<endl;
		//assert(st <= n);
		//assert(ed <= n);
		edge[st].push_back(EDGE(ed,cost,v,idToPoint[ed]));
		tmp = PQgetvalue(res,i,5);
		if(tmp != "yes")
			edge[ed].push_back(EDGE(st,cost,v,idToPoint[st]));

		Road.push_back(RoadSegment(idToPoint[st],idToPoint[ed],st,ed,id,v,tmp!="yes"?false:true));
		idToidx[id] = i;
	}
	PQclear(res);

	cerr << "construct Graph cost = "<<clock()-tm<<"ms"<<endl;
}

//根据道路类型，返回道路速度，单位为km
double Graph::judgeV(string ss){
	char s[50];
	strcpy_s(s,ss.c_str());
	int len = (int)strlen(s);
	s[len] = '\0';
	if(strcmp(s,"motorway") == 0) return 120;
	if(strcmp(s,"trunk") == 0) return 80;
	if(strcmp(s,"primary") == 0) return 80;
	if(strcmp(s,"secondary") == 0) return 80;
	if(strcmp(s,"tertiary ") == 0) return 60;
	if(strcmp(s,"unclassified") == 0) return 50;
	if(strcmp(s,"residential ") == 0) return 50;
	if(strstr(s,"link")) return 50;
	if(strcmp(s,"service") == 0) return 30;
	if(strcmp(s,"living street") == 0) return 30;
	if(strcmp(s,"pedestrian") == 0) return 10;
	if(strcmp(s,"bicycle road") == 0) return 15;
	if(strcmp(s,"track") == 0) return 10;
	if(strcmp(s,"bus guideway") == 0) return 50;
	if(strcmp(s,"raceway") == 0) return 180;
	if(strcmp(s,"road") == 0) return 50;
	if(strcmp(s,"footway") == 0) return 10;
	if(strcmp(s,"cycleway") == 0) return 15;
	if(strcmp(s,"bridleway") == 0) return 30;
	if(strcmp(s,"steps") == 0) return 5;
	if(strcmp(s,"path") == 0) return 10;
	if(strcmp(s,"sidewalk") == 0) return 10;
	if(strcmp(s,"cycleway lane") == 0) return 15;
	if(strcmp(s,"cycleway tracks") == 0) return 15;
	if(strcmp(s,"busway") == 0) return 50;
	return 10;
}

//dijkstra,返回编号为S到编号为T的最短路长度
double Graph::shortest_path(int src,int des){
	//if(fa[src] != fa[des]) return DBL_MAX;
	for(int i=0;i<=n;++i) dis[i] = DBL_MAX,vis[i] = false;
	pre[src] = -1;

	if(src == des) return 0;
	priority_queue < NODE > Q;
	Q.push(NODE(0,src));
	dis[src] = 0;
	while(!Q.empty()){
		double curDis = Q.top().first;
		int u = Q.top().second;
		Q.pop();
		if(vis[u] == true)continue;
		if(u == des) return dis[des];
		vis[u] = true;
		size_t sz = edge[u].size();
		for(int i=0;i<sz;++i){
			int v = edge[u][i].v;
			double cost = edge[u][i].cost;
			if(!vis[v] && dis[v] > dis[u]+cost){
				pre[v] = u;
				preV[v] = edge[u][i].speed;
				dis[v] = dis[u]+cost;
				Q.push(NODE(dis[v],v));
			}
		}
	}
	return dis[des];
}

//返回终点编号为des的最短路有多少边
int Graph::getSegNum(int des){
	int res = 0;
	while(pre[des] != -1){
		des = pre[des];
		res++;
	}
	return res;
}

//返回终点编号为des的最短路上的点
vector <int> Graph::getSegPoint(int des){
	vector <int> res;
	while(pre[des] != -1){
		res.push_back(des);
		des = pre[des];
	}
	res.push_back(des);
	return res;
}

//返回终点编号为des的最短路上各个路段的速度
vector <double> Graph::getSegSpeed(int des){
	vector <double> res;
	while (pre[des] != -1){
		res.push_back(preV[des]);
		des = pre[des];
	}
	return res;
}

Point Graph::getCandiPointById(int id){
	return allCandiPoint[id];
}

Point Graph::getPointById(int id){
	return idToPoint[id];
}

mutex lock_totCandiPoint;

vector < Point > Graph::getCandidate(Point p,double DIS,int K){

	int regionsz = (int)RegionCen.size();
	priority_queue < pair<double,int> > Q;
	for(int i=0;i<regionsz;++i){
		Q.push(make_pair(getGeoDis(p,RegionCen[i]),i));
		if(Q.size() > 4) Q.pop();
	}

	//ofstream fout("candiPoint.txt",ios::app);
	vector < Point > st;
	vector < TMP > tmp;

	while(!Q.empty()){
		int u = Q.top().second;
		Q.pop();
		size_t sz = RoadIdxOfRegion[u].size();
		for(int j=0;j<sz;++j){
			//对最近点与p距离在R以内的点为候选点，为其分配ID，并加入所有候选点集
			int idx = RoadIdxOfRegion[u][j];//分割的区域u中第j条道路在Road中的下标
			int id = Road[idx].id;//此条道路的id
			Point la = Road[idx].st;
			Point lb = Road[idx].ed;
			double dis = dispToseg(p,la,lb);
			if(dis <= DIS){
				Point pivot(pToseg(p,la,lb));
				tmp.push_back(TMP(dis,pivot,id));
			}
		}
	}
	int tmpsz = (int)tmp.size();
	if(tmpsz > K) 
		sort(tmp.begin(),tmp.end());
	tmpsz = min(K,tmpsz);
	for(int i=0;i<tmpsz;++i){
		Point pivot(tmp[i].pts);

		lock_totCandiPoint.lock();
		pivot.id = totCandiPoint++;
		allCandiPoint.push_back(pivot);
		lock_totCandiPoint.unlock();

		pInSeg[pivot.id] = tmp[i].rid;
		st.push_back(pivot);
		//fout<<pivot.id<<" "<<pivot.x<<" "<<pivot.y<<" "<<tmp[i].rid<<endl;
	}
	//fout<<"END"<<endl;
	//fout.close();
	return st;
}

//返回从t->s的路径上的点
vector <int> Graph::getPath(Point t,Point s){
	getCandiShortest(t,s);
	return getSegPoint(n-2);
}

bool Graph::isInSameSeg(int id1,int id2){
	return pInSeg[id1] == pInSeg[id2];
}

//轨迹点i-1的候选点t到轨迹点i的候选点s的地图最近距离
double Graph::getCandiShortest(Point t,Point s){
	double res = DBL_MAX;
	//如果s,t在同一条线段上，直接返回其距离即可
	int tid = pInSeg[t.id],sid = pInSeg[s.id];
	if(tid == sid){
		tTosSeg = 1;
		return tTosMin = getGeoDis(t,s);
	}

	RoadSegment Road1 = Road[idToidx[tid]];
	RoadSegment Road2 = Road[idToidx[sid]];

	int stID1 = Road1.stID;
	int edID1 = Road1.edID;
	int stID2 = Road2.stID;
	int edID2 = Road2.edID;
	Point st1 = Road1.st;
	Point ed1 = Road1.ed;
	Point st2 = Road2.st;
	Point ed2 = Road2.ed;

	pre[n-1] = pre[n-2] = -1;

	//t's id -> n-1 , s'id -> n-2
	double d1 = getGeoDis(t,st1);
	double d2 = getGeoDis(t,ed1);
	edge[stID1].push_back(EDGE(n-1,d1,Road1.v,t));//src -> t
	edge[n-1].push_back(EDGE(edID1,d2,Road1.v,ed1));//t -> des
	if(!Road1.oneway){
		edge[edID1].push_back(EDGE(n-1,d2,Road1.v,t));//des -> t
		edge[n-1].push_back(EDGE(stID1,d1,Road1.v,st1));//t -> src
	}

	d1 = getGeoDis(s,st2);
	d2 = getGeoDis(s,ed2);
	edge[stID2].push_back(EDGE(n-2,d1,Road2.v,s));//src -> s
	edge[n-2].push_back(EDGE(edID2,d2,Road2.v,ed2));//s -> des
	if(!Road2.oneway){
		edge[edID2].push_back(EDGE(n-2,d2,Road2.v,s));//des -> s
		edge[n-2].push_back(EDGE(stID2,d1,Road2.v,st2));//s -> src
	}

	res = shortest_path(n-1,n-2);

	edge[n-1].clear();
	edge[n-2].clear();
	edge[stID1].pop_back();
	edge[stID2].pop_back();
	if(!Road1.oneway) edge[edID1].pop_back();
	if(!Road2.oneway) edge[edID2].pop_back();

	return res;
}

vector <double> Graph::getSpeed(){
	return MinSpeed;
}

mutex mylock;
int iter;
DWORD WINAPI Graph::calc(LPVOID ptr){
	//cerr<<"hi"<<endl;
	Graph* Ptr = (Graph*)ptr;
	vector <Point> RegionCen = Ptr->RegionCen;
	vector <RoadSegment> Road = Ptr->Road;
	vector <int> *RoadIdxOfRegion = Ptr->RoadIdxOfRegion;

	double R = Ptr->R;
	int upd = (int)RegionCen.size();
	int rsz = (int)Road.size();
	
	int cur;
	while (true){
		mylock.lock();
		if(iter >= upd){
			mylock.unlock();
			return 0;
		}
		cur = iter;
		++ iter;
		mylock.unlock();
		
		Point pp = RegionCen[cur];
		for(int j=0;j<rsz;++j){
			if(R > dispToseg(pp,Road[j].st,Road[j].ed)){
				RoadIdxOfRegion[cur].push_back(j);
			}
		}
		//cerr<<cur<<" done!"<<endl;
	}
	//cerr<<"bye!"<<endl;
}

void Graph::divideRegion(){
	time_t tm = clock();
	cerr <<"start divideRegion"<<endl;

	if(!needDevided && readFile()) {
		cerr<<"divide region cost "<<clock()-tm<<"ms"<<endl;
		return;
	}

	double LonR = (MaxLon - MinLon) / (DIVID_NUM*2);
	double LatR = (MaxLat - MinLat) / (DIVID_NUM*2);
	double LonD = 2*LonR;
	double LatD = 2*LatR;
	for(int i=0;i<DIVID_NUM;++i){
		double x = MinLon + LonR + LonD*i;
		for(int j=0;j<DIVID_NUM;++j){
			double y = MinLat + LatR + LatD*j;
			RegionCen.push_back(Point(x,y));
		}
	}

	R = getGeoDis(Point(MinLon,MinLat),RegionCen[0]);
	
	iter = 0;

	HANDLE handle[threadNum];
	for(int i=0;i<threadNum;++i){
		handle[i] = CreateThread(NULL, 0, calc, this, 0, NULL);  
	}
	for(int i=0;i<threadNum;++i){
		WaitForSingleObject(handle[i], INFINITE); 
	}
	writeToFile();
	cerr<<"divide region cost "<<clock()-tm<<"ms"<<endl;
}

void Graph::reset(){
	allCandiPoint.clear();
	MinSpeed.clear();
	pInSeg.clear();
	totCandiPoint = 0;
}

void Graph::writeToFile(){
	ofstream fout("region");
	int sz = (int)RegionCen.size();
	fout<<sz<<endl;//有几个区域
	for(int i=0;i<sz;++i){
		fout<<RegionCen[i].x<<" "<<RegionCen[i].y<<endl;//区域中心坐标
		int idxSz = (int)RoadIdxOfRegion[i].size();
		fout<<idxSz<<endl;//每个区域有多少道路
		for(int j=0;j<idxSz;++j){
			fout<<RoadIdxOfRegion[i][j]<<" ";//每条道路的编号
		}
	}
	fout.close();
}

bool Graph::readFile(){
	ifstream fin("region");
	try{
		int sz;
		int val;
		double x,y;
		fin>>sz;
		RegionCen.resize(sz);//有多少区域
		for(int i=0;i<sz;++i){
			fin>>x>>y;
			RegionCen[i] = Point(x,y);
			int idxSz;
			fin>>idxSz;
			RoadIdxOfRegion[i].resize(idxSz);//每个区域多少条道路
			for(int j=0;j<idxSz;++j){
				fin >> val;
				RoadIdxOfRegion[i][j] = val;//道路的编号
			}
		}
		fin.close();
	}
	catch (exception e){
		return false;
	}
	return true;
}