#pragma once

#include "stdafx.h"
#include "geometry.h"
#include "database.h"
using namespace std;

struct EDGE{
	int v;//邻接边的点
	double cost,speed;
	Point pts;
	EDGE(int _v,double _cost,double _speed,Point _pts):v(_v),cost(_cost),speed(_speed),pts(_pts){}
};
struct RoadSegment{
	int stID,edID;
	int id;
	double v;
	bool oneway;
	Point st,ed;
	RoadSegment(){}
	RoadSegment(Point _st,Point _ed,int _stID,int _edID,int _id,double _v,bool _oneway):st(_st),ed(_ed),stID(_stID),edID(_edID),id(_id),v(_v),oneway(_oneway){}
};
struct NODE{
	double first;
	int second;
	NODE(double _f,int _s):first(_f),second(_s){};
	bool operator < (const NODE& x) const{
		return first > x.first;
	}
};

//筛选轨迹点的候选点的数据结构
struct TMP{
	double dis;
	Point pts;
	int rid;
	TMP(double _dis,Point _pts,int _rid):dis(_dis),pts(_pts),rid(_rid){}
	bool operator < (const TMP& t)const{
		if(dis == t.dis){
			if(pts == t.pts) return rid < t.rid;
			else return pts < t.pts;
		}
		else
			return dis < t.dis;
	}
};

//把地图分为DIVID_NUM*DIVID_NUM个区域
//取值为40时，求40个candiPoint需要2s
//取值为10时，求40个candiPoint需要9s
#define DIVID_NUM 40
const bool needDevided = false;
//地图的经纬度范围，约为146km*244km，南北长
const double MinLon = 115.7;
const double MaxLon = 117.4;
const double MinLat = 39.4;
const double MaxLat = 41.6;
class Graph{
private:
	static const int threadNum = 4;
	int n;//图中点总数
	string roadTN;
	vector < EDGE > * edge;//存边
	vector <RoadSegment> Road;//路段
	vector <Point> allCandiPoint;//对于candidatePoint:PointID->Point 的映射	
	vector <double> MinSpeed;

	vector <Point> RegionCen;
	vector <int> RoadIdxOfRegion[DIVID_NUM*DIVID_NUM];//记录区域内道路的id

	map <int,Point> idToPoint;//所有道路网络点的映射，点id->Point
	map <int,int> pInSeg;//对于candiPoint，PointID->roadID的映射，即某个候选点属于哪个路段
	
	bool* vis;//记录是否访问过
	double* dis;//到src的距离
	int* pre;//记录最短路路径
	int* fa;//标明属于哪个连通块
	double* preV;//记录最短路路径上各路段的速度
	double R;//划分区域的半径

	void constructGraph();//建图
	void divideRegion();
	void bfs(int u,int sg);
	void constructConn();//构建连通块
	double judgeV(string s);//根据道路类型，返回道路速度，单位为km/h
	vector <double> getSegSpeed(int des);//返回终点编号为des的最短路上各个路段的速度
	vector <int> getSegPoint(int des);//返回终点编号为des的最短路上的点
	int getSegNum(int des);//返回终点编号为des的最短路有多少边
	double queryShortest(int id1,int id2,Point S,Point T);
	double shortest_path(int S,int T);//返回编号为S到编号为T的最短路长度
	static DWORD WINAPI calc(LPVOID);
	void writeToFile();
	bool readFile();
public:

	double tTosMin;//求t->s时的最短路长度,m
	int tTosSeg;//求t->s时最短路的路段数
	int totCandiPoint;//candiPoint的总数，用于编号

	Graph(string roadTN);
	~Graph();
	void reset();
	vector < Point > Graph::getCandidate(Point p,double R,int K);
	vector <int> getPath(Point t,Point s);
	vector <double> getSpeed();//m/s
	Point getCandiPointById(int id);
	Point getPointById(int id);
	double getCandiShortest(Point t,Point s);
	bool isInSameSeg(int id1,int id2);
};