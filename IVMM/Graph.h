#pragma once

#include "stdafx.h"
#include "geometry.h"
#include "database.h"
using namespace std;

struct EDGE{
	int v;//�ڽӱߵĵ�
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

//ɸѡ�켣��ĺ�ѡ������ݽṹ
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

//�ѵ�ͼ��ΪDIVID_NUM*DIVID_NUM������
//ȡֵΪ40ʱ����40��candiPoint��Ҫ2s
//ȡֵΪ10ʱ����40��candiPoint��Ҫ9s
#define DIVID_NUM 40
const bool needDevided = false;
//��ͼ�ľ�γ�ȷ�Χ��ԼΪ146km*244km���ϱ���
const double MinLon = 115.7;
const double MaxLon = 117.4;
const double MinLat = 39.4;
const double MaxLat = 41.6;
class Graph{
private:
	static const int threadNum = 4;
	int n;//ͼ�е�����
	string roadTN;
	vector < EDGE > * edge;//���
	vector <RoadSegment> Road;//·��
	vector <Point> allCandiPoint;//����candidatePoint:PointID->Point ��ӳ��	
	vector <double> MinSpeed;

	vector <Point> RegionCen;
	vector <int> RoadIdxOfRegion[DIVID_NUM*DIVID_NUM];//��¼�����ڵ�·��id

	map <int,Point> idToPoint;//���е�·������ӳ�䣬��id->Point
	map <int,int> pInSeg;//����candiPoint��PointID->roadID��ӳ�䣬��ĳ����ѡ�������ĸ�·��
	
	bool* vis;//��¼�Ƿ���ʹ�
	double* dis;//��src�ľ���
	int* pre;//��¼���··��
	int* fa;//���������ĸ���ͨ��
	double* preV;//��¼���··���ϸ�·�ε��ٶ�
	double R;//��������İ뾶

	void constructGraph();//��ͼ
	void divideRegion();
	void bfs(int u,int sg);
	void constructConn();//������ͨ��
	double judgeV(string s);//���ݵ�·���ͣ����ص�·�ٶȣ���λΪkm/h
	vector <double> getSegSpeed(int des);//�����յ���Ϊdes�����·�ϸ���·�ε��ٶ�
	vector <int> getSegPoint(int des);//�����յ���Ϊdes�����·�ϵĵ�
	int getSegNum(int des);//�����յ���Ϊdes�����·�ж��ٱ�
	double queryShortest(int id1,int id2,Point S,Point T);
	double shortest_path(int S,int T);//���ر��ΪS�����ΪT�����·����
	static DWORD WINAPI calc(LPVOID);
	void writeToFile();
	bool readFile();
public:

	double tTosMin;//��t->sʱ�����·����,m
	int tTosSeg;//��t->sʱ���·��·����
	int totCandiPoint;//candiPoint�����������ڱ��

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