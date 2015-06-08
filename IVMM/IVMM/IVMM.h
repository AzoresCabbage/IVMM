#pragma once
#include "stdafx.h"
#include "geometry.h"
#include "database.h"
#include "Eigen\Core"

using namespace Eigen;
namespace std
{
	class IVMM
	{
	private:
		class DATA
		{
		public:
			DATA(){network = new Graph("network");}
			DATA(string roadTN){network = new Graph(roadTN);}
			~DATA(){delete network;}
			vector <GeoPoint> P;//轨迹点
			vector <vector <Point>> candiPoint;//每个轨迹点的候选点集合
			vector <MatrixXd> M;
			vector <int> vote;
			vector <double> fvalue;
			Graph* network;
			void reset()
			{
				P.clear();
				candiPoint.clear();
				M.clear();
				vote.clear();
				fvalue.clear();
				network->reset();
			}
		}data;

		static mutex lock_it;
		static mutex lockVote;
		static int it;

		const string dbname;//数据库名称
		const string dbport;//数据库端口号
		const string dbaddr;//数据库地址
		const string roadTN;//道路表名称
		
		static double R;//选取某轨迹点的候选点的范围，单位为m,对数据预处理时删去间距小于50的点
		static double Sigma;//正态分布，单位为m
		static double miu;
		static int K;//候选点最多的数量
		static double beta;//m
		const int threadNum;//用于投票的线程数量
		const int getCandiPointThreadNum;//用于计算候选点的线程数量

		long long carID;

		time_t tm;//计时变量
		static double Coef;//norm distribution coef
		Database* DB;

		static double f(double x);
		static double N(int i,int t,DATA&);
		double V(double d,Point& t,Point& s);

		void init();
		bool getTaxiTrajectory(string filePath);
		void writeToDB(vector <Point>& Traj,string tbname);
		void writeToFile(vector <Point>& Traj,string path);
		static void calc_candiPoint(DATA &data);
		static void FindMatchedSequence(vector <Point>&,int i,int k,vector <MatrixXd>& fi_i,vector <double>& Wi,DATA&);
		static void interactiveVoting(DATA& );
		void solve(vector <Point>&);
		void dealFlyPoint(vector <Point>& Ori,vector <Point>& res);
		
	public:
		IVMM(string _dbname = "osm",string _dbport = "5432",string _dbaddr = "127.0.0.1",string _roadTN = "network",
		   double _R = 50,double _sigma = 10,double _miu = 5,int _K = 5,double _beta = 7000,
		   int _threadNum = 3,int _candiThreadNum = 3);
		~IVMM();
		void batchMatching(string filepath);
	};
}