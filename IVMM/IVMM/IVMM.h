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
			vector <GeoPoint> P;//�켣��
			vector <vector <Point>> candiPoint;//ÿ���켣��ĺ�ѡ�㼯��
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

		const string dbname;//���ݿ�����
		const string dbport;//���ݿ�˿ں�
		const string dbaddr;//���ݿ��ַ
		const string roadTN;//��·������
		
		static double R;//ѡȡĳ�켣��ĺ�ѡ��ķ�Χ����λΪm,������Ԥ����ʱɾȥ���С��50�ĵ�
		static double Sigma;//��̬�ֲ�����λΪm
		static double miu;
		static int K;//��ѡ����������
		static double beta;//m
		const int threadNum;//����ͶƱ���߳�����
		const int getCandiPointThreadNum;//���ڼ����ѡ����߳�����

		long long carID;

		time_t tm;//��ʱ����
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