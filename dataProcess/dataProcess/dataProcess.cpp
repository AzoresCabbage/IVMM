// dataProcess.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
using namespace std;
const double mxV = 25;//m/s
const double mnDis = 50;//m
const double eps = 1e-18;//判定的误差
const double PI = acos(-1.0);//圆周率


struct NODE{
    long long carID;
    int y,mon,d,h,m,s;
    double lati,logi;
    static const int daySec = 3600*24;
    NODE(long long carID,int y,int mon,int d,int h,int m,int s,double logi,double lati):carID(carID),y(y),mon(mon),d(d),h(h),m(m),s(s),logi(logi),lati(lati){}
    bool operator == (const NODE& x){
        return carID == x.carID 
            && y == x.y 
            && mon == x.mon
            && d == x.d
            && h == x.h
            && m == x.m
            && s == x.s;
    }
    int operator - (const NODE& rhs) const{
		int s1 = h*3600+m*60+s;
		int s2 = rhs.h*3600+rhs.m*60+rhs.s;
		if(y == rhs.y && mon == rhs.mon && d == rhs.d)
			return abs(s1-s2);
		else
			return abs(daySec - abs(s1-s2));
	}
};

class Point{
public:
    double x,y;
    Point(double x,double y):x(x),y(y){}
	Point(const NODE& node){
        x = node.logi;
        y = node.lati;
    }
};

vector <NODE> rec;

//返回角度d的弧度
inline double rad(double d){
	return d * PI / 180.0;
}

double getGeoDis(Point pa,Point pb){
	double lat1 = pa.y;
	double lng1 = pa.x;
	double lat2 = pb.y;
	double lng2 = pb.x;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double dx = lng1 - lng2; // 经度差值
	double dy = lat1 - lat2; // 纬度差值
	double b = (lat1 + lat2) / 2.0; // 平均纬度
	double Lx = rad(dx) * 6367000.0* cos(rad(b)); // 东西距离
	double Ly = 6367000.0 * rad(dy); // 南北距离
	return sqrt(Lx * Lx + Ly * Ly);  // 用平面的矩形对角距离公式计算总距离
}

vector <NODE> speedProc(vector <NODE> &vec){
	vector <NODE> res;
	int sz = vec.size();
	res.push_back(vec[0]);
	for(int i=0;i<sz;++i){
		NODE cur = vec[i];
		NODE pre = res[res.size()-1];
		//delete error point
		if(pre == cur) continue;
        //delete over speed point
        if(getGeoDis(pre,cur)/(cur-pre) > mxV)
            continue;

        res.push_back(cur);
	}
	return res;
}

vector <NODE> distProc(vector <NODE> &vec){
	vector <NODE> res;
	int sz = vec.size();
	res.push_back(vec[0]);
	for(int i=0;i<sz;++i){
		NODE cur = vec[i];
		NODE pre = res[res.size()-1];
		if(getGeoDis(cur,pre) < mnDis)
			continue;
        res.push_back(cur);
	}
	return res;
}

int _tmain(int argc, _TCHAR* argv[])
{
	ifstream fin("input.csv");
	if(!fin.is_open()){
		cout<<"file open error!"<<endl;
		return 0;
	}
	char str[100];
	long long carID;
    int y,mon,d,h,m,s;
    double logi,lati;
	while(fin.getline(str,100)){
        sscanf_s(str,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&m,&s,&logi,&lati);
        rec.push_back(NODE(carID,y,mon,d,h,m,s,logi,lati));
    }
	fin.close();
	vector <NODE> res = speedProc(rec);
	res = distProc(res);
	ofstream fout("output.csv");
	int sz = res.size();
	for(int i=0,sz=res.size();i<sz;i++){
		sprintf_s(str,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",res[i].carID,res[i].y,res[i].mon,res[i].d,res[i].h,res[i].m,res[i].s,res[i].logi,res[i].lati);
        fout<<str<<endl;
    }
	fout.close();
	return 0;
}

