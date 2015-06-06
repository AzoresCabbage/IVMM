
#pragma once

#include "stdafx.h"

const double eps = 1e-18;//判定的误差
//地球半径，单位为m
#define EARTHRADIUS 6371000
const double PI = acos(-1.0);//圆周率

//判断浮点数与0的关系
//x<0 --- -1 ; x==0 --- 0 ; x>0 --- 1
inline int dcmp(double x){return (x > eps) - (x < -eps);}

//取平方
template <class T>
T SQR(T x) {return x*x;}

struct Date{
	int year,month,day;
	int hour,minute,second;
	static const int daySec = 3600*24;
	explicit Date(int y,int mon,int d,int h,int min,int sec):year(y),month(mon),day(d),hour(h),minute(min),second(sec){}
	//两个轨迹点记录时间的差值不会跨越一天

	int operator - (const Date& rhs) const{
		int s1 = hour*3600+minute*60+second;
		int s2 = rhs.hour*3600+rhs.minute*60+rhs.second;
		if(year == rhs.year && month == rhs.month && day == rhs.day)
			return abs(s1-s2);
		else
			return abs(daySec - abs(s1-s2));
	}
	bool operator == (const Date& rhs) const{
		return year == rhs.year && month == rhs.month && day == rhs.day && hour == rhs.hour && minute == rhs.minute && second == rhs.second;
	}
};

struct GeoPoint{
	double latitude,longitude;//纬度，经度
	Date date;//日期和时间
	GeoPoint(double lat,double lon,Date dat):latitude(lat),longitude(lon),date(dat){}
};

class Point
{
private:
	//已知角度，返回对应的弧度
	double getRad(double deg);

	//已知弧度角，返回对应的角度
	double getDeg(double rad);
public:
	double x,y,z;//经纬度转化为三维空间坐标
	int id;
	//默认构造函数，do what？
	Point();

	Point(const Point& p):x(p.x),y(p.y),z(p.z),id(p.id){}

	Point(const GeoPoint& p);

	//此映射在计算角度的时候比较方便，不需要除以R^2，但是在求实际距离的时候需要乘以地球半径
	Point( double _lon, double _lat);

	Point(double _x, double _y, int _id);

	//此构造函数用于初始化向量(x,y,z)
	Point(const double& _x,const double& _y,const double& _z);
	
	//求该点到另一点p的欧几里得距离，单位为m
	double EucDisTo(const Point& p);

	//求该点到另一点的地理距离，单位为m
	double GeoDisTo(Point& p);

	//返回两点与地心构成的向量之间的夹角，单位为弧度
	double degBetween(Point& p);

	//重载加号，用于计算向量
	Point operator + (const Point& p);

	//重载减号，用于计算向量
	Point operator - (const Point& p);

	//重载乘号，用于计算向量点乘
	double operator * (const Point& p);

	//重载乘号，用于计算向量数乘
	Point operator * (const double& ratio);

	//重载除号，用于计算数除
	Point operator / (const double& ratio);

	bool operator == (const Point &p) const {return dcmp(x-p.x) == 0 && dcmp(y-p.y) == 0 && dcmp(z-p.z) == 0;}

	bool operator < (const Point &p) const {
		if (dcmp(y - p.y) == 0) return dcmp(x - p.x) <= 0;
		return dcmp(y - p.y) <= 0;
    }

	//求向量模长
	double length();

	//返回p点的纬度
	double getLat();

	//返回p点的经度
	double getLon();
};
typedef Point Vector;

//点到线段最近点
Point pToseg(Point p,Point la,Point lb);

//点到线段距离
double dispToseg(Point p,Point a,Point b);
