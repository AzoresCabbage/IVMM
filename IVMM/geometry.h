
#pragma once

#include "stdafx.h"

const double eps = 1e-18;//判定的误差
const double EARTH_RADIUS = 6378.137;//地球半径，km
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

class Point{
public:
	double x, y;
	
	int id;

	Point() {}
	Point(const GeoPoint& p){x = p.longitude;y = p.latitude;}
	Point(double _x, double _y) : x(_x), y(_y) {}
	Point(double _x, double _y, int _id) : x(_x), y(_y),id(_id) {}
	Point operator + (const Point &p) const {return Point(x + p.x, y + p.y);}
	Point operator - (const Point &p) const {return Point(x - p.x, y - p.y);}
	Point operator * (const double &p) const {return Point(x * p, y * p);}
	Point operator / (const double &p) const {return Point (x / p, y / p);}

	bool operator == (const Point &p) const {return dcmp(x-p.x) == 0 && dcmp(y-p.y) == 0;}

	bool operator < (const Point &p) const {
        if (dcmp(y - p.y) == 0) return dcmp(x - p.x) <= 0;
        return dcmp(y - p.y) <= 0;
    }
};

//返回度数的弧度
inline double rad(double d);

//叉积
double xmult(Point p1,Point p2,Point p0);

//返回欧几里得距离
double getEucDis(Point,Point);

//已知两点经纬度，计算两点距离，单位为km
double getGeoDis(Point,Point);

//计算两直线交点,注意事先判断直线是否平行!
//线段交点请另外判线段相交(同时还是要判断是否平行!)
Point Intersection(Point ua,Point ub,Point va,Point vb);

//点到线段最近点
Point pToseg(Point p,Point la,Point lb);

//点到线段距离
double dispToseg(Point p,Point a,Point b);
