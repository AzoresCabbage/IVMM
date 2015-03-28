
#pragma once

#include "stdafx.h"

const double eps = 1e-18;//�ж������
const double EARTH_RADIUS = 6378.137;//����뾶��km
const double PI = acos(-1.0);//Բ����

//�жϸ�������0�Ĺ�ϵ
//x<0 --- -1 ; x==0 --- 0 ; x>0 --- 1
inline int dcmp(double x){return (x > eps) - (x < -eps);}

//ȡƽ��
template <class T>
T SQR(T x) {return x*x;}

struct Date{
	int year,month,day;
	int hour,minute,second;
	static const int daySec = 3600*24;
	explicit Date(int y,int mon,int d,int h,int min,int sec):year(y),month(mon),day(d),hour(h),minute(min),second(sec){}
	//�����켣���¼ʱ��Ĳ�ֵ�����Խһ��

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
	double latitude,longitude;//γ�ȣ�����
	Date date;//���ں�ʱ��
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

//���ض����Ļ���
inline double rad(double d);

//���
double xmult(Point p1,Point p2,Point p0);

//����ŷ����þ���
double getEucDis(Point,Point);

//��֪���㾭γ�ȣ�����������룬��λΪkm
double getGeoDis(Point,Point);

//������ֱ�߽���,ע�������ж�ֱ���Ƿ�ƽ��!
//�߶ν������������߶��ཻ(ͬʱ����Ҫ�ж��Ƿ�ƽ��!)
Point Intersection(Point ua,Point ub,Point va,Point vb);

//�㵽�߶������
Point pToseg(Point p,Point la,Point lb);

//�㵽�߶ξ���
double dispToseg(Point p,Point a,Point b);
