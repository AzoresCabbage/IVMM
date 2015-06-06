
#pragma once

#include "stdafx.h"

const double eps = 1e-18;//�ж������
//����뾶����λΪm
#define EARTHRADIUS 6371000
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

class Point
{
private:
	//��֪�Ƕȣ����ض�Ӧ�Ļ���
	double getRad(double deg);

	//��֪���Ƚǣ����ض�Ӧ�ĽǶ�
	double getDeg(double rad);
public:
	double x,y,z;//��γ��ת��Ϊ��ά�ռ�����
	int id;
	//Ĭ�Ϲ��캯����do what��
	Point();

	Point(const Point& p):x(p.x),y(p.y),z(p.z),id(p.id){}

	Point(const GeoPoint& p);

	//��ӳ���ڼ���Ƕȵ�ʱ��ȽϷ��㣬����Ҫ����R^2����������ʵ�ʾ����ʱ����Ҫ���Ե���뾶
	Point( double _lon, double _lat);

	Point(double _x, double _y, int _id);

	//�˹��캯�����ڳ�ʼ������(x,y,z)
	Point(const double& _x,const double& _y,const double& _z);
	
	//��õ㵽��һ��p��ŷ����þ��룬��λΪm
	double EucDisTo(const Point& p);

	//��õ㵽��һ��ĵ�����룬��λΪm
	double GeoDisTo(Point& p);

	//������������Ĺ��ɵ�����֮��ļнǣ���λΪ����
	double degBetween(Point& p);

	//���ؼӺţ����ڼ�������
	Point operator + (const Point& p);

	//���ؼ��ţ����ڼ�������
	Point operator - (const Point& p);

	//���س˺ţ����ڼ����������
	double operator * (const Point& p);

	//���س˺ţ����ڼ�����������
	Point operator * (const double& ratio);

	//���س��ţ����ڼ�������
	Point operator / (const double& ratio);

	bool operator == (const Point &p) const {return dcmp(x-p.x) == 0 && dcmp(y-p.y) == 0 && dcmp(z-p.z) == 0;}

	bool operator < (const Point &p) const {
		if (dcmp(y - p.y) == 0) return dcmp(x - p.x) <= 0;
		return dcmp(y - p.y) <= 0;
    }

	//������ģ��
	double length();

	//����p���γ��
	double getLat();

	//����p��ľ���
	double getLon();
};
typedef Point Vector;

//�㵽�߶������
Point pToseg(Point p,Point la,Point lb);

//�㵽�߶ξ���
double dispToseg(Point p,Point a,Point b);
