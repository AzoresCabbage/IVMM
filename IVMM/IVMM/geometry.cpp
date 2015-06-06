#include "stdafx.h"
#include "geometry.h"

///////////////////Point Class Begin//////////////////////

//��֪�Ƕȣ����ض�Ӧ�Ļ���
double Point::getRad(double deg)
{
	return PI/180.0*deg;
}

//��֪���Ƚǣ����ض�Ӧ�ĽǶ�
double Point::getDeg(double rad)
{
	double tmp = 180/PI*rad;
	if(tmp < 0)
		tmp = 180 + tmp;
	return tmp;
}

//Ĭ�Ϲ��캯����do what��
Point::Point():x(0),y(0),z(0){}

Point::Point(const GeoPoint& p)
{
	double lon = getRad(p.longitude);
	double lat = getRad(p.latitude);
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}


Point::Point(double _x, double _y, int _id):id(_id)
{
	double lon = getRad(_x);
	double lat = getRad(_y);
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}

//lonΪ����ֵ��latΪγ��ֵ����λΪ����rad
//��ӳ���ڼ���Ƕȵ�ʱ��ȽϷ��㣬����Ҫ����R^2����������ʵ�ʾ����ʱ����Ҫ���Ե���뾶
Point::Point(double lon,double lat)
{
	lon = getRad(lon);
	lat = getRad(lat);
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}

//�˹��캯�����ڳ�ʼ������(x,y,z)
Point::Point(const double& _x,const double& _y,const double& _z):x(_x),y(_y),z(_z)
{

}

//��õ㵽��һ��p��ŷ����þ��룬��λΪm
double Point::EucDisTo(const Point& p)
{
	return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
}

//��õ㵽��һ��ĵ�����룬��λΪm
double Point::GeoDisTo(Point& p)
{
	//return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
	double cosTheta = degBetween(p);
	return EARTHRADIUS*sqrt(fabs(1.0-SQR(cosTheta)));
}

//������������Ĺ��ɵ�����֮��ļнǣ���λΪ����
double Point::degBetween(Point& p)
{
	return (*this * p) / (length()*p.length());
}

//���ؼӺţ����ڼ�������
Point Point::operator + (const Point& p)
{
	return Point(x+p.x,y+p.y,z+p.z);
}

//���ؼ��ţ����ڼ�������
Point Point::operator - (const Point& p)
{
	return Point(x-p.x,y-p.y,z-p.z);
}

//���س˺ţ����ڼ�����
double Point::operator * (const Point& p)
{
	return x*p.x + y*p.y + z*p.z;
}

//���س˺ţ����ڼ�������
Point Point::operator * (const double& ratio)
{
	return Point(x*ratio,y*ratio,z*ratio);
}

//���س��ţ����ڼ�������
Point Point::operator / (const double& ratio)
{
	if(ratio == 0)
	{
		fprintf(stderr,"vector devided by zero!\n");
		system("pause");
		exit(-1);
	}
	return Point(this->x/ratio,this->y/ratio,this->z/ratio);
}

//������ģ��
double Point::length()
{
	return sqrt(SQR(x)+SQR(y)+SQR(z));
}

//����p���γ��
double Point::getLat()
{
	assert(z >=-1 && z <= 1);
	return getDeg(asin(z));
}

//����p��ľ���
double Point::getLon()
{
	assert(fabs(x) > eps);
	return getDeg(atan(y/x));
}
///////////////////Point Class End//////////////////////



//��p���߶�(begin,end)�����
Point pToseg(Point p,Point begin,Point end){
	Point retVal;

	double dx = begin.x - end.x;  
	double dy = begin.y - end.y;  
	double dz = begin.z - end.z;  
	if(fabs(dx) < eps && fabs(dy) < eps && fabs(dz) < eps )  
	{  
	  retVal = begin;  
	  return retVal;  
	}  

	double v1 = ((p-begin)*(end-begin));
	double v2 = ((p-end)*(begin-end));
	if(  v1*v2  < 0)
	{
		return p.EucDisTo(begin) < p.EucDisTo(end)?begin:end;
	}

	double u = (p.x - begin.x)*(begin.x - end.x)
		+ (p.y - begin.y)*(begin.y - end.y)
		+ (p.z - begin.z)*(begin.z - end.z);  
	u = u/((dx*dx)+(dy*dy)+(dz*dz));  

	retVal.x = begin.x + u*dx;  
	retVal.y = begin.y + u*dy;  
	retVal.z = begin.z + u*dz;  
	return retVal;  
}

//�㵽�߶ξ���
double dispToseg(Point p,Point a,Point b){
	Point tmp = pToseg(p,a,b);
	return p.EucDisTo(tmp);
}