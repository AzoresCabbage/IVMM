#include "stdafx.h"
#include "geometry.h"

///////////////////Point Class Begin//////////////////////

//已知角度，返回对应的弧度
double Point::getRad(double deg)
{
	return PI/180.0*deg;
}

//已知弧度角，返回对应的角度
double Point::getDeg(double rad)
{
	double tmp = 180/PI*rad;
	if(tmp < 0)
		tmp = 180 + tmp;
	return tmp;
}

//默认构造函数，do what？
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

//lon为经度值，lat为纬度值，单位为弧度rad
//此映射在计算角度的时候比较方便，不需要除以R^2，但是在求实际距离的时候需要乘以地球半径
Point::Point(double lon,double lat)
{
	lon = getRad(lon);
	lat = getRad(lat);
	x = cos(lat)*cos(lon);
	y = cos(lat)*sin(lon);
	z = sin(lat);
}

//此构造函数用于初始化向量(x,y,z)
Point::Point(const double& _x,const double& _y,const double& _z):x(_x),y(_y),z(_z)
{

}

//求该点到另一点p的欧几里得距离，单位为m
double Point::EucDisTo(const Point& p)
{
	return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
}

//求该点到另一点的地理距离，单位为m
double Point::GeoDisTo(Point& p)
{
	//return EARTHRADIUS*sqrt(SQR(x-p.x)+SQR(y-p.y)+SQR(z-p.z));
	double cosTheta = degBetween(p);
	return EARTHRADIUS*sqrt(fabs(1.0-SQR(cosTheta)));
}

//返回两点与地心构成的向量之间的夹角，单位为弧度
double Point::degBetween(Point& p)
{
	return (*this * p) / (length()*p.length());
}

//重载加号，用于计算向量
Point Point::operator + (const Point& p)
{
	return Point(x+p.x,y+p.y,z+p.z);
}

//重载减号，用于计算向量
Point Point::operator - (const Point& p)
{
	return Point(x-p.x,y-p.y,z-p.z);
}

//重载乘号，用于计算点乘
double Point::operator * (const Point& p)
{
	return x*p.x + y*p.y + z*p.z;
}

//重载乘号，用于计算数乘
Point Point::operator * (const double& ratio)
{
	return Point(x*ratio,y*ratio,z*ratio);
}

//重载除号，用于计算数除
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

//求向量模长
double Point::length()
{
	return sqrt(SQR(x)+SQR(y)+SQR(z));
}

//返回p点的纬度
double Point::getLat()
{
	assert(z >=-1 && z <= 1);
	return getDeg(asin(z));
}

//返回p点的经度
double Point::getLon()
{
	assert(fabs(x) > eps);
	return getDeg(atan(y/x));
}
///////////////////Point Class End//////////////////////



//点p到线段(begin,end)最近点
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

//点到线段距离
double dispToseg(Point p,Point a,Point b){
	Point tmp = pToseg(p,a,b);
	return p.EucDisTo(tmp);
}