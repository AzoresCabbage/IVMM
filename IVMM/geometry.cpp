#include "stdafx.h"
#include "geometry.h"

//返回角度d的弧度
inline double rad(double d){
	return d * PI / 180.0;
}

inline double degree(double rad){
	return 180 * rad / PI;
}

//叉积
double xmult(Point p1,Point p2,Point p0){
    return(p1.x-p0.x)*(p2.y-p0.y)-(p2.x-p0.x)*(p1.y-p0.y);
}

//返回欧几里得距离
double getEucDis(Point a,Point b){
	return sqrt(SQR(a.x-b.x)+(a.y-b.y));
}

//返回球面距离，单位为m
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

	
	/*double a = radLat1 - radLat2;
	double b = rad(lng1) - rad(lng2);

	double sin_a2 = SQR(sin(a/2));
	double sin_b2 = SQR(sin(b/2));
	double s = 2 * asin(sqrt(sin_a2 +
		cos(radLat1)*cos(radLat2)*sin_b2));
	s = s * EARTH_RADIUS;
	return s;*/
}


//计算两直线交点,注意事先判断直线是否平行!
//线段交点请另外判线段相交(同时还是要判断是否平行!)
Point Intersection(Point ua,Point ub,Point va,Point vb){
    Point ret=ua;
    double t=((ua.x-va.x)*(va.y-vb.y)-(ua.y-va.y)*(va.x-vb.x))/((ua.x-ub.x)*(va.y-vb.y)-(ua.y-ub.y)*(va.x-vb.x));
    ret.x+=(ub.x-ua.x)*t;
    ret.y+=(ub.y-ua.y)*t;
    return ret;
}

//点到线段最近点
Point pToseg(Point p,Point la,Point lb){
    Point t=p;
    t.x+=la.y-lb.y,t.y+=lb.x-la.x;
    if(xmult(la,t,p)*xmult(lb,t,p)>eps){
		//double d1 = getEucDis(p,l.a);
		double d11 = getGeoDis(p,la);
		//double d2 = getEucDis(p,l.b);
		double d22 = getGeoDis(p,lb);
		return d11 < d22 ? la:lb;
        //return getEucDis(p,l.a)<getEucDis(p,l.b)?l.a:l.b;
	}
    return Intersection(p,t,la,lb);
}

//点到线段距离
double dispToseg(Point p,Point a,Point b){
	//double pa = getGeoDis(p,a);
	//double pb = getGeoDis(p,b);
	//double ab = getGeoDis(a,b);
	//double sum = (pa+pb+ab)/2;
	//double s = sqrt(sum*(sum-pa)*(sum-pb)*(sum-ab));
	//return 2*s/ab;

	Point tmp = pToseg(p,a,b);
	return getGeoDis(p,tmp);
	//double lat3 = p.y;//3.227511;
	//double lon3 = p.x;//101.724318;
	//double lat2 = a.y;//3.222895;
	//double lon2 = a.x;//101.719751;
	//double lat1 = b.y;//3.224972;
	//double lon1 = b.x;//101.722932;

	//double y = sin(lon3 - lon1) * cos(lat3);
	//double x = cos(lat1) * sin(lat3) - sin(lat1) * cos(lat3) * cos(lat3 - lat1);
	//double bearing1 = degree(atan2(y, x));
	//bearing1 = 360 - (bearing1 + 360 % 360);

	//double y2 = sin(lon2 - lon1) * cos(lat2);
	//double x2 = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lat2 - lat1);
	//double bearing2 = degree(atan2(y2, x2));
	//bearing2 = 360 - (bearing2 + 360 % 360);

	//double lat1Rads = rad(lat1);
	//double lat3Rads = rad(lat3);
	//double dLon = rad(lon3 - lon1);

	//double distanceAC = acos(sin(lat1Rads) * sin(lat3Rads)+cos(lat1Rads)*cos(lat3Rads)*cos(dLon)) * 6371000;  
	//double min_distance = fabs(asin(sin(distanceAC/6371)*sin(rad(bearing1)-rad(bearing2))) * 6371000);
	//return min_distance;
	/*
	Point t=p;
    t.x+=a.y-b.y,t.y+=b.x-a.x;
    if(xmult(a,t,p)*xmult(b,t,p)>eps){
		double d1 = getGeoDis(p,a);
		double d2 = getGeoDis(p,b);
        return d1<d2?d1:d2;
	}
    return fabs(xmult(p,a,b))/getGeoDis(a,b);
	*/
}



