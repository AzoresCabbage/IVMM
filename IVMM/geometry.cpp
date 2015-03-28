#include "stdafx.h"
#include "geometry.h"

//返回角度d的弧度
inline double rad(double d){
	return d * PI / 180.0;
}

//叉积
double xmult(Point p1,Point p2,Point p0){
    return(p1.x-p0.x)*(p2.y-p0.y)-(p2.x-p0.x)*(p1.y-p0.y);
}

//返回欧几里得距离
double getEucDis(Point a,Point b){
	return sqrt(SQR(a.x-b.x)+(a.y-b.y));
}

//返回球面距离，单位为km
double getGeoDis(Point pa,Point pb){
	double lat1 = pa.y;
	double lng1 = pa.x;
	double lat2 = pb.y;
	double lng2 = pb.x;

	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = rad(lng1) - rad(lng2);

	double sin_a2 = SQR(sin(a/2));
	double sin_b2 = SQR(sin(b/2));
	double s = 2 * asin(sqrt(sin_a2 +
		cos(radLat1)*cos(radLat2)*sin_b2));
	s = s * EARTH_RADIUS;
	return s;
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
//	Point tmp = pToseg(p,l);
//	return getGeoDis(p,tmp);
    
	Point t=p;
    t.x+=a.y-b.y,t.y+=b.x-a.x;
    if(xmult(a,t,p)*xmult(b,t,p)>eps){
		double d1 = getGeoDis(p,a);
		double d2 = getGeoDis(p,b);
        return d1<d2?d1:d2;
	}
    return fabs(xmult(p,a,b))/getGeoDis(a,b);
	
}



