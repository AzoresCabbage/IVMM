#include <bits/stdc++.h>
using namespace std;

//处理“飞点”，设定相邻两点之间速度的最大值
//m/s
#define UB 25
//m
#define LB 50

const double eps = 1e-18;//判定的误差
const double EARTH_RADIUS = 6378.137;//地球半径，km
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

vector <NODE> rec,ans;

class Point{
public:
    double x,y;
    Point(double x,double y):x(x),y(y){}
};

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

char str[100];

int main(int argc, char** argv){
    /*
    if(argc != 2) {
        fprintf(stderr,"add file path,like PreprocessData.exe yourfilepath\n");
        return 0;
    }*/
    if(NULL == freopen(argv[1],"r",stdin)){
        fprintf(stderr,"can not open %s\n",argv[1]);
        return 0;
    }
    long long carID;
    int y,mon,d,h,m,s;
    double logi,lati;
    while(gets(str)){
        sscanf(str,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&m,&s,&logi,&lati);
        rec.push_back(NODE(carID,y,mon,d,h,m,s,logi,lati));
    }
    fclose(stdin);
    string filename = string("out_")+argv[1];
    
    freopen(filename.c_str(),"w",stdout);
    if(rec.empty()) {
        fclose(stdout);
        return 0;
    }
    int sz = rec.size();
    ans.push_back(rec[0]);

    for(int i=1;i<sz;i++){
        NODE pre = ans[ans.size()-1];
        NODE cur = rec[i];
        //delete date&time same point
        if(pre == cur) continue;
        //delete error point
        if(getGeoDis(Point(pre.logi,pre.lati),Point(cur.logi,cur.lati))/(cur-pre) > UB)
            continue;
        if(getGeoDis(Point(pre.logi,pre.lati),Point(cur.logi,cur.lati)) < LB)
            continue;
        
        ans.push_back(cur);
    }
    sz = ans.size();
    for(int i=0;i<sz;i++){
        printf("%lld,%d-%d-%d %d:%d:%d,%lf,%lf\n",ans[i].carID,ans[i].y,ans[i].mon,ans[i].d,ans[i].h,ans[i].m,ans[i].s,ans[i].logi,ans[i].lati);
    }

    fclose(stdout);
    return 0;
}
