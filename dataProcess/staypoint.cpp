#include <bits/stdc++.h>
using namespace std;

const double eps = 1e-18;//判定的误差
const double PI = acos(-1.0);//圆周率
const double disThreshold = 10;
const double timeThreshold = 30;

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

vector <NODE> p,ans;
set <int> P;

NODE Mean(set <int> st);
void solve();

int main(int argc, char** argv){
    if(NULL == freopen(argv[1],"r",stdin)){
        fprintf(stderr,"can not open %s\n",argv[1]);
        return 0;
    }
    long long carID;
    int y,mon,d,h,m,s;
    double logi,lati;
    while(gets(str)){
        sscanf(str,"%lld,%d-%d-%d %d:%d:%d,%lf,%lf",&carID,&y,&mon,&d,&h,&m,&s,&logi,&lati);
        p.push_back(NODE(carID,y,mon,d,h,m,s,logi,lati));
    }
    fclose(stdin);
    string filename = string("out_")+argv[1];
    
    freopen(filename.c_str(),"w",stdout);
    if(p.empty()) {
        fclose(stdout);
        return 0;
    }
    solve();
    int sz = ans.size();
    for(int i=0;i<sz;i++){
        printf("%lld,%d-%d-%d %d:%d:%d,%lf,%lf\n",ans[i].carID,ans[i].y,ans[i].mon,ans[i].d,ans[i].h,ans[i].m,ans[i].s,ans[i].logi,ans[i].lati);
    }

    fclose(stdout);
    return 0;
}

void solve(){
    int M = p.size();
    int i=0;
    while(i < M-1){
        int j = i+1;
        bool flag = false;
        while(j < M){
            double dist = getGeoDis(Point(p[i].logi,p[i].lati),Point(p[j].logi,p[j].lati));
            if(dist < disThreshold){
                j = j+1;
                flag = true;
            }
            else break;
        }
        if(p[j-1] - p[i] > timeThreshold && flag)
        {
            for(int k=i;k<j;++k){
                P.insert(k);
            }
            if(i == j-1){
                ans.push_back(Mean(P));
                P.clear();
            }
        }
        if(P.find(i) == P.end())
            ans.push_back(p[i]);
        i = i+1;
    }
}

NODE Mean(set <int> st){
    /*
    double logi = 0, lati = 0;
    int sz = st.size();
    for(auto i=st.begin();i!=st.end();++i){
        logi += p[*i].logi;
        lati += p[*i].lati;
    }
    logi /= sz;
    lati /= sz;
    */
    auto i = st.end();

    return NODE(p[*i].carID,p[*i].y,p[*i].mon,p[*i].d,p[*i].h,p[*i].m,p[*i].s,p[*i].logi,p[*i].lati);
}
