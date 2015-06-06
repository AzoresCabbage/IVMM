#include "stdafx.h"
#include "database.h"
#include "Graph.h"
using namespace std;

Database::Database(/*string dbname,string port,string dbaddr*/):conn(NULL){
	std::string dbname = "osm";//数据库名称
	std::string port = "5432";//数据库端口号
	std::string dbaddr = "127.0.0.1";//数据库地址
	char buff[500];
	sprintf_s(buff,"port = '%s' dbname = '%s' hostaddr = '%s' ",port.c_str(),dbname.c_str(),dbaddr.c_str());
	connInfo = buff;
	if(!connDB()){
		exit(0);
	}
}

bool Database::connDB(){
	std::string tmp,username,password;
	
	username = "postgres";
	password = "wyjcool";
	/*std::cout<<"Database user name:";
	std::cin>>username;
	std::cout<<"Database user password:";
	std::cin>>password;*/
	
	char buff[500];
	sprintf_s(buff,"user = '%s' password = '%s' ",username.c_str(),password.c_str());
	connInfo = connInfo + buff;
	
	conn = PQconnectdb(connInfo.c_str());

	if(PQstatus(conn) == CONNECTION_BAD){
		std::cerr<<"Database connection failed!"<<std::endl;
		return false;
	}
	else{
		std::cerr<<"Database connection success!"<<std::endl;
		return true;
	}
}

void Database::closeConn(){
	PQfinish(conn);
}

PGresult* Database::execQuery(std::string SQL){
	PGresult* res = PQexec(conn,SQL.c_str());
	if(PQresultStatus(res) != PGRES_TUPLES_OK){
		std::cerr<<"SQL query error! statement:"<<SQL<<std::endl;
		PQclear(res);
		//closeConn(conn);
		return NULL;
	}
	return res;
}

bool Database::execUpdate(std::string SQL){
	PGresult* res = PQexec(conn,SQL.c_str());
	if(PQresultStatus(res) != PGRES_COMMAND_OK){
		//std::ofstream fout("output.txt");
		std::cerr<<"SQL update error! statement:"<<SQL<<std::endl;
		//fout<<SQL<<std::endl;
		//fout.close();
		PQclear(res);
		//closeConn(conn);
		return false;
	}
	return true;
}

//把轨迹点存入数据库
void Database::loadInitPoint(const vector <GeoPoint>& P){
	string SQL = "select * from pg_class where relname = 'init_point'";
	PGresult* res = execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from init_point";
		execUpdate(SQL);
	}
	else{
		SQL = "create table init_point (id integer primary key,year integer,month integer,day integer,hour integer,minute integer,second integer,way geometry(Point,4326))";
		execUpdate(SQL);
	}
	int sz = (int)P.size();
	char buffer[500];

	for(int i=0;i<sz;++i){
		sprintf_s(buffer,"insert into init_point values(%d,%d,%d,%d,%d,%d,%d,ST_GeomFromText('Point(%lf %lf)',4326))",
			i+1,
			P[i].date.year,
			P[i].date.month,
			P[i].date.day,
			P[i].date.hour,
			P[i].date.minute,
			P[i].date.second,
			P[i].longitude,
			P[i].latitude);
		SQL = buffer;
		execUpdate(SQL);
	}
}

//把候选点存入数据库
void Database::loadCandiPoint( vector < vector <Point> >& candiPoint){
	string SQL = "select * from pg_class where relname = 'candi_point'";
	PGresult* res = execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from candi_point";
		execUpdate(SQL);
	}
	else{
		SQL = "create table candi_point (id integer primary key,belong integer,way geometry(Point,4326))";
		execUpdate(SQL);
	}
	int sz = (int)candiPoint.size();
	char buffer[500];

	for(int i=0;i<sz;i++){
		int candisz = (int)candiPoint[i].size();
		for(int j=0;j<candisz;++j){
			sprintf_s(buffer,"insert into candi_point values(%d,%d,ST_GeomFromText('Point(%lf %lf)',4326))",
				candiPoint[i][j].id,
				i+1,
				candiPoint[i][j].getLon(),
				candiPoint[i][j].getLat());
			SQL = buffer;
			execUpdate(SQL);
		}
	}
}






//把数据库中读出的LINESTRING字符串转化为点坐标
//返回点的vector
vector < pair<double,double> > Database::parseString(string str){
	vector < pair<double,double> > res;
	int len = (int)str.size();
	int pre = 11;
	double x,y;
	for(int i=0;i<len;++i){
		if(str[i] == ',' || str[i] == ')'){
			string tmp = str.substr(pre,i-pre);
			//cerr<<tmp<<endl;
			sscanf_s(tmp.c_str(),"%lf %lf",&x,&y);
			res.push_back(make_pair(x,y));
			pre = i+1;
		}
	}
	return res;
}

//处理原始数据，把LINESTRING中含有多于两个点的部分拆分后存入数据库
//只需要运行一次
int Database::Insert(string SQL,int id){
	PGresult* res = execQuery(SQL);

	int tupleNum = PQntuples(res);
	int fieldNum = PQnfields(res);

	for(int i=0;i<tupleNum;i++){
		string content = PQgetvalue(res,i,fieldNum-1);
		vector < pair<double,double> > pts = parseString(content);
		int sz = (int)pts.size();
		for(int j=1;j<sz;j++){

			SQL = "insert into network values(";
			SQL += std::to_string(id++)+",0,0,";//id,source,target
			SQL += PQgetvalue(res,i,0);//osm_id
			SQL += ",";
			for(int k=1;k<fieldNum;k++){

				char * ss = PQgetvalue(res,i,k);
				if(strlen(ss) == 0) 
					SQL+="NULL";
				else{
					char * name = PQfname(res,k);
					if(strcmp(name,"name") == 0 || strcmp(name,"tags") == 0){
						SQL += "$$";
						SQL += ss;
						SQL += "$$";
					}
					else if(strcmp(name,"way") == 0){
						SQL += "ST_geomFromText('LINESTRING(";
						SQL += std::to_string(pts[j-1].first);
						SQL += " ";
						SQL += std::to_string(pts[j-1].second);
						SQL += ",";
						SQL += std::to_string(pts[j].first);
						SQL += " ";
						SQL += std::to_string(pts[j].second);
						SQL += ")',900913)";
					}
					else{
						SQL += "'";
						SQL += ss;
						SQL += "'";
					}
				}
				if(k+1 == fieldNum) break;
				SQL += ",";
			}
			SQL += ")";
			//cerr<<SQL<<endl;
			execUpdate(SQL);
		}
	}

	PQclear(res);
	return id;
}

void Database::preProcData(){

	string SQL = "select * from pg_class where relname = 'network'";
	PGresult* res = execQuery(SQL);
	int num = PQntuples(res);
	PQclear(res);

	if(num != 0){
		SQL = "Delete from network";
		execUpdate(SQL);
	}
	else{
		SQL = "";
		ifstream fin;
		fin.open("createNetwork.txt");
		string tmp;
		while(fin>>tmp) SQL += tmp;
		fin.close();
		execUpdate(SQL);
	}

	string Field = "";
	SQL = "select * from allroads";
	res = execQuery(SQL);

	int tupleNum = PQntuples(res);
	int fieldNum = PQnfields(res);

	for(int i=0;i<fieldNum-1;i++){
		Field += "allroads.";
		char * name = PQfname(res,i);
		if(strstr(name,":") == NULL) 
			Field += name;
		else{
			Field += "\"";
			Field += name;
			Field += "\"";
		}
		Field += ",";
	}
	Field += "ST_AsText(way) as way";
	PQclear(res);
	int id = 1;
	id = Insert("select "+ Field + " from allroads",id);
}

void Database::reOrder(){
	string SQL = "select gid from network order by gid";
	PGresult *res = execQuery(SQL);
	int num = PQntuples(res);
	for(int i=0;i<num;++i){
		string id = PQgetvalue(res,i,0);
		if(id == std::to_string(i+1)) continue;
		SQL = "update network set gid = "+std::to_string(i+1) + " where gid = " + id;
		execUpdate(SQL);
	}
	PQclear(res);
}