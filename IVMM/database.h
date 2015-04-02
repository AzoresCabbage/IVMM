/*
PQntuples Returns the number of tuples (instances) in the query result.
PQnfields Returns the number of fields (attributes) in each tuple of the query result.
PQfname Returns the field (attribute) name associated with the given field index. Field indices start at 0.
PQgetvalue Returns a single field (attribute) value of one tuple of a PGresult. Tuple and field indices start at 0.
PQprint Prints out all the tuples and, optionally, the attribute names to the specified output stream.
PQclear Frees the storage associated with the PGresult. Every query result should be freed via PQclear when it is no longer needed.
*/
#pragma once

#include "stdafx.h"
#include "geometry.h"
#include "Graph.h"
using namespace std;

class Database{
private:
	string connInfo;//Á¬½Ó×Ö
	PGconn* conn;
	void closeConn();
	bool connDB();
	vector < pair<double,double> > parseString(string str);
	int Insert(string SQL,int id);
public:
	Database();
	~Database(){closeConn();}
	void preProcData();
	PGresult* execQuery(string SQL);
	bool execUpdate(string SQL);
	void loadInitPoint(const vector <GeoPoint>& P);
	void loadCandiPoint(const vector < vector <Point> >& candiPoint);
	void reOrder();
};