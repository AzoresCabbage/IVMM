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

class Database{
private:
	std::string connInfo;//Á¬½Ó×Ö
	PGconn* conn;
	void closeConn();
	bool connDB();
public:
	Database(std::string dbname,std::string port,std::string dbaddr):conn(NULL){
		char buff[500];
		sprintf_s(buff,"port = '%s' dbname = '%s' hostaddr = '%s' ",port.c_str(),dbname.c_str(),dbaddr.c_str());
		connInfo = buff;
		if(!connDB()){
			exit(0);
		}
	}
	~Database(){closeConn();}
	PGresult* execQuery(std::string SQL);
	bool execUpdate(std::string SQL);
};