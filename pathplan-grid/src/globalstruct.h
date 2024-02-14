#ifndef STRUCTGLOBAL_H
#define STRUCTGLOBAL_H
#include <iostream>
#include <vector>

struct tagLineInfo
{
	double fStartLon; // degree
	double fStartLat; // degree
	float fStartX;	  // nm
	float fStartY;	  // nm
	float fStartZ;	  // nm
	float fSpd;		  // kn
	float fCos;		  // degree
	double fEndLon;	  // degree
	double fEndLat;	  // degree
	float fEndX;	  // nm
	float fEndY;	  // nm
	float fEndZ;	  // nm
				 // 每条线可设置属于本线的属性
	float fLineObsRadius; // 线上取点时取点间隔 nm
};

struct tagPointInfo
{
	double fLon; // degree
	double fLat; // degree
	float fPosX; // nm
	float fPosY; // nm
	float fPosZ; // nm
};

struct tagChartLineInfo
{
	int nID;		// 编号
	int nType;		// 线条的类型：0-安全等深线 1-航道边界 2-计划航线 3-面状物线条
	float fRangTOL; // 安全距离:nm
	float fGamma;	// positive parameter that is used to adjust the range of influence for the curve obstacle
	std::vector<tagLineInfo> tLineInfo;
};

struct tagChartPointInfo
{
	int nID;		// 编号
	int nType;		// 点类型 0-孤立障碍物，如危险沉船，礁石 1-灯浮等助航标志
	float fRangTOL; // 安全距离:nm
	float fBeta;	// positive scalar
	tagPointInfo tPosInfo;
};

#endif
