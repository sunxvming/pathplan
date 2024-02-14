#ifndef STRUCTGLOBAL_H
#define STRUCTGLOBAL_H
#include <iostream>
#include <vector>

// 路权定义
enum OSWAYRIGHT
{
	OSWayNULL = 0,
	OSGiveWay,
	OSRightWay
};

// 他船相对于我船的属性
enum TARGETPROPERTY
{
	PROPERTYNULL = 0,
	BEOVERTAKEN /*被追越*/,
	STARBOARD_CLASS_A /*右舷A类船*/,
	STARBOARD_CLASS_B /*右舷B类船*/,
	OVERTAKING /*正追越他船*/,
	CLASS_D /*左舷来船-D类船*/,
	HEADON /*对遇*/
};

// 静态信息
struct tagShipStaticInfo
{
	std::string strName;	  // 船名
	int nCode;				  // Code号
	float fLength;			  // 船长m
	float fBreadth;			  // 船宽m
	float fDraft;			  // 吃水m
	float fWeight;			  // 船舶重量，单位t
	float fTn_S;			  // 本船全速满舵90度所需的时间Tn,单位:秒
	float fTn_P;			  // 本船全速满舵90度所需的时间Tn,单位:秒
	float fTCPATOL;			  // TCPA阈值 min//协商避碰实施的阈值
	float fDCPATOL;			  // DCPA阈值 nm//协商避碰实施的阈值
	float fEmgTCPATOL;		  // TCPA阈值 min//应急避碰实施的阈值
	float fEmgDCPATOL;		  // DCPA阈值 nm//应急避碰实施的阈值
	float fTurnAngTOL;		  // 最大转向角度限制，每15秒内转向角度一般小于5.0度//degree
	float fAdvisedTurnAngTOL; // 推荐的转向角度限制
	float fRadius;			  // 障碍物半径，除本船外，所有目标（障碍物均膨化为圆）//nm

	tagShipStaticInfo()
	{
		strName = "";
		nCode = 0;
		fLength = 222.2222;
		fBreadth = 22.22;
		fDraft = 7.7;
		fWeight = 22222.22222;
		fTn_S = 120.0;
		fTn_P = 120.0;
		fTCPATOL = 11.11;
		fDCPATOL = 2.2;
		fEmgTCPATOL = 6.6;
		fEmgDCPATOL = 1.0;
		fTurnAngTOL = 10.0;
		fAdvisedTurnAngTOL = 10.0;
		fRadius = 1.0;
	}
};
// 动态信息
struct tagShipDynamicInfo
{
	// 本船当前位置与姿态
	double fLon; // 本船经度，单位degree
	double fLat; // 本船纬度，单位degree
	float fPosX; // 本船位置，nm
	float fPosY; // 本船位置，nm
	float fPosZ; // 本船位置，nm
	float fHdg;	 // 本船船艏向，rad
	float fCos;	 // 本船航向，rad
	float fSpd;	 // 本船速度kn
	// add by zzx 2020.04.27
	float fSpdU;		// 速度m/s
	float fSpdV;		// 速度m/s
	float fSpdYaw;		// 转艏速率 rad/s
	float fWindSpd;		// 风速m/s
	float fWindDir;		// 风向 rad/s
	float fCurSpd;		// 流速m/s
	float fCurDir;		// 流向rad/s
	float fHsWave;		// 波高
	float fWaveDir;		// 浪向
	float fToWave;		// 周期
	float fRealRudder;	// 实际舵角
	float fOrderRudder; // 命令舵角
	float fRealRPM;		// 实际转速 转/s
	float fOrderRPM;	// 命令转速 转/s
	// end by zzx 2020.04.27
	//
	float fTCPA;		// 当前TCPA：目标船对本船min
	float fDCPA;		// 当前DCPA：目标船对本船nm
	float fRang;		// 当前距离：目标船相对于本船 nm
	float fTrueBearing; // 当前方位：目标船相对于本船degree
	float fRelBearing;	// 相对方位：目标船相对于本船degree
	float fRelSpd;		// 当前相对速度：目标船相对于本船m/s
	float fRelCos;		// 当前相对航向：目标船相对于本船radius

	OSWAYRIGHT OSWayRight;	   // 本船路权：直航与让路
	TARGETPROPERTY TSProperty; // 交会时目标船属性
	int nInitState;			   // 当前状态是紧急避碰还是协商避碰，0 = 默认， 1=协商， 2= 紧急。
	tagShipDynamicInfo()
	{
		// 本船当前位置与姿态
		fLon = 0.0;	 // 本船经度，单位degree
		fLat = 0.0;	 // 本船纬度，单位degree
		fPosX = 0.0; // 本船位置，nm
		fPosY = 0.0; // 本船位置，nm
		fPosZ = 0.0; // 本船位置，nm
		fHdg = 0.0;	 // 本船船艏向，rad
		fCos = 0.0;	 // 本船航向，rad
		fSpd = 0.0;	 // 本船速度kn
					// add by zzx 2020.04.27
		fSpdU = 0.0;		// 速度m/s
		fSpdV = 0.0;		// 速度m/s
		fSpdYaw = 0.0;		// 转艏速率 rad/s
		fWindSpd = 0.0;		// 风速m/s
		fWindDir = 0.0;		// 风向 rad/s
		fCurSpd = 0.0;		// 流速m/s
		fCurDir = 0.0;		// 流向rad/s
		fHsWave = 0.0;		// 波高
		fWaveDir = 0.0;		// 浪向
		fRealRudder = 0.0;	// 实际舵角
		fOrderRudder = 0.0; // 命令舵角
		fRealRPM = 0.0;		// 实际转速 转/s
		fOrderRPM = 0.0;	// 命令转速 转/s
						 // end by zzx 2020.04.27
						 //
		fTCPA = 0.0;		// 当前TCPA：目标船对本船min
		fDCPA = 0.0;		// 当前DCPA：目标船对本船nm
		fRang = 0.0;		// 当前距离：目标船相对于本船 nm
		fTrueBearing = 0.0; // 当前方位：目标船相对于本船degree
		fRelBearing = 0.0;	// 相对方位：目标船相对于本船degree
		fRelSpd = 0.0;		// 当前相对速度：目标船相对于本船m/s
		fRelCos = 0.0;		// 当前相对航向：目标船相对于本船radius

		OSWayRight = OSGiveWay;	   // 本船路权：直航与让路
		TSProperty = PROPERTYNULL; // 交会时目标船属性
		nInitState = 0;
	}
};
// 目标信息
struct tagShipGoalInfo
{
	// 目标位置：也可能是移动目标或者追踪的其他船
	double fGoalLon;	 // 目标点经度：degree
	double fGoalLat;	 // 目标点纬度：degree
	float fGoalPosX;	 // 目标点位置：nm
	float fGoalPosY;	 // 目标点位置：nm
	float fGoalPosZ;	 // 目标点位置：nm
	float fGoalSpd;		 // 目标点速度：kn
	float fGoalCos;		 // 目标点航向：rad
	bool bLastGoal;		 // 对应航线中最后一个目标点：对于非计划航线，此点即为最后一个点，为true
	float fDis2Goal;	 // 当前位置距离目标点的距离//nm
	float fRealDis2Goal; // 外部显示使用：//当前位置距离目标点的距离//nm
	float fAng2Goal;	 // 目标点的角度//rad
	// 目标点对本船的引力：可以是固定目标，也可以移动目标
	float fAttAng;	 // 引力角度
	double fAttSize; // 引力大小
	double fAttX;	 // 引力大小
	double fAttY;	 // 引力大小
	// 2020.06.01:补充第二个目标点
	double fNextGoalLon; // 目标点经度：degree
	double fNextGoalLat; // 目标点纬度：degree
	float fNextGoalPosX; // 目标点位置：nm
	float fNextGoalPosY; // 目标点位置：nm
	float fNextGoalPosZ; // 目标点位置：nm
	float fNextGoalSpd;	 // 目标点速度：kn
	float fNextGoalCos;	 // 目标点航向：rad
	// 2022.07.07新增加变量用于恢复到原航线，引入第二个引力，进行双引力控制
	float fDis2Route;	// 偏离计划航线的距离:nm
	double fPreGoalLon; // 目标点经度：degree
	double fPreGoalLat; // 目标点纬度：degree
	float fPreGoalPosX; // 目标点位置：nm
	float fPreGoalPosY; // 目标点位置：nm
	float fPreGoalPosZ; // 目标点位置：nm
	float fPreGoalSpd;	// 目标点速度：kn
	float fPreGoalCos;	// 目标点航向：rad

	// 第二个引力：恢复到原航线
	float fGoalPosX2;	  // 目标点位置：nm
	float fGoalPosY2;	  // 目标点位置：nm
	float fGoalPosZ2;	  // 目标点位置：nm
	float fDis2Goal2;	  // 当前位置距离目标点的距离//nm
	float fRealDis2Goal2; // 外部显示使用：//当前位置距离目标点的距离//nm
	float fAng2Goal2;	  // 目标点的角度//rad
	float fAttAng2;		  // 引力角度
	double fAttSize2;	  // 引力大小
	double fAttX2;		  // 引力大小
	double fAttY2;		  // 引力大小
	tagShipGoalInfo()
	{
		// 目标位置：也可能是移动目标或者追踪的其他船
		fGoalLon = 0.0;		 // 目标点经度：degree
		fGoalLat = 0.0;		 // 目标点纬度：degree
		fGoalPosX = 0.0;	 // 目标点位置：nm
		fGoalPosY = 0.0;	 // 目标点位置：nm
		fGoalPosZ = 0.0;	 // 目标点位置：nm
		fGoalSpd = 0.0;		 // 目标点速度：kn
		fGoalCos = 0.0;		 // 目标点航向：rad
		bLastGoal = 0.0;	 // 对应航线中最后一个目标点：对于非计划航线，此点即为最后一个点，为true
		fDis2Goal = 0.0;	 // 当前位置距离目标点的距离//nm
		fRealDis2Goal = 0.0; // 外部显示使用：//当前位置距离目标点的距离//nm
		fAng2Goal = 0.0;	 // 目标点的角度//rad
		// 目标点对本船的引力：可以是固定目标，也可以移动目标
		fAttAng = 0.0;	// 引力角度
		fAttSize = 0.0; // 引力大小
		fAttX = 0.0;	// 引力大小
		fAttY = 0.0;	// 引力大小
		// 2020.06.01:补充第二个目标点
		fNextGoalLon = 0.0;	 // 目标点经度：degree
		fNextGoalLat = 0.0;	 // 目标点纬度：degree
		fNextGoalPosX = 0.0; // 目标点位置：nm
		fNextGoalPosY = 0.0; // 目标点位置：nm
		fNextGoalPosZ = 0.0; // 目标点位置：nm
		fNextGoalSpd = 0.0;	 // 目标点速度：kn
		fNextGoalCos = 0.0;	 // 目标点航向：rad

		fDis2Route = 0.0; // 偏离计划航线的距离:nm
		fPreGoalLon = 0.0;
		fPreGoalLat = 0.0;
		fPreGoalPosX = 0.0;
		fPreGoalPosY = 0.0;
		fPreGoalPosZ = 0.0;
		fPreGoalSpd = 0.0;
		fPreGoalCos = 0.0;

		fGoalPosX2 = 0.0;	  // 目标点位置：nm
		fGoalPosY2 = 0.0;	  // 目标点位置：nm
		fGoalPosZ2 = 0.0;	  // 目标点位置：nm
		fDis2Goal2 = 0.0;	  // 当前位置距离目标点的距离//nm
		fRealDis2Goal2 = 0.0; // 外部显示使用：//当前位置距离目标点的距离//nm
		fAng2Goal2 = 0.0;	  // 目标点的角度//rad
		fAttAng2 = 0.0;		  // 引力角度
		fAttSize2 = 0.0;	  // 引力大小
		fAttX2 = 0.0;		  // 引力大小
		fAttY2 = 0.0;		  // 引力大小
	}
};
// 操纵性信息
struct tagShipManeuverInfo
{
};
// 避让信息
struct tagColliAvoidInfo
{
	// 斥力和引力角度:弧度rad
	float fAttRepAng;
	// 引力
	double fAttSize2; // 引力大小
	double fAttX2;
	double fAttY2;
	// 斥力
	double fRepSize; // 斥力大小
	double fRepX;
	double fRepY;
	double fRepSize2; // 斥力大小
	double fRepX2;
	double fRepY2;
	tagColliAvoidInfo()
	{
		// 斥力和引力角度:弧度rad
		fAttRepAng = 0.0;

		fAttX2 = 0.0;
		fAttY2 = 0.0;
		// 斥力
		fRepSize = 0.0; // 斥力大小
		fRepX = 0.0;
		fRepY = 0.0;
		fRepSize2 = 0.0; // 斥力大小
		fRepX2 = 0.0;
		fRepY2 = 0.0;
	}
};

// 本船信息
struct tagOSInfo
{
	int nOSID;					// ID号
	tagShipStaticInfo StaInfo;	// 静态信息
	tagShipDynamicInfo DynInfo; // 动态信息
	tagShipGoalInfo GoalInfo;	// 目标信息
							  // 总力
	double fSumForceX; // 总合力分量
	double fSumForceY; // 总合力分量
};

// 障碍物信息:动态障碍物和静态障碍物
struct tagObsInfo
{
	int nID;					// ID号
	tagShipStaticInfo StaInfo;	// 静态信息
	tagShipDynamicInfo DynInfo; // 动态信息
	tagColliAvoidInfo CAInfo;	// 避让信息
};

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

struct tagMyPOINT
{
	double x; // nm
	double y; // nm
	double z; // nm
};

struct tagLINE // 直线的解析方程 a*x+b*y+c=0  为统一表示，约定 a >= 0
{
	double a;
	double b;
	double c;
	tagLINE(double d1 = 1, double d2 = -1, double d3 = 0)
	{
		a = d1;
		b = d2;
		c = d3;
	}
};

struct tagLineSeg
{
	tagMyPOINT s;
	tagMyPOINT e;
	tagLineSeg(tagMyPOINT a, tagMyPOINT b)
	{
		s = a;
		e = b;
	}
	tagLineSeg() {}
};

struct tagChartLineInfo
{
	int nID;		// 编号
	int nType;		// 线条的类型：0-安全等深线 1-航道边界 2-计划航线 3-面状物线条
	float fRangTOL; // 安全距离:nm
	float fGamma;	// positive parameter that is used to adjust the range of influence for the curve obstacle
	std::vector<tagLineInfo> tLineInfo;
};

struct tagFacePOINT
{
	int Num;		// 节点编号
	int nFatherNum; // 父节点编号
	double fLon;	// degree
	double fLat;	// degree
	double x;		// nm
	double y;		// nm
	double z;		// nm
	double g;
	tagLINE tl;		// 该点对应的线的方程-规范化之后的
	bool bReserved; // 标识是否为插入的点
};

class CConvexDecreasing;
struct tagChartFaceInfo
{
	// NOTE：输入默认最后一个点与第一个点连接，不需要将最后一个点与第一个点强制一致//如果最后一个点与每一个点一致，应该消除最后一个点
	int nID;		// 编号//
	int nType;		// 面状物体的类型：0-安全等深线 1-可航区域（如航道边界围成的多边形）
	float fAlpha;	//%决定六边形物标的斥力势场的陡峭程度，取值越小，越平滑，坡度越缓3;1;10，0.5
	float fRangTOL; // 安全距离:nm
	std::vector<tagFacePOINT> tPointInfo;
	CConvexDecreasing *mConvexDecreasing;
	bool bCovexDone; // 是进对多边形进行了初步处理
};

struct tagChartPointInfo
{
	int nID;		// 编号
	int nType;		// 点类型 0-孤立障碍物，如危险沉船，礁石 1-灯浮等助航标志
	float fRangTOL; // 安全距离:nm
	float fBeta;	// positive scalar
	tagPointInfo tPosInfo;
};

struct tagAvoidingTSStatus // 参与避碰的目标船属性
{
	int nID;				  //
	int nAvoType;			  // 0= 无 1=协商避碰，2= 紧急避碰
	TARGETPROPERTY nProperty; // 0= 对遇，1= 右舷A类船（正横前），2= 右舷B类船（正横后） 3=追越船 4=被追越船 5= 左舷来船
	std::string strName;	  //
	float fDCPA;
	float fTCPA;
	float fBearing;
	float fDistance;
};

struct tagPreAttitude // 本船运动态势预测
{
	double fLon; // degree
	double fLat; // degree
	float fPosX; // nm
	float fPosY; // nm
	float fPosZ; // nm

	float fSpd;			// m/s
	float fCos;			// degree
	float fSpdU;		// m/s
	float fSpdV;		// m/s
	float fHdg;			// degree
	float fROT;			// degree/min
	float fOrderRPM;	// 转/min
	float fRealRPM;		// 转/min
	float fOrderRudder; // degree
	float fRealRudder;	// degree
	tagPreAttitude()
	{
		fLon = 0.0;	 // degree
		fLat = 0.0;	 // degree
		fPosX = 0.0; // nm
		fPosY = 0.0; // nm
		fPosZ = 0.0; // nm

		fSpd = 0.0;			// m/s
		fSpdU = 0.0;		// m/s
		fSpdV = 0.0;		// m/s
		fHdg = 0.0;			// degree
		fROT = 0.0;			// degree/min
		fOrderRPM = 0.0;	// 转/min
		fRealRPM = 0.0;		// 转/min
		fOrderRudder = 0.0; // degree
		fRealRudder = 0.0;	// degree
	}
};

struct tagCAPathPoint
{
	double fLon; // degree
	double fLat; // degree
	float fPosX; // nm
	float fPosY; // nm
	float fPosZ; // nm
	float fCos;	 // degree
	float fSpd;	 // kn
	float fHdg;	 // degree
};
struct tagOptimizedCAPath
{
	bool bOptimized;
	float fDCPATOL;
	float fTCPATOL;
	std::vector<tagCAPathPoint> vPathPoints;
	tagOptimizedCAPath()
	{
		bOptimized = false;
		fDCPATOL = 0.0;
		fTCPATOL = 0.0;
		vPathPoints.resize(0);
	}
};

#endif
