#ifndef MAP_INFLATION_TOOL_H
#define MAP_INFLATION_TOOL_H

#include <queue>

#include <ros/ros.h>
#include <csc_nav2d_navigator/GridMap.h>

class CellData
{
public:
	CellData(double d, double i, unsigned int sx, unsigned int sy);
	double distance;
	unsigned int index;
	unsigned int sx, sy;
};

inline bool operator<(const CellData &a, const CellData &b)
{
	return a.distance > b.distance;
}


class MapInflationTool
{
public:
	MapInflationTool();
	~MapInflationTool();
	
	void computeCaches(unsigned int radius);
	void inflateMap(GridMap* map);
	
private:
	void enqueueObstacle(unsigned int index, unsigned int sx, unsigned int sy);
	inline double distanceLookup(int mx, int my, int src_x, int src_y);
	inline char costLookup(int mx, int my, int src_x, int src_y);
	
	GridMap* mGridMap;
	
	unsigned int mCellInflationRadius;
	char** mCachedCosts;
	double ** mCachedDistances;
	
	std::priority_queue<CellData> mInflationQueue;  // 优先队列，最大的元素在队首（重载运算符<，距离小的Cell优先级高）
	unsigned char* mInflationMarkers;
	
	char mCostObstacle;
//	char mCostLethal;
};

class LocalMapInflationTool
{
public:
	LocalMapInflationTool();
	~LocalMapInflationTool();

	void computeCaches(unsigned int radius_a, unsigned int radius_b);
	void inflateMap(GridMap* map);

private:
	void enqueueObstacle(unsigned int index, unsigned int sx, unsigned int sy);
	inline double distanceLookup(int mx, int my, int src_x, int src_y);
	inline char costLookup(int mx, int my, int src_x, int src_y);

	GridMap* mGridMap;

	unsigned int mCellInflationRadius_a;
	unsigned int mCellInflationRadius_b;
	char** mCachedCosts;
	double ** mCachedDistances;

	std::priority_queue<CellData> mInflationQueue;  // 优先队列，最大的元素在队首（重载运算符<，距离小的Cell优先级高）
	unsigned char* mInflationMarkers;

	char mCostObstacle;
//	char mCostLethal;
};

#endif
