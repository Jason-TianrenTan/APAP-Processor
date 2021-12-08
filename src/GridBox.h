#pragma once
#include "stdafx.h"
class GridBox
{
public:
	double vertx[4], verty[4];
	GridBox(Point2d tl, Point2d tr, Point2d bl, Point2d br);
	GridBox();
	~GridBox();
	bool contains(double x, double y);
};

