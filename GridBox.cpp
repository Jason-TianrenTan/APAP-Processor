#include "GridBox.h"



GridBox::GridBox(Point2d tl, Point2d tr, Point2d bl, Point2d br)
{
	topleft = tl;
	topright = tr;
	botleft = bl;
	botright = br;
}

GridBox::GridBox()
{

}


GridBox::~GridBox()
{
}


bool GridBox::contains(double x, double y)
{
	Point2d pt = Point2d(x, y);
	return (pt.x >= topleft.x) && (pt.y >= topleft.y)
		&& (pt.x <= topright.x) && (pt.y >= topright.y)
		&& (pt.x >= botleft.x) && (pt.y <= botleft.y)
		&& (pt.x <= botright.x) && (pt.y <= botright.y);
}
