#include "GridBox.h"



GridBox::GridBox(Point2d tl, Point2d tr, Point2d bl, Point2d br)
{
	vertx[0] = tl.x;
	verty[0] = tl.y;

	vertx[1] = tr.x;
	verty[1] = tr.y;

	vertx[2] = br.x;
	verty[2] = br.y;

	vertx[3] = bl.x;
	verty[3] = bl.y;

}

GridBox::GridBox()
{

}


GridBox::~GridBox()
{
}


bool pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
	int i, j;
	bool flag = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((verty[i]>testy) != (verty[j]>testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
			flag = !flag;
	}
	return flag;
}

bool GridBox::contains(double x, double y)
{
	return pnpoly(4, vertx, verty, x, y);
}
