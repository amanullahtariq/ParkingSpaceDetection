#include "stdafx.h"
#include "RectDetail.h"


RectDetail::RectDetail()
{
}


RectDetail::~RectDetail()
{
}

RectDetail::RectDetail(int l, bool isLength)
{
	lCount = 0;
	lWidth = 0;
	length = l;
	lCount++;
}

RectDetail::RectDetail(int w)
{
	width = w;
	lWidth++;
}


void RectDetail::AddLength(int l)
{
	length = l;
}
void RectDetail::AddLengthCount()
{
	lCount++;
}

void RectDetail::AddWidth(int w)
{
	width = w;
}
void RectDetail::AddWidthCount()
{
	lWidth++;
}

float RectDetail::getLengthCount()	{ return lCount; }
float RectDetail::getLength()	{ return length; }

float RectDetail::getWidthCount()	{ return lWidth; }
float RectDetail::getWidth()	{ return width; }
