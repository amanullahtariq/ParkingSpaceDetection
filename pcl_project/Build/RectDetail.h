#pragma once

class RectDetail
{
private:
	float length;
	float lCount;
	float width;
	float lWidth;
public:
	RectDetail();
	~RectDetail();
	RectDetail(int l, bool isLength);
	RectDetail(int w);
	void AddLength(int l);
	void AddLengthCount();
	void AddWidth(int w);
	void AddWidthCount();
	float getLengthCount();
	float getLength();

	float getWidthCount();
	float getWidth();
};

