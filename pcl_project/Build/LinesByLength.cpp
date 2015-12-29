#include "stdafx.h"
#include "LinesByLength.h"


LinesByLength::LinesByLength()
{
}


LinesByLength::~LinesByLength()
{
}

LinesByLength::LinesByLength(MyLines line)
{
	linelength = line.getlength();
	myliness.push_back(line);
};

void LinesByLength::Add(MyLines li){
	myliness.push_back(li);
};

float LinesByLength::getlength() { return linelength; };
std::vector<MyLines> LinesByLength::getLines(){ return myliness; };

