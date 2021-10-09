#include "mclMap.h"

int main()
{
	std::cout<<"--------------- Program Begins ---------------"<<std::endl;
	
	
	mcl::MclMap map;
	
	map.loadMapFromBMP( "./test.bmp" );	

	map.showMap( );

	return 0;
}
