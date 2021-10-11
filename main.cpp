#include "mclMap.h"
#include "mcl.h"

int main()
{
	std::cout<<"--------------- Program Begins ---------------"<<std::endl;
	
	
	/*mcl::MclMap map;
	
	map.loadMapFromBMP( "./test.bmp" );	

	map.showMap( );

	map.showMapWithDistance();	
*/
	mcl::MCL<1000> mcl;
	mcl.initMclMapFromBMP( "./test.bmp" );

	mcl.initializeParticles();

	mcl.showMap();
	cv::waitKey(0);

	return 0;
}
