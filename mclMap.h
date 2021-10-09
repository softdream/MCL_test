#ifndef __MCL_MAP_H_
#define __MCL_MAP_H_

#include <iostream>
#include <vector>
#include <assert.h>

#include <opencv2/opencv.hpp>

namespace mcl{

class MapCell
{
public:
	MapCell()
	{

	}

	~MapCell()	
	{

	}

	void setOccDist( const float occDist )
	{
		this->occDist = occDist;
	}

	const float getOccDist() const
	{
		return occDist;
	}

	void setOccState( const int occState )
	{
		this->occState = occState;
	}

	const int getOccState() const
	{
		return occState;
	}

	void setOccDistAndState( const float occDist, const int occState )
	{
		this->occDist = occDist;
		this->occState = occState;
	}

private:
	float occDist = -1;
	int occState = 0;
};



class MclMap
{
public:
	MclMap()	
	{
		allocateMap();
	}

	MclMap( const int sizeX_, const int sizeY_, const float cellLength_ ) : sizeX( sizeX_ ), sizeY( sizeY_ ), cellLength( cellLength_ )
			
	{
		allocateMap();
	}

	~MclMap()	
	{
		deleteMap();
	}

	void setMapSize( const int sizeX, const int sizeY, const float cellLength )
	{
		this->sizeX = sizeX;
		this->sizeY = sizeY;
		this->cellLength = cellLength;
	}

	void setSizeX( const int sizeX )
	{
		this->sizeX = sizeX;
	}

	const int getSizeX() const
	{
		return sizeX;
	}
	
	void setSizeY( const int sizeY )
	{
		this->sizeY = sizeY;
	}

	const int getSizeY() const
	{
		return sizeY;
	}
	
	void setScale( const float scale )
	{
		this->scale = scale;
	}	

	const float getScale() const
	{
		return scale;
	}
	
	void setCellLength( const float cellLength )
	{
		this->cellLength = cellLength;
	}

	const float getCellLength() const
	{
		return cellLength;
	}

	void setMaxOccDist( const float maxOccDist )
	{
		this->maxOccDist = maxOccDist;
	}

	const float getMaxOccDist() const
	{
		return maxOccDist;
	}

	const int cellIndex( const int x, const int y )
	{
		return x + y * sizeY;
	}

	bool isOutOfRange( const int x, const int y )	
	{
		return ( (x >= 0) && (x < sizeX) && (y >= 0) && (y < sizeY) );
	}
	
	const MapCell getMapCell( const int x, const int y )
	{
		return map[ cellIndex( x, y ) ];
	}

	const MapCell getMapCell( const int index ) const
	{
		return map[ index ];
	}

	void setMapCell( const int x, const int y, const float occDist, const int occState )
	{
		return map[ cellIndex( x, y ) ].setOccDistAndState( occDist, occState );
	}

	void setMapCell( const int x, const int y, const float occDist )
	{
		return map[ cellIndex( x, y ) ].setOccDist( occDist );
	}

	void setMapCell( const int x, const int y, const int occState )
	{
		return map[ cellIndex( x, y ) ].setOccState( occState );
	}

	void setMapCell( const int index, const float occDist, const int occState )
	{
		return map[ index ].setOccDistAndState( occDist, occState );
	}

	void setMapCell( const int index, const float occDist )
        {
                return map[ index ].setOccDist( occDist );
        }

        void setMapCell( const int index, const int occState )
        {
                return map[ index ].setOccState( occState );
        }

	const int getMapCellOccState( const int x, const int y ) 
	{
		return map[ cellIndex( x, y ) ].getOccState();
	}

	void loadMapFromTheOccupiedGridMap()
	{

	}

	void loadMapFromBMP( const std::string &fileName )
	{
		cv::Mat image = cv::imread( fileName, CV_8UC1 );
		std::cout<<"open the bmp map ..."<<std::endl;
		//cv::imshow( "test", image );
		//cv::waitKey(0);

		int occupiedCount  = 0;
		for (int i = 0; i < image.cols; i++) {
			for (int j = 0; j < image.rows; j++) {
				if( image.at<uchar>(i, j) == 0 ){ // occupied points
					occupiedCount ++; 	
					setMapCell( i, j, 1 );
				}	
				else if( image.at<uchar>(i, j) == 255 ) { // free points
					setMapCell( i, j, -1 );
				}
				else { // unknow
					setMapCell( i, j, 0 );
				}
			}
		}

		std::cout<<"-------------------- Map -------------------"<<std::endl;
                std::cout<<"occupied point number: "<<occupiedCount<<std::endl;
	}

	void showMap()
	{
		cv::Mat image = cv::Mat(sizeX, sizeY, CV_8UC1, cv::Scalar::all(125));
		
		int occupiedCount  = 0;

		for (int i = 0; i < image.cols; i++) {
                        for (int j = 0; j < image.rows; j++) {
				if( getMapCellOccState( i, j ) == 1 ) { // occupied
					occupiedCount ++;				
	
					image.at<uchar>(i, j ) = 0;
				}		
				else if( getMapCellOccState( i, j ) == -1 ){
					image.at<uchar>(i, j ) = 255;
				}
			}
		}
	
		std::cout<<"-------------------- Map -------------------"<<std::endl;
		std::cout<<"occupied point number: "<<occupiedCount<<std::endl;
	
		cv::imshow( "map", image );
		
		cv::waitKey(0);
	}

private:
	bool allocateMap()
	{
		map = new MapCell[ sizeX * sizeY ];
	
		if( map == nullptr ){
			std::cerr<<"Can not allocate the memory for the map ..."<<std::endl;

			assert( map );
		}
		
		std::cerr<<"Allocate the memory for the map ..."<<std::endl;

		return true;
	}
	
	void deleteMap()
	{
		if( map == nullptr ){
			std::cerr<<"Can not release the memory for the map ..."<<std::endl;
			assert( map );
		}
	
		delete[] map;
		
		if( map != nullptr ){
			std::cerr<<"Release The memory failed ..."<<std::endl;
			assert( map );
		}

		std::cerr<<"Release the memory for the map ..."<<std::endl;		
	}	

	void reAllocateMap( const int newSizeX, const int newSizeY )
	{
		if( map == nullptr ){
			map = new MapCell[ newSizeX * newSizeY ];
			
			if( map == nullptr ){
				std::cerr<<"reallocate memory failed ..."<<std::endl;
				assert( map );
			}
		}
		else {
			delete[] map;
			
			if( map != nullptr ){
				std::cerr<<"Release the memory failed ..."<<std::endl;
				assert( map );
			}
		
			map = new MapCell[ newSizeX * newSizeY ];

                        if( map == nullptr ){
                                std::cerr<<"reallocate memory failed ..."<<std::endl;
                                assert( map );
                        }

		}
	}
	
private:
	int sizeX = 1001;
	int sizeY = 1001;
	float scale = 0.1f;
	float cellLength = 10.0f;

	float maxOccDist = 50.0;

	//std::vector<MapCell> map;
	MapCell *map;
};

}

#endif
