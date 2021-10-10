#ifndef __MCL_MAP_H_
#define __MCL_MAP_H_

#include <iostream>
#include <vector>
#include <assert.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include "KDTree.hpp"

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
	float occDist = -1.0f;
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
	
	const int getMapCenterX() const
	{
		return mapCenterX;
	}

	const int getMapCenterY() const
	{
		return mapCenterY;
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

	void setMapCellOccDist( const int x, const int y, const float occDist )
	{
		return map[ cellIndex( x, y ) ].setOccDist( occDist );
	}

	void setMapCellOccState( const int x, const int y, const int occState )
	{
		return map[ cellIndex( x, y ) ].setOccState( occState );
	}

	void setMapCellOccState( const int index, const float occDist, const int occState )
	{
		return map[ index ].setOccDistAndState( occDist, occState );
	}

	void setMapCellOccDist( const int index, const float occDist )
        {
                return map[ index ].setOccDist( occDist );
        }

        void setMapCellOccState( const int index, const int occState )
        {
                return map[ index ].setOccState( occState );
        }

	const int getMapCellOccState( const int x, const int y ) 
	{
		return map[ cellIndex( x, y ) ].getOccState();
	}

	const int getMapCellOccState( const int index )
	{
		return map[ index ].getOccState();
	}

	const float getMapCellOccDist( const int x, const int y )
        {
                return map[ cellIndex( x, y ) ].getOccDist();
        }

        const float getMapCellOccDist( const int index )
        {
                return map[ index ].getOccDist();
        }

	const int getValidAreaMinX() const 
	{
		return min_x;
	}
	
	const int getValidAreaMinY() const
	{
		return min_y;
	}

	const int getValidAreaMaxX() const
	{
		return max_x;
	}

	const int getValidAreaMaxY() const
	{
		return max_y;
	}


	void loadMapFromTheOccupiedGridMap()
	{

	}

	void loadMapFromBMP( const std::string &fileName )
	{
		cv::Mat image = cv::imread( fileName, CV_8UC1 );
		std::cout<<"open the bmp map ..."<<std::endl;

		// get the valid area of the map
		//findValidAreaOfTheMap( image );

		std::vector<Eigen::Vector2d> freePoints;
		pointVec occupiedPoints;

		int occupiedCount  = 0;
		for (int i = 0; i < image.cols; i++) {
			for (int j = 0; j < image.rows; j++) {
				if( image.at<uchar>(i, j) == 0 ){ // occupied points
					occupiedCount ++; 	
					setMapCellOccState( i, j, 1 ); // set value as 1
					//std::cout<<"occupied int: ( "<<i<<", "<<j<<" )"<<std::endl;
					
					// added to the the vector
					//occupiedPoints.push_back( Eigen::Vector2f( static_cast<float>( i ), static_cast<float>( j ) ) );
					point_t pt = { static_cast<double>(i), static_cast<double>(j) };	
					occupiedPoints.push_back( pt );		
					//std::cout<<"occupied double: ( "<< pt[0] <<", "<<pt[1]<<" )"<<std::endl;
				}	
				else if( image.at<uchar>(i, j) == 255 ) { // free points
					setMapCellOccState( i, j, -1 ); // set value as -1
					freePoints.push_back( Eigen::Vector2d( static_cast<double>(i), static_cast<double>(j) ) );
				}
				else { // unknow points
					setMapCellOccState( i, j, 0 ); // set value as 0
				}
			}
		}

		std::cout<<"-------------------- Map -------------------"<<std::endl;
                std::cout<<"occupied point number: "<<occupiedCount<<std::endl;
		
		// get the valid area of the map	
		findValidAreaOfTheMap( occupiedPoints );

		// build a kd tree
		KDTree tree( occupiedPoints );
	
		for( auto it : freePoints ){ // for free points
			point_t pt = { it(0), it(1) };
			point_t ret = tree.nearest_point( pt );
			
			//std::cout<<"free point: "<<cacuDist( pt, ret )<<std::endl;
			setMapCellOccDist( static_cast<int>( it(0) ), static_cast<int>( it(1) ), cacuDist( pt, ret ) );
		}	

		for( auto it : occupiedPoints ){ 
			setMapCellOccDist( static_cast<int>( it[0] ), static_cast<int>( it[1] ), 0.0f );
			
			//std::cout<<"occupied : ( "<< it[0] <<", "<<it[1]<<" )"<<std::endl;
			for( int x = 0; x < kernelSize; x ++ ){ // for unknow points
				for( int y = 0; y < kernelSize; y ++ ) {
					int middle_kernel = ::floor( kernelSize / 2 );
					
					int index_x = static_cast<int>( it[0] ) - middle_kernel + x;
					int index_y = static_cast<int>( it[1] ) - middle_kernel + y;
				//	std::cout<<"( "<< index_x <<", "<<index_y<<" )"<<std::endl;	
		
					if( !isOutOfRange( index_x, index_y ) ) continue; // judge if the point is out of range
	
					if( getMapCellOccState( index_x, index_y ) == 0 && getMapCellOccDist( index_x, index_y ) == -1 ){ // avoid invalid caculation
						point_t pt = { static_cast<double>( index_x ), static_cast<double>( index_y ) };
						point_t ret = tree.nearest_point( pt );
						
						//std::cout<<"( "<< pt[0] <<", "<<pt[1]<<" ) & ( "<<ret[0]<<", "<<ret[1]<<" ) : " <<cacuDist( pt, ret )<<std::endl;
						setMapCellOccDist( index_x, index_y, cacuDist( pt, ret ) );
					}
				}
			}
			//std::cout<<"---------------------------------------------"<<std::endl;	
		}
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
				else if( getMapCellOccState( i, j ) == -1 ){ // free
					image.at<uchar>(i, j ) = 255;
				}
			}
		}
	
		std::cout<<"-------------------- Map -------------------"<<std::endl;
		std::cout<<"occupied point number: "<<occupiedCount<<std::endl;
	
		cv::imshow( "map", image );
		
		cv::waitKey(0);
	}

	void showMapWithDistance()
	{
		cv::Mat image = cv::Mat(sizeX, sizeY, CV_8UC1, cv::Scalar::all(255));
	
		for (int i = 0; i < image.cols; i++) {
                        for (int j = 0; j < image.rows; j++) {
                                if( getMapCellOccState( i, j ) == 1 ) { // occupied

                                        image.at<uchar>(i, j ) = 0;      // black
                                }
				else if( getMapCellOccState( i, j ) == 0 ){ // unknow points
					image.at<uchar>(i, j) = 255;	 // white
				}
				else if( getMapCellOccState( i, j ) == -1 ){ // free points
					float valueF = 20.0f * getMapCellOccDist( i, j );
					if( valueF > 255.0f ) valueF = 255.0f;
					image.at<uchar>(i, j) = static_cast<uchar>( valueF );
				}

				if( getMapCellOccDist(i, j ) > 0.0f && getMapCellOccState( i, j ) != -1 ){ // some unknow points
					float valueF = 40.0f * getMapCellOccDist( i, j );
                                        if( valueF > 255.0f ) valueF = 255.0f;
                                        image.at<uchar>(i, j) = static_cast<uchar>( valueF );
				}
                        }
                }

		cv::imshow( "map distance", image );
	
		cv::waitKey(0);
	}

private:
	const float cacuDist( const point_t &a, const point_t &b )
	{
		return static_cast<float>( ::sqrt( ( ( b[1] - a[1] ) * ( b[1] - a[1] ) ) + ( ( b[0] - a[0] ) * ( b[0] - a[0] ) )  ) );
	}

	const float nerestDistance( const int x, const int y, const std::vector<Eigen::Vector2f> &occupiedPoints )	
	{
		float minDist = 1000000.0f;
		
		Eigen::Vector2f point( static_cast<float>( x ), static_cast<float>( y ) );

		for( auto it : occupiedPoints ){
			float dist = ( point - it ).norm();
			
			if( dist < minDist ){
				minDist = dist;
			}
		}
		
		return minDist;
	}
	
	void findValidAreaOfTheMap(const cv::Mat &image)
	{
		//int max_x = 0, max_y = 0, min_x = 1000, min_y = 1000;

		for (int i = 0; i < image.cols; i++) {
			for (int j = 0; j < image.rows; j++) {
				if (image.at<uchar>(i, j) == 0) { // occupied points	
					if (j > max_x) {
						max_x = j;
					}

					if (i > max_y) {
						max_y = i;
					}

					if (j < min_x) {
						min_x = j;
					}

					if (i < min_y) {
						min_y = i;
					}
				}
			}
		}
		//cv::circle(image, cv::Point(min_x, min_y), 1, cv::Scalar(255));
		//cv::circle(image, cv::Point(max_x, max_y), 1, cv::Scalar(255));
		//cv::rectangle( image, cv::Point(min_x, min_y), cv::Point(max_x, max_y), cv::Scalar(255), 1 );

		std::cout << "min x: " << min_x << ", min y: "<<min_y<< std::endl;
		std::cout << "max x: " << max_x << ", max y: " << max_y << std::endl;
	}

        void findValidAreaOfTheMap(const pointVec &occupiedPoints)
        {

		for( auto it : occupiedPoints ){
			int i = static_cast<int>( it[0] );
			int j = static_cast<int>( it[1] );
                        
			if (j > max_x) {
                        	max_x = j;
                        }

                        if (i > max_y) {
                        	max_y = i;
                        }

                        if (j < min_x) {
	                        min_x = j;
	                }

                        if (i < min_y) {
                              	 min_y = i;
                        }
                }

                std::cout << "min x: " << min_x << ", min y: "<<min_y<< std::endl;
                std::cout << "max x: " << max_x << ", max y: " << max_y << std::endl;
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
	
	int mapCenterX = static_cast<int>( sizeX / 2 );
	int mapCenterY = static_cast<int>( sizeY / 2 );

	float scale = 0.1f;
	float cellLength = 10.0f;

	float maxOccDist = 50.0;

	int max_x = 0, max_y = 0, min_x = sizeX, min_y = sizeY;

	//std::vector<MapCell> map;
	MapCell *map;

	int kernelSize = 9;
};

}

#endif
