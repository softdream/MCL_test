#ifndef __MCL_H_
#define __MCL_H_

#include "mclMap.h"


namespace mcl{

class Particle
{
public:
	Particle()
	{

	}

	~Particle()
	{

	}

	Particle( const Particle &rhs ) : pose( rhs.pose ),
					  weight( rhs.weight )
	{

	} 

	const Particle& operator=( const Particle &rhs )
	{
		if( &rhs == this ){
			return *this;
		}

		pose = rhs.pose;
		weight = rhs.weight;

		return *this;
	}

	void setPose( const Eigen::Vector3f &pose )
	{
		this->pose = pose;
	}

	void setPose( const float x, const float y, const float theta )
	{
		this->pose = Eigen::Vector3f( x, y, theta );
	}

	const Eigen::Vector3& getPose(  ) const 
	{
		return pose;
	}
	
	const float getPoseX() const 
	{
		return pose(0);
	}

	const float getPoseY() const
	{
		return pose(1);
	}

	const float getPoseTheta() const
	{
		return pose(2);
	}

	void setWeight( const float weight )
	{
		this->weight = weight;
	}

	const float getWeight() const
	{
		return weight;
	}

private:
	// state variable
	Eigen::Vector3f pose( 0.0f, 0.0f, 0.0f );
		
	// weight 	
	float weight = 0;
};

template<int ParticleNum>
class MCL
{

public:
	MCL()
	{

	}

	~MCL()
	{

	}

	void initMclMapFromBMP( const std::string fileName )
	{
		map.loadMapFromBMP();

		map.showMapWithDistance();
	}

	void initializeParticles()	
	{
		gen.seed( rd() ); //Set random seed for random engine
		
		std::uniform_real_distribution<float> x_pose( static_cast<float>( map.getValidAreaMinX(), static_cast<float>( map.getValidAreaMaxX ) ) );
                std::uniform_real_distribution<float> y_pose( static_cast<float>( map.getValidAreaMinY(), static_cast<float>( map.getValidAreaMaxY ) ) );
                std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180 ~ 180 Deg
	
		int count = 0;
		while( count < ParticleNum ){
			Eigen::Vector3f pose;
			pose(0) = x_pose( gen );
			pose(1) = y_pose( gen );
			pose(2) = theta_pos( gen );

			if( map.setMapCellOccState( static_cast<int>( pose(0) ), static_cast<int>( pose(1) ) ) != -1 ){
				continue;
			}
			
			Particle p;
			p.setPose( pose );
			p.setWeight( 1 / static_cast<float>( ParticleNum ) );
			particles[count] = p;
			
			count ++;
		}
	}

	void showMap()
	{
		cv::Mat image = cv::Mat( map.getSizeX(), map.getSizeY(), CV_8UC3, cv::Scalar( 125, 125, 125 ) );

		for( int i = 0; i < map.getSizeX(); i ++ ){
			for( int j = 0; j < map.getSizeY(); j ++ ){
				if( map.getMapCellOccState( i, j ) == -1 ){ // free points
					cv::Vec3b p;
					p[0] = 255;
					p[1] = 255;
					p[2] = 255;	
					
					image.at<cv::Vec3b>(i, j) = p;
				}
				else if( map.getMapCellOccState( i, j ) == 1 ){ // occupied points
					cv::Vec3b p;
                                        p[0] = 0;
                                        p[1] = 0;
                                        p[2] = 0;

                                        image.at<cv::Vec3b>(i, j) = p;
				}
			}
		}

		for( int i = 0; i < ParticleNum; i ++ ){
			cv::circle( image, cv::Point( particles[i].getPoseX(), particles[i].getPoseY() ), 1, cv::Scalar( 0, 0, 255 ) );
		}

		cv::imshow( "map", image );
	}
	
private:
	
	// particles
	Particle particles[ParticleNum];

	// mcl map
	MclMap map;

	// random number generator c++11
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
  	std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
};

}

#endif
