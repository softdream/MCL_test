#ifndef __MCL_H_
#define __MCL_H_

#include "mclMap.h"
#include <random>

#include "mclLaser.h"

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

	const Eigen::Vector3f& getPose(  ) const 
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
	Eigen::Vector3f pose = Eigen::Vector3f::Zero();
		
	// weight 	
	float weight = 0;
};

template<int ParticleNum = 1000>
class MCL
{

public:
	MCL()
	{

	}

	~MCL()
	{

	}

	// ------------------ Load A map from the bmp picture ----------------- //
	void initMclMapFromBMP( const std::string fileName )
	{
		map.loadMapFromBMP( fileName );

		map.showMapWithDistance();
	}


	// ------------------- Initialize the particles ------------------------ //
	void initializeParticles()	
	{
		gen.seed( rd() ); //Set random seed for random engine
		
		std::uniform_real_distribution<float> x_pose( static_cast<float>( map.getValidAreaMinX() ), static_cast<float>( map.getValidAreaMaxX() ) ) ;
                std::uniform_real_distribution<float> y_pose( static_cast<float>( map.getValidAreaMinY() ), static_cast<float>( map.getValidAreaMaxY() ) );
        	//std::uniform_real_distribution<float> x_pose( static_cast<float>( map.getValidAreaMinY() ), static_cast<float>( map.getValidAreaMaxY() ) ) ;
                //std::uniform_real_distribution<float> y_pose( static_cast<float>( map.getValidAreaMinX() ), static_cast<float>( map.getValidAreaMaxX() ) );
	
	        std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180 ~ 180 Deg
	
		int count = 0;
		while( count < ParticleNum ){
			Eigen::Vector3f pose;
			pose(0) = x_pose( gen );
			pose(1) = y_pose( gen );
			pose(2) = theta_pos( gen );

			if( map.getMapCellOccState( static_cast<int>( pose(1) ), static_cast<int>( pose(0) ) ) != -1 ){
				continue;
			}
			
			Particle p;
			p.setPose( pose );
			p.setWeight( 1 / static_cast<float>( ParticleNum ) );
			particles[count] = p;
			
			count ++;
		}
	}

	// ------------------ Display the map with particles --------------------- //
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
			//cv::circle( image, cv::Point( particles[i].getPoseX(), particles[i].getPoseY() ), 1, cv::Scalar( 0, 0, 255 ) );
			cv::Vec3b p;
                        p[0] = 0;
                        p[1] = 0;
                        p[2] = 255;
			image.at<cv::Vec3b>(particles[i].getPoseY(), particles[i].getPoseX()) = p;
		}

		cv::imshow( "map", image );
	}
	
	// ----------------- Odometry Motion Model ---------------- //
	// refer to <<Prababilistic Robotics>>
	const Eigen::Vector3f sampleMotionModelOdometry( const Eigen::Vector3f &p, const Eigen::Vector3f &pNew, const Eigen::Vector3f &pOld )
	{
		// delta_rot1 = atan2( y' - y, x' - x ) - theta
		float delta_rot1 = ::atan2( pNew(1) - pOld(1), pNew(0) - pOld(0) ) - pOld(2);
		// delta_trans = sqrt( ( x - x' )^2 + ( y - y' )^2 )
		float delta_trans = ::sqrt( ( pOld(0) - pNew(0) ) * ( pOld(0) - pNew(0) ) + ( pOld(1) - pNew(1) ) * ( pOld(1) - pNew(1) ) );
		// delta_rot2 = theta' - theta - delta_rot1
		float delta_rot2 = pNew(2) - pOld(2) - delta_rot1;

		//float delta_rot1_hat = delta_rot1 - sampleStandardNormalDistribution( alpha1_ * delta_rot1 * delta_rot1 + alpha2_ * delta_rot2 * delta_rot2 );
		//float delta_trans_hat = delta_trans - sampleStandardNormalDistribution( alpha3_ * delta_trans * delta_trans + alpha4_ * ( delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2 ) );
		//float delta_rot2_hat = delta_rot2 - sampleStandardNormalDistribution( alpha1_ * delta_rot2 * delta_rot2 + alpha2_ * delta_trans * delta_trans );

		float delta_rot1_hat = delta_rot1 - sampleStandardNormalDistribution( alpha1_ * delta_rot1 + alpha2_ * delta_rot2 );
                float delta_trans_hat = delta_trans - sampleStandardNormalDistribution( alpha3_ * delta_trans + alpha4_ * ( delta_rot1 + delta_rot2 ) );
                float delta_rot2_hat = delta_rot2 - sampleStandardNormalDistribution( alpha1_ * delta_rot2 + alpha2_ * delta_trans );

		Eigen::Vector3f noisy_p = Eigen::Vector3f::Zero();
		noisy_p(0) = p(0) + delta_trans_hat * ::cos( p(2) + delta_rot1_hat );
		noisy_p(1) = p(1) + delta_trans_hat * ::sin( p(2) + delta_rot1_hat );
		noisy_p(2) = p(2) + delta_rot1_hat + delta_rot2_hat;

		return noisy_p;
	}

	// ----------------- Likelihood field range finder model ----------------//
	const float likelihoodFieldRangeFinderModel( const mcl::sensor::MclLaser &scanPoints, const Eigen::Vector3f &poseInMap )
	{
		int size = scanPoints.getSize();
	
		float z_hit_denom = 2.0f * sigma_hit * sigma_hit;
    		float z_rand_mult = 1.0f / range_max;
	
		float p = 1.0f;

		// for all the laser beam
		for( int i = 0; i < size; i ++ ){
			// 1. get the current point in laser map frame
			Eigen::Vector2f currPointInScaleLaser( scanPoints.getIndexData( i ) * map.getCellLength() );
			
			// 2. get the current point in map frame
			Eigen::Vector2f currPointInMap( observedPointPoseLaser2Map( poseInMap ) );

			// 3. 
			int mi = static_cast<int>( currPointInMap(0) );
			int mj = static_cast<int>( currPointInMap(1) );

			// 4. get the distance 
			float dist = map.getMapCellOccDist( mi, mj );
				
			// 5. 
			float pz = 0.0f;
			pz += z_hit * ::exp( -( dist * dist ) / z_hit_denom );
			pz += z_rand * z_rand_mult;

			assert( pz <= 1.0f );
      			assert( pz >= 0.0f );
			
			// TODO ... May be changed according to different situation
			p += pz * pz;
		}	
	
		return p;
	}

	// -------------------- Low Variance ReSampling -------------------- //
	void lowVarianceSample()
	{	
		std::vector<Particle> particles_temp( particles, particles + ParticleNum );

		float r = ( rand() / (float)RAND_MAX) * (1.0f / (float)ParticleNum ); 

		float c = particles[0].getWeight();

		int i = 0;

		for (int m = 0; m < ParticleNum; m ++) {
			float u = r + (float)m / (float)ParticleNum ;
 		
			while (u > c && i < ParticleNum - 1){ 
				i++;				
				c += particles_temp[i].getWeight();	
			}
			particles[ m ] = particles_temp[i]; 	 
			particles[ m ].setWeight( 1.0f / ParticleNum );
		
			//std::cout << " each weight is : " << particles[m].getWeight() << std::endl;
		}	
	}

	// ------------------ Monte Carlo Localization ------------------ //
	void monteCarloLocalization( const mcl::sensor::MclLaser &scanPoints, const Eigen::Vector3f &poseInMapNew, const Eigen::Vector3f &poseInMapOld )
	{
		float totalWeight  = 0.0f;

		// for each particle
		for( int i = 0; i < ParticleNum; i ++ ){
			// 1. draw from the motion model
			Eigen::Vector3f samplingPose = sampleMotionModelOdometry( particles[i].getPose(), poseInMapNew, poseInMapOld );
			particles[i].setPose( samplingPose );

			// 2. caculate the particles' weight according to measurement model
			float weight = likelihoodFieldRangeFinderModel( scanPoints, samplingPose );
			particles[i].setWeight( weight );
			
			// 3. caculate the total weight 
			totalWeight += weight;
		
		}

		// 4. normalize the weight of each particle
		for( int i = 0; i < ParticleNum; i ++ ){
			float normalWeight = particles[i].getWeight() / totalWeight;
			particles[i].setWeight( normalWeight );
		}
		
		// 5. resampling 
		lowVarianceSample();

		// 6. TODO ... caculate the real robot pose in world frame
		
		// 7. visualize the results by opencv
		
	}


private:
	const Eigen::Vector2f observedPointPoseLaser2Map( const Eigen::Vector2f &pointInLaser, const Eigen::Vector3f &poseInMap )
	{
		Eigen::Matrix2f R;
		R << ::cos( poseInMap(2) ), -::sin( poseInMap(2) ),
		     ::sin( poseInMap(2) ),  ::cos( poseInMap(2) );
		
		return R * pointInLaser + poseInMap.head<2>();
	}

	const Eigen::Vector2f observedPointPoseMap2Laser( const Eigen::Vector2f &pointInMap, const Eigen::Vector3f &poseInWorld )
	{
		Eigen::Matrix2f R;
		R << ::cos( poseInWorld(2) ), -::sin( poseInWorld(2) ),
		     ::sin( poseInWorld(2) ),  ::cos( poseInWorld(2) );

		return R.inverse() * pointInMap - poseInWorld.head<2>();
	}


	const float sampleStandardNormalDistribution(const float var)
	{
		float sum = 0;
		for (int i = 0;i < 12; i++){
			//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
			sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
		}

		return (var / 6.0) * sum;
	}

	
private:
	
	// particles
	Particle particles[ParticleNum];

	// mcl map
	MclMap map;

	// random number generator c++11
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
  	std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()

	// parameters for motion model
	float alpha1_ = 0.025f, alpha2_ = 0.025f, alpha3_ = 0.4f, alpha4_ = 0.4f; 

	// parameters for measurement model
	float range_max = 10.0f, sigma_hit = 0.2f, z_hit = 0.8f, z_rand = 0.2f;
};

}

#endif
