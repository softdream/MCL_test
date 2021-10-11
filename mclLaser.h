#ifndef __MCL_LASER_H_
#define __MCL_LASER_H_

#include "dataType.h"
#include <vector>
#include <Eigen/Dense>

namespace mcl{

namespace sensor{

class MclLaser
{

public:
	MclLaser()
	{

	}

	~MclLaser()
	{

	}

	void addData( const Eigen::Vector2f &data )
	{
		return dataVec.push_back( data );
	}

	void clear()
	{
		return dataVec.clear();
	}

	const Eigen::Vector2f& getIndexData( const int index ) const
	{
		return dataVec[ index ];
	}

	const int getSize() const
	{
		return dataVec.size();
	}

private:
	std::vector<Eigen::Vector2f> dataVec;

};

}

}

#endif
