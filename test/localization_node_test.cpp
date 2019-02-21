#include <gtest/gtest.h>
#include "indoor_localization/AnchorScan.h"
#include "../include/localization_node.h"

  // coordinates of anchors
  double AX = 7.79;
  double AY = 0.96;
  double AZ = 1.64;

  double BX = 0.92;
  double BY = 0.16;
  double BZ = 1.21;

  double SX = 8.3;
  double SY = 4.085;
  double SZ = 1.945;

  double TZ = 1.72;

  // TDOA values
  double dist_diff_A_S = -1.053;
  double dist_diff_B_S = -4.998;


TEST(IndoorTest , value)
{

	pos tagpos;

	tagpos = calcPos2D3AIte(SX, SY, SZ, AX, AY, AZ, BX, BY, BZ, TZ, dist_diff_A_S, dist_diff_B_S);

	ASSERT_EQ(tagpos.TXcm,213);
	ASSERT_EQ(tagpos.TYcm,173);
	ASSERT_EQ(tagpos.TZcm,172);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc,argv);
	ros::init(argc,argv,"localization_node_test");
	ros::NodeHandle nh;
	
	return RUN_ALL_TESTS();
}
