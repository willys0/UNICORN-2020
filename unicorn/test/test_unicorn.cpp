// Bring in my package's API, which is what I'm testing
#include "unicorn/unicorn_statemachine.h"
// Bring in gtest
#include <gtest/gtest.h>

/**********************************************
 * To declare a test:
 * TEST(UnicornTestSuite, <enter test case name>)
 * {
 *   <write code to test>
 * }
 * 
 * A reference to ASSERT_* and EQ_* MACROS can be found here:
 * http://cheezyworld.com/wp-content/uploads/2010/12/PlainGoogleQuickTestReferenceGuide1.pdf
 * 
 **********************************************/

// Declare a test
TEST(TestSuite, testCase1)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
