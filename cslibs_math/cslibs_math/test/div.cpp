#include <gtest/gtest.h>

#include <cslibs_math/common/div.hpp>

TEST(Test_cslibs_math, testDiv)
{
    EXPECT_EQ(cslibs_math::common::div(-11101, 100), -112);
    EXPECT_EQ(cslibs_math::common::div(-1, 100), -1);
    EXPECT_EQ(cslibs_math::common::div(-2, 100), -1);
    EXPECT_EQ(cslibs_math::common::div(-100, 100), -1);
    EXPECT_EQ(cslibs_math::common::div(5, 3), 1);
    EXPECT_EQ(cslibs_math::common::div(3, 5), 0);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
