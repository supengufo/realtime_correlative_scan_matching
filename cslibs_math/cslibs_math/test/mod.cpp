#include <gtest/gtest.h>

#include <cslibs_math/common/mod.hpp>

TEST(Test_cslibs_math, testMod)
{
    EXPECT_EQ(cslibs_math::common::mod(-11101,100), 99);
    EXPECT_EQ(cslibs_math::common::mod(5, 3), 2);
    EXPECT_EQ(cslibs_math::common::mod(3, 5), 3);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
