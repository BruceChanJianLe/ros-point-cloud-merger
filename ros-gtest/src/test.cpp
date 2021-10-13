#include "gtest/gtest.h"
#include "cmath"
#include "iostream"
/* #include "AxisManager.h" */

class AxisValue
{
private:
    int x;
    int y;
    int z;
    std::string input_topics;

public:
    AxisValue();

    AxisValue(int a, int b, int c, std::string inputs){ 
        setX(a);
        setY(b);
        setZ(c);
        setInputTopics(inputs);
    }

    int getX(){
        return x;
    }
    void setX(int input){
        x = input;
    }

    int getY(){
        return y;
    }
    void setY(int input){
        y = input;
    }

    int getZ(){
        return z;
    }
    void setZ(int input){
        z = input;
    }

    std::string getInputTopics(){
        return input_topics;
    }
    void setInputTopics(std::string input){
        input_topics = input;
    }
};

TEST(TestCase1, testName1){
    AxisValue value(4, 5, 6, "/points");

    ASSERT_NE(5, value.getX());
    EXPECT_EQ(4, value.getX());
    EXPECT_NE(4, value.getY());
    EXPECT_NE(4, value.getZ());
}

TEST(TestCase2, testName2){
    AxisValue value(4, 5, 6, "/points");
    std::string compare = "/points";

    ASSERT_EQ(compare, value.getInputTopics());
}

TEST(TestCase3, testName3){
    AxisValue value(4, 5, 6, "/points");
    std::string compare = "/POINTS";

    EXPECT_NE(compare, value.getInputTopics());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    int ret = RUN_ALL_TESTS();

    return ret;
}