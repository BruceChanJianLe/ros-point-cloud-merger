#ifndef AXIS_MANAGER
#define AXIS_MANAGER

#include <iostream>

namespace ros_util
{
    class AxisValue
    {
    private:
        int x;
        int y;
        int z;
        std::string input_topics;

        /* HERE */
        double pmin_range_x_;
        double pmin_range_y_;
        double pmin_range_z_;
        int input_size_;

    public:
        AxisValue();

        AxisValue(int a, int b, int c, std::string inputs)
        {
            setX(a);
            setY(b);
            setZ(c);
            setInputTopics(inputs);
        }

        int getX()
        {
            return x;
        }
        void setX(int input)
        {
            x = input;
        }

        int getY()
        {
            return y;
        }
        void setY(int input)
        {
            y = input;
        }

        int getZ()
        {
            return x;
        }
        void setZ(int input)
        {
            x = input;
        }

        std::string getInputTopics()
        {
            return input_topics;
        }
        void setInputTopics(std::string input)
        {
            input_topics = input;
        }

        /* HERE */
        AxisValue(double x, double y, double z, int input)
        {
            setXValue(x);
            setYValue(y);
            setZValue(z);
            setInputSize(input);
        }

        void setXValue(double pmin_range_x)
        {
            pmin_range_x_ = pmin_range_x;
        }

        void setYValue(double pmin_range_y)
        {
            pmin_range_y_ = pmin_range_y;
        }

        void setZValue(double pmin_range_z)
        {
            pmin_range_z_ = pmin_range_z;
        }

        void setInputSize(int input)
        {
            input_size_ = input;
        }

        double getXValue()
        {
            return pmin_range_x_;
        }

        double getYValue()
        {
            return pmin_range_y_;
        }

        double getZValue()
        {
            return pmin_range_z_;
        }

        int getInputSize()
        {
            return input_size_;
        }

        std::string checkInputSize()
        {
            std::string output = "Successful!";

            if (input_size_ < 2 || input_size_ > 8)
            {
                output = "Rejected! Out of bound input size.";
            }

            return output;
        }
    };
}

#endif