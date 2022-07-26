#include <stdio.h>

namespace agv_controller_ns
{
    class Test_class
    {
    private:
        /* data */
    public:
        Test_class(/* args */)
        
        {
            printf("test Class created");
        };
        void print_test()
        { printf("test Class test run");

        }

        ~Test_class();
    };
    class B
    {
            public:
        B()
        {
            test=new Test_class();
        }
        void b_test()
        {
            test->print_test();
        }
        ~B()
        {};
        private:
        Test_class* test;


    };
}