#include <vector>

class EKF_Slam
{
    public:
        EKF_Slam(){};
        ~EKF_Slam(){};

        void prediction_step();
        void correction_step();

    private:
        std::vector <double> mean;
        double covarianceMatrix [3][3];
        double map [1][1];
};