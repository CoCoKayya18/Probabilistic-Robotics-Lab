class EKF_Slam
{
    public:
        EKF_Slam();
        ~EKF_Slam();

        void prediction_step();
        void correction_step();

    private:
        double mean;
        // add covariance matrix as sigma here
};