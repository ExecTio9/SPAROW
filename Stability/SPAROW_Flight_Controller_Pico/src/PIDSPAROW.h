





class PIDSPAROW
{
    public:
    PIDSPAROW(double*, double*, double*,double, double, double, int);

    bool compute();

    void SetOutputLimits(double, double);

    //Display Functions
    double GetKp();
    double GetKi();
    double GetKd();

    private:
    void Initialize();
    
};