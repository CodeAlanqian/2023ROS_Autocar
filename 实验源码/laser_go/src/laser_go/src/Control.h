namespace Control
{
    class PID 
    {
    public:
        double kp;
        double ki;
        double kd;
        double error_out;
        double last_error;
        double integral;
        double inte_max;//积分限副
        double last_diff;//上一次微分值

        double PIDPositional(double error);//位置式PID控制器
        double PIDIncremental(double error);
        void Init();
        
    };
    
    double PID::PIDPositional(double error)//位置式PID控制器
    {
        integral += error;

        if(integral>inte_max)
            integral = inte_max;

        error_out = (kp*error) + (ki*integral) + (kd*(error-last_error));
        last_error = error;
        return error_out;
    }
    
    double PID::PIDIncremental(double error)//增量式PID控制器
    {
        error_out = kp*(error-last_error) + ki*error + kd*((error-last_error)-last_diff);

        last_diff = error - last_error;
        last_error = error;

        return error_out;
    }

    void PID::Init()//PID初始化,参数在此调节
    {
        kp =14.0;
        ki = 0.0;
        kd = 6.1;
        error_out = 0.0;
        last_error = 0.0;
        integral = 0.0;
        inte_max = 8.0;
        last_diff = 0.0;
        // kp =18.0;
        // ki = 0.1;
        // kd = 3.0;
        // error_out = 0.0;
        // last_error = 0.0;
        // integral = 0.0;
        // inte_max = 8.0;
        // last_diff = 0.0;
    }
    
}