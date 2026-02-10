#ifndef _CONFIG_H
#define _CONFIG_H
enum ExecutionMode
{
    STEP_BY_STEP,
    HIGH_PERFORMANCE
};

class Config
{
private:
    Config();

public:
    ExecutionMode mExecutionMode = STEP_BY_STEP;
    static Config& GetInstance();
};


#endif
