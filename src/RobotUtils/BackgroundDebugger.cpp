#include "BackgroundDebugger.h"

BackgroundDebugger::BackgroundDebugger(double debugInterval, bool clearContents) :
    HotSubsystem("BackgroundDebugger")
{
    Preferences* prefs;

    prefs = Preferences::GetInstance();

    m_debugInterval = debugInterval;
    m_manualLog = "manualLog.txt";
    m_fout = new ofstream;
    m_csv = new CSVWriter;

    //Auton setup
    m_autonCase = NULL;
    m_lastCase = 0;
    m_endCase = 100;
    m_caseDuration = AUTON_CASE_DURATION;
    m_caseTime = NULL;
    f_autonState = true;
    f_watchAuton = true;

    f_running = false;
    f_delContents = clearContents;
    m_runNum = prefs->GetInt("DebugRun");
    m_runPath = "/home/lvuser/DisabledLogs";

    if (!S_ISDIR(stat(m_runPath.c_str())))
    {
        mkdir(m_runPath.c_str());
        m_runPath += '/';
    }
    else
    {
        m_runPath += '/';

        struct dirent* curFile;
        DIR* folder;
        string filepath;

        folder = opendir(m_runPath.c_str());

        while (curFile = readdir(folder))
        {
            if (strcmp(curFile->d_name,".") != 0 && strcmp(curFile->d_name,"..") != 0)
            {
                filepath = m_runPath+curFile->d_name;

                m_fout->open(filepath.c_str(), ios::app);

                if (m_fout->is_open())
                    (*m_fout)<<"**REBOOT**"<<endl;
            }
        }
    }
}

BackgroundDebugger::~BackgroundDebugger()
{
    if (m_fout->is_open())
        m_fout->close();
}

void BackgroundDebugger::AddValue(string id, FUNCPTR function)
{
    FuncData funcData;

    funcData.id = id;
    funcData.function = function;

    m_funcList.push_back(funcData);
}

void BackgroundDebugger::AddValue(string id, double *value)
{
    NumData numData;

    numData.id = id;
    numData.value = value;

    m_numList.push_back(numData);
}

void BackgroundDebugger::AddValue(string id, string *value)
{
    StringData stringData;

    stringData.id = id;
    stringData.value = value;

    m_numList.push_back(stringData);
}

void BackgroundDebugger::LogData(string id, double value)
{
    time_t currentTime;

    time(&currentTime);
    m_fout->open(m_runPath.c_str()+m_manualLog.c_str(), ios::app);

    if (m_fout->is_open())
    {
        (*m_fout)<<ctime(currentTime)<<' '<<id<<' '<<value<<endl;
        m_fout->close();
    }
}

void BackgroundDebugger::LogData(string id, string value)
{
    time_t currentTime;

    time(&currentTime);
    m_fout->open(m_runPath.c_str()+m_manualLog.c_str(), ios::app);

    if (m_fout->is_open())
    {
        (*m_fout)<<ctime(currentTime)<<' '<<id<<' '<<value<<endl;
        m_fout->close();
    }
}

void BackgroundDebugger::ResetRunNumber()
{
    if (!f_running)
        m_runNum = 0;
}

void BackgroundDebugger::StartRun()
{
    m_runNum++;
    m_runPath = "/home/lvuser/Run"+m_runNum;

    time(&m_startTime);

    if (!S_ISDIR(stat(m_runPath.c_str())))
    {
        mkdir(m_runPath.c_str());
        m_runPath += '/';
    }
    else if (f_delContents)
    {
        m_runPath += '/';

        struct dirent* curFile;
        DIR* folder;
        string filepath;

        folder = opendir(m_runPath.c_str());

        while (curFile = readdir(folder))
        {
            if (strcmp(curFile->d_name, ".") != 0 && strcmp(curFile->d_name, ".."))
            {
                filepath = m_runPath+curFile->d_name;
                remove(filepath.c_str());
            }
        }
    }

    m_csv->open(m_runPath.c_str()+"data.csv");
    m_csv->setColumns(m_numList.size()+m_stringList.size()+m_funcList.size()+1);

    if (m_csv->is_open())
    {
        m_csv->writeCell("Time");

        for (x = 0; x < m_funcList.size(); x++)
            m_csv->writeCell(m_funcList[x].id);
        for (int x = 0; x < m_numList.size(); x++)
            m_csv->writeCell(m_numList[x].id);
        for (x = 0; x < m_stringList.size(); x++)
            m_csv->writeCell(m_stringList[x].id);
    }

    f_running = true;
}

void BackgroundDebugger::StopRun()
{
    Preferences* prefs;

    m_csv->close();

    prefs = Preferences::GetInstance();

    prefs->PutInt("DebugRun",m_runNum);
    prefs->Save();

    f_running = false;
    m_runPath = "/home/lvuser/DisabledLogs/";
}

void BackgroundDebugger::Update()
{
    if (f_running)
    {
        int x;
        string message;

        message = m_tempMsg;

        //Logging
        if ((difftime(time(NULL),m_lastDebugTime)*1000.0) >= m_debugInterval)
        {
            if (m_csv->is_open())
            {
                m_csv->writeCell((float)difftime(time(NULL),m_startTime));
                if (message != "")
                {
                    m_csv->writeCell(message);
                    m_csv->newRow();
                    m_csv->writeCell((float)difftime(time(NULL),m_startTime));
                }

                for (x = 0; x < m_funcList.size(); x++)
                    m_csv->writeCell(m_funcList[x].function());
                for (x = 0; x < m_numList.size(); x++)
                    m_csv->writeCell(*m_numList[x].value);
                for (x = 0; x < m_stringList.size(); x++)
                    m_csv->writeCell(*m_stringList[x].value);
            }

            time(&m_lastDebugTime);
            if (m_tempMsg.compare(message) == 0)
                m_tempMsg = "";
        }

        if (DriverStation::GetInstance()->IsAutonomous())
        {
            if (m_caseTime == NULL)
                time(&m_caseTime);

            watchAuton();
        }
        else if (m_caseTime != NULL)
            m_caseTime = NULL;
    }
}

void BackgroundDebugger::watchAuton()
{
    if ((*m_autonCase > 0 && *m_autonCase < m_endCase) && f_watchAuton && f_autonState)
    {
        if (*m_autonCase > m_lastCase)
        {
            time(&m_caseTime);
            m_lastCase = *m_autonCase;
        }

        if (difftime(time(NULL),m_caseTime) > m_caseDuration)
        {
            f_autonState = false;
            dumpAuton();
        }
    }
}

void BackgroundDebugger::dumpAuton()
{
    m_fout->open(m_runPath+"AUTON_FAIL.txt");

    (*m_fout)<<"AUTONOMOUS FAILURE DETECTED AT "<<DriverStation::GetInstance()->GetMatchTime()<<" s"<<endl<<endl;
    (*m_fout)<<"BackgroundDebugger created this file because it detected that case "<<(*m_autonCase);
    (*m_fout)<<" of the autonomous mode you ran did not advance to the next case inside of "<<m_caseDuration<<" seconds."<<endl<<endl;
    (*m_fout)<<"Below is a dump of all sensor and time data available to BackgroundDebugger."<<endl<<endl;
    (*m_fout)<<"Maximum configured case duration: "<<m_caseDuration<<endl<<"Case run time: "<<difftime(time(NULL),m_caseTime)<<endl<<endl;

    for (int x = 0; x < m_funcList.size(); x++)
        (*m_fout)<<m_funcList[x].id<<": "<<m_funcList[x].function()<<endl;
    for (int x = 0; x < m_numList.size(); x++)
        (*m_fout)<<m_numList[x].id<<": "<<(*m_numList[x].value)<<endl;
    for (int x = 0; x < m_stringList.size(); x++)
        (*m_fout)<<m_stringList[x].id<<": "<<(*m_stringList[x].value)<<endl;

    m_fout->close();
}
