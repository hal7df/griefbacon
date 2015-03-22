#include "BackgroundDebugger.h"

BackgroundDebugger::BackgroundDebugger(double debugInterval, bool clearContents) :
    HotSubsystem("BackgroundDebugger")
{
    Preferences* prefs;
    struct stat st;

    prefs = Preferences::GetInstance();

    m_debugInterval = debugInterval;
    m_manualLog = "manualLog.csv";
    m_concat = new stringstream (ios::in | ios::out);
    m_csv = new CSVWriter;
    m_manualCsv = new CSVWriter;
    m_manualCsv->setColumns(3);

    m_startTime = new time_t;
    m_lastDebugTime = new time_t;

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
    m_runPath = "/home/lvuser/DisabledLogs/";

    (*m_concat) << m_runPath << m_manualLog;
    (*m_concat) >> m_manualLogPath;

    m_concat->str("");
    m_concat->clear();

    if (!S_ISDIR(stat(m_runPath.c_str(),&st)))
    {
        mkdir(m_runPath.c_str(), 0755);
    }
    else
    {
        struct dirent* curFile;
        DIR* folder;
        string filepath;

        folder = opendir(m_runPath.c_str());

        while (curFile = readdir(folder))
        {
            if (strcmp(curFile->d_name,".") != 0 && strcmp(curFile->d_name,"..") != 0)
            {
                filepath = m_runPath+curFile->d_name;

                m_fout.open(filepath.c_str(), ios::app);

                if (m_fout.is_open())
                    m_fout<<"**REBOOT**"<<endl;
            }
        }
    }
}

BackgroundDebugger::~BackgroundDebugger()
{
	StopRun();

    if (m_fout.is_open())
        m_fout.close();

    if (m_csv->is_open())
    	m_csv->close();

    if (m_manualCsv->is_open())
    	m_manualCsv->close();
}

void BackgroundDebugger::AddValue(string id, PIDSource* value)
{
    SensorData sensorData;

    sensorData.id = id;
    sensorData.source = value;

    m_sensorList.push_back(sensorData);
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

    m_stringList.push_back(stringData);
}

void BackgroundDebugger::LogData(string id, double value)
{
    LogData(id,to_string(value));
}

void BackgroundDebugger::LogData(string id, string value)
{
	struct tm curTime;
	time_t t;

    char buf [80];
    t = time(NULL);
    curTime = *localtime(&t);
    strftime(buf, 80, "%X", &curTime);

    if (!m_manualCsv->is_open())
    	m_manualCsv->open(m_manualLogPath.c_str());

    if (m_manualCsv->is_open())
    {
    	m_manualCsv->writeCell(string(buf));
        m_manualCsv->writeCell(id);
        m_manualCsv->writeCell(value);
    }
}

void BackgroundDebugger::CloseFile()
{
	if (m_fout.is_open())
		m_fout.close();

	if (m_manualCsv->is_open())
		m_manualCsv->close();
}

void BackgroundDebugger::ResetRunNumber()
{
    if (!f_running)
        m_runNum = 0;
}

void BackgroundDebugger::StartRun()
{
	unsigned x;
	struct stat st;
	std::string tmp;

	CloseFile();

    m_runNum++;
    (*m_concat) << "/home/lvuser/Run"<<m_runNum<<'/';
    (*m_concat) >> m_runPath;

    m_manualLogPath = m_runPath + m_manualLog;

    m_concat->str("");
    m_concat->clear();

    time(m_startTime);

    if (!S_ISDIR(stat(m_runPath.c_str(),&st)))
    {
        mkdir(m_runPath.c_str(), 0755);
    }
    else if (f_delContents)
    {
        struct dirent* curFile;
        DIR* folder;
        string filepath;

        folder = opendir(m_runPath.c_str());

        while (curFile = readdir(folder))
        {
            if (strcmp(curFile->d_name, ".") != 0 && strcmp(curFile->d_name, "..") != 0)
            {
                filepath = m_runPath+curFile->d_name;
                remove(filepath.c_str());
            }
        }
    }

    (*m_concat) << m_runPath << "data.csv";
    (*m_concat) >> tmp;

    m_csv->open(tmp.c_str());
    m_csv->setColumns(m_numList.size()+m_stringList.size()+m_sensorList.size()+1);

    if (m_csv->is_open())
    {
        m_csv->writeCell(string("Time"));

        vector<SensorData>::iterator snit;
        vector<NumData>::iterator nit;
        vector<StringData>::iterator stit;

        for (snit = m_sensorList.begin(); snit != m_sensorList.end(); ++snit)
        	m_csv->writeCell((*snit).id);

        for (nit = m_numList.begin(); nit != m_numList.end(); ++nit)
        	m_csv->writeCell((*nit).id);

        for (stit = m_stringList.begin(); stit != m_stringList.end(); ++stit)
        	m_csv->writeCell((*stit).id);
    }
    f_running = true;

    m_concat->str("");
    m_concat->clear();
}

void BackgroundDebugger::StopRun()
{
    Preferences* prefs;

    m_csv->close();
    CloseFile();

    prefs = Preferences::GetInstance();

    prefs->PutInt("DebugRun",m_runNum);
    prefs->Save();

    f_running = false;
    m_runPath = "/home/lvuser/DisabledLogs/";
}

void BackgroundDebugger::SetManualLoggingName (string fName)
{
	m_manualLog = fName + "_manual.txt";
	m_manualLogPath = m_runPath + m_manualLog;
}

void BackgroundDebugger::Update()
{
    if (f_running)
    {

        string message;
        vector<SensorData>::iterator snit;
        vector<NumData>::iterator nit;
        vector<StringData>::iterator stit;

        message = m_tempMsg;

        //Logging
        if ((time(NULL) - *m_lastDebugTime) >= m_debugInterval)
        {
        	LogData("File Status",m_csv->is_open());
            if (m_csv->is_open())
            {
				m_csv->writeCell((float)(time(NULL) - *m_startTime));
				if (message != "")
				{
					m_csv->writeCell(message);
					m_csv->newRow();
					m_csv->writeCell((float)(time(NULL) - *m_startTime));
				}

				for (snit = m_sensorList.begin(); snit != m_sensorList.end(); ++snit)
					m_csv->writeCell((float)((*snit).source->PIDGet()));
				for (nit = m_numList.begin(); nit != m_numList.end(); ++nit)
					m_csv->writeCell((float)*((*nit).value));
				for (stit = m_stringList.begin(); stit != m_stringList.end(); ++stit)
					m_csv->writeCell(*((*stit).value));
            }

            time(m_lastDebugTime);
            if (m_tempMsg.compare(message) == 0)
                m_tempMsg = "";
        }

        if (DriverStation::GetInstance()->IsAutonomous())
        {
            if (m_caseTime == NULL)
                time(m_caseTime);

            watchAuton();
        }
        else if (m_caseTime != NULL)
            m_caseTime = NULL;
    }
}

void BackgroundDebugger::PrintData()
{
	SmartDashboard::PutBoolean("Auton Good",GetAutonState());
}

void BackgroundDebugger::watchAuton()
{
    if ((*m_autonCase > 0 && *m_autonCase < m_endCase) && f_watchAuton && f_autonState)
    {
        if (*m_autonCase > m_lastCase)
        {
            time(m_caseTime);
            m_lastCase = *m_autonCase;
        }

        if (difftime(time(NULL),*m_caseTime) > m_caseDuration)
        {
            f_autonState = false;
            dumpAuton();
        }
    }
}

void BackgroundDebugger::dumpAuton()
{
	unsigned x;

	(*m_concat) << m_runPath << "AUTON_FAIL.txt";

    m_fout.open(m_concat->str().c_str());

    m_fout<<"AUTONOMOUS FAILURE DETECTED AT "<<DriverStation::GetInstance()->GetMatchTime()<<" s"<<endl<<endl;
    m_fout<<"BackgroundDebugger created this file because it detected that case "<<(*m_autonCase);
    m_fout<<" of the autonomous mode you ran did not advance to the next case inside of "<<m_caseDuration<<" seconds."<<endl<<endl;
    m_fout<<"Below is a dump of all sensor and time data available to BackgroundDebugger."<<endl<<endl;
    m_fout<<"Maximum configured case duration: "<<m_caseDuration<<endl<<"Case run time: "<<difftime(time(NULL),*m_caseTime)<<endl<<endl;

    for (x = 0; x < m_sensorList.size(); x++)
    	m_fout<<m_sensorList[x].id<<": "<<m_sensorList[x].source->PIDGet()<<endl;
    for (x = 0; x < m_numList.size(); x++)
        m_fout<<m_numList[x].id<<": "<<(*m_numList[x].value)<<endl;
    for (x = 0; x < m_stringList.size(); x++)
        m_fout<<m_stringList[x].id<<": "<<(*m_stringList[x].value)<<endl;

    m_fout.close();
    m_concat->clear();
    m_concat->str("");
}
