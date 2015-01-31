#ifndef BACKGROUNDDEBUGGER_H
#define BACKGROUNDDEBUGGER_H

#include "HotSubsystem.h"
#include "WPILib.h"
#include "tools/csvwriter.h"
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <cstdio>
#include <sstream>
#include <string>

#define DEBUG_INTERVAL 500 //ms
#define AUTON_CASE_DURATION 2.5 //s

using namespace std;

typedef struct {
    string id;
    double* value;
} NumData;

typedef struct {
    string id;
    string* value;
} StringData;

typedef struct {
    string id;
    PIDSource* source;
} SensorData;

class BackgroundDebugger: public HotSubsystem
{
public:
    friend class HotSubsystemHandler;

    /**
     * @brief BackgroundDebugger: Class constructor
     * @param debugInterval: The interval to write debug data at (in ms).
     * @param clearContents: Clear the contents of existing debug folders.
     */
    BackgroundDebugger(double debugInterval=DEBUG_INTERVAL, bool clearContents=true);
    ~BackgroundDebugger();

    // DEBUG FUNCTIONS ---------------------------------------------------------------------------
    /**
     * @brief AddValue: Adds a number/string/sensor to the debugger's watch list.
     * @param id: The identifier for the number/string/sensor.
     * @param value: The pointer to the value to debug. Note: if you are trying to write a sensor value,
     * 				 the object needs to inherit PIDSource.
     */
    void AddValue (string id, double* value);
    void AddValue (string id, string* value);
    void AddValue (string id, PIDSource* value);

    /**
     * @brief SetTempMessage: Set a message to print out into all of the automatic
     *        debug files. This is useful for printing messages to show where the program
     *        is in its execution.
     * @param msg: The message to print.
     */
    void SetTempMessage (string msg) { m_tempMsg = msg; }

    /**
     * @brief LogData: manually save data to a file
     * @param id: The identifier of the value
     * @param value: The value to print to file
     */
    void LogData(string id, double value);
    void LogData(string id, string value);

    /**
     * @brief StartRun: Start the debugger. Creates a new folder
     *        containing all of the debug info from the run.
     */
    void StartRun();

    /**
     * @brief StopRun: Disables the debugger.
     */
    void StopRun();

    /**
     * @brief CloseFile: Closes non-automatic debugging files.
     */
    void CloseFile();

    // AUTON DEBUGGER ---------------------------------------------------------------------------

    /* BackgroundDebugger provides functionality to watch the robot's autonomous mode. Should
     * a predetermined amount of time pass without the case variable changing,
     * BackgroundDebugger will automatically dump all of the data that it is
     * configured to handle to a file.
     *
     * BackgroundDebugger will watch the variable until any of the following happens:
     *
     *   -The robot leaves autonomous mode.
     *   -The case variable reaches the configured max case (set using setMaxAutonCase).
     *   -The case variable becomes negative.
     *
     * BackgroundDebugger will only watch the variable if the debugger is enabled (StartRun).
     */

    /**
     * @brief SetAutonCase: Sets the pointer to the auton case variable
     *        so BackgroundDebugger can watch it.
     * @param auton: A pointer to the auton case variable
     */
    void SetAutonCase (int* auton) { m_autonCase = auton; }

    /**
     * @brief SetMaxAutonCase: Sets the highest case that the case variable
     *        will reach. This is used to make BackgroundDebugger stop watching
     *        the case variable.
     * @param max: The maximum case value.
     */
    void SetMaxAutonCase (int max) { m_endCase = max; }

    /**
     * @brief SetCaseDuration: Sets the maximum duration between case changes.
     *        Set this to just over the execution time of your longest case.
     * @param interval: The maximum case duration.
     */
    void SetCaseDuration (double duration=AUTON_CASE_DURATION) { m_caseDuration = duration; }

    /**
     * @brief EnableWatch: Temporarily enable/disable watching the auton case.
     * @param watch: Whether to watch the auton case.
     */
    void EnableWatch (bool watch) { f_watchAuton = watch; }

    /**
     * @brief GetAutonState: Returns whether BackgroundDebugger has detected a fault in
     *        autonomous.
     * @return : True if autonomous ran without a detected fault, false if a fault was detected.
     */
    bool GetAutonState () { return f_autonState; }


    // CONFIGURATION FUNCTIONS ------------------------------------------------------------------

    /**
     * @brief ClearContentsOnNewRun: Clear the contents of a folder
     *        from a previous run that shares the same name as the
     *        current run.
     * @param clear: Whether or not to clear out the contents of old
     *        folders
     */
    void ClearContentsOnNewRun (bool clear=true) { f_delContents = clear; }

    /**
     * @brief ResetRunNumber: Resets the run number.
     */
    void ResetRunNumber ();

    /**
     * @brief SetManualLoggingName: Set the file to which
     *        LogData writes to data to
     * @param fName: File name to write manual log to (without the extension)
     */
    void SetManualLoggingName (string fName) { m_manualLog = fName+"_manual.txt"; }

    /**
     * @brief SetDebugInterval: Set the interval at which
     *        the debugger automatically writes data to file.
     * @param interval: The new interval for debugging in ms.
     */
    void SetDebugInterval (double interval) { m_debugInterval = interval; }

protected:
    void Update();
private:
    //Auton debugger
    void watchAuton();
    void dumpAuton();

    int* m_autonCase;
    int m_lastCase;
    int m_endCase;
    double m_caseDuration;
    bool f_autonState;
    bool f_watchAuton;
    time_t m_caseTime;

    //User-defined debugging data
    vector<NumData> m_numList;
    vector<StringData> m_stringList;
    vector<SensorData> m_sensorList;
    string m_tempMsg;
    string m_manualLog;

    stringstream* m_concat;
    ofstream m_fout;
    CSVWriter* m_csv;
    CSVWriter* m_manualCsv;
    double m_debugInterval;
    time_t m_lastDebugTime;
    time_t m_startTime;

    bool f_running;
    bool f_delContents;
    int m_runNum;
    string m_runPath;
};

#endif // BACKGROUNDDEBUGGER_H
