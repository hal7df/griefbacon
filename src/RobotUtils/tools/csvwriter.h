#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <fstream>
#include <string>
#include <cstdio>
using namespace std;

class CSVWriter
{
public:
   //Constructor
   CSVWriter(const char* fname="", int cols=0);

   //Destructor
   ~CSVWriter();
   

   //File handling
   void open(const char* fname);
   void close() { m_fout.close(); }

   bool is_open() { return m_fout.is_open(); }
   bool good() { return m_fout.good(); }
   bool bad() { return m_fout.bad(); }
   bool fail() { return m_fout.fail(); }

   //CSV functions
   bool writeCell(string str);
   bool writeCell(float data);
   bool writeCell(bool data);
   void newRow();

   void setColumns(int cols) { m_cols = cols; }
   int getColumns () { return m_cols; }
   
private:
   ofstream m_fout;

   int m_cols;
   int m_curCol;
};

#endif //CSVWRITER_H
