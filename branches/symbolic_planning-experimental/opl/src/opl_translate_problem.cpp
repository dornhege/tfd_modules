/*
 * main.cpp
 *
 *  Created on: Nov 19, 2010
 *      Author: Andreas Hertle
 */

#include <iostream>
#include <assert.h>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include "parsing/Translator.h"
#include "OPLSemantic.h"

using namespace std;

int main (int argc, char* argv[])
{
    if (argc != 3 && argc != 4)
    {
        printf("wrong parameter count.\n");
        printf("\n  opl_translate_problem <domain file> <problem file> [<destination directory>]\n");
        printf("\naborting\n");
        return 0;
    }
    QString domainFileName = argv[1];
    QString problemFileName = argv[2];
    int nameStartIndex = problemFileName.lastIndexOf("/") + 1;
    int nameEndIndex = problemFileName.lastIndexOf(".");
    QString problemName = problemFileName.mid(nameStartIndex, nameEndIndex - nameStartIndex);
    QString destinationDirectory = "/tmp/";
    if (argc == 4)
    {
        destinationDirectory = argv[3];
        if (! destinationDirectory.endsWith("/"))
        {
            destinationDirectory.append("/");
        }
    }


    cout << "Parsing domain file: " << qPrintable(domainFileName) << endl;
    opl::Translator translator;
    QStringList tokens;
    translator.preprocessFile(domainFileName, tokens);
    QStringListIterator it(tokens);

    opl::Parser* parser = new opl::Parser();
    opl::ParseUnit* topLevel = parser->parse(it);

    opl::OPLSemantic semantic;
    QStringList topLevelKeywords;
    topLevelKeywords.append(QString("Domain"));
    opl::Element* domain = semantic.getDomainDetector().detect(topLevel, topLevelKeywords);
    domain->evaluate();

    cout << "Parsing problem file: " << qPrintable(problemFileName) << endl;
    tokens.clear();
    translator.preprocessFile(problemFileName, tokens);
    it = QStringListIterator(tokens);
    opl::ParseUnit* topLevelProblem = parser->parse(it);
//    cout << *(topLevelProblem);

    topLevelKeywords.clear();
    topLevelKeywords.append(QString("Problem"));
    opl::Element* problem = semantic.getProblemDetector().detect(topLevelProblem, topLevelKeywords);
    problem->setParent(domain);
    problem->evaluate();

    QDir directory(destinationDirectory);
    if (! directory.exists())
    {
        directory.mkpath(".");
    }
    QString pddlProblemFileName = destinationDirectory + problemName + ".pddl";
    cout << "Writing problem: " << qPrintable(pddlProblemFileName) << endl;
    QFile problemFile(pddlProblemFileName);
    if (problemFile.exists())
    {
        problemFile.remove();
    }
    assert(problemFile.open(QFile::WriteOnly));
    QTextStream out(&problemFile);
    out <<  qPrintable(problem->toPDDL()) << flush;
    problemFile.close();
    cout << "Done "<< endl;

    return 0;

}
