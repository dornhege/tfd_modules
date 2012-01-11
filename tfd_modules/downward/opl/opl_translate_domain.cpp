/*
 * main.cpp
 *
 *  Created on: Nov 19, 2010
 *      Author: Andreas Hertle
 */

//#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <assert.h>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include "parsing/Translator.h"
#include "Element.h"
#include "parsing/Parser.h"
#include "InterfaceGenerator.h"
#include "OPLSemantic.h"

using namespace std;

int main (int argc, char* argv[])
{
    if (argc != 3 && argc != 2)
    {
        printf("wrong parameter count.\n");
        printf("\n  opl_translate_domain <domain file> [<destination directory>]\n");
        printf("\naborting\n");
        return -1;
    }
    QString domainFileName = argv[1];
    int nameStartIndex = domainFileName.lastIndexOf("/") + 1;
    int nameEndIndex = domainFileName.lastIndexOf(".");
    QString domainName = domainFileName.mid(nameStartIndex, nameEndIndex - nameStartIndex);

    QString destinationDirectory = "/tmp/";
    if (argc == 3)
    {
        destinationDirectory = argv[2];
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

    QDir directory(destinationDirectory);
    if (! directory.exists())
    {
        directory.mkpath(".");
    }
    QString pddlDomainFileName = destinationDirectory + domainName + ".pddl";
    cout << "Writing domain: " << qPrintable(pddlDomainFileName) << endl;
    QFile domainFile(pddlDomainFileName);
    if (domainFile.exists())
    {
        domainFile.remove();
    }
    assert(domainFile.open(QFile::WriteOnly));
    QTextStream out(&domainFile);
    out <<  qPrintable(domain->toPDDL()) << flush;
    domainFile.close();

    // generate C++ module interface
    QString basePath = getenv("TFD_HOME");
    basePath += "/downward/opl/";
    cout << "Reading templates from: " << qPrintable(basePath) << "templates" << endl;
    cout << "Generating module interface to: " << qPrintable(basePath) << "generated" << endl;
    opl::InterfaceGenerator generator(basePath+"templates", basePath+"generated");
    generator.createInterface(domain);
    cout << "Done "<< endl;

    return 0;

}
