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
#include <ros/ros.h>
#include <ros/package.h>

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
    ros::init(argc, argv, "opl_translate_domain");
    QString domainFileName = argv[1];
    int nameStartIndex = domainFileName.lastIndexOf("/") + 1;
    int nameEndIndex = domainFileName.lastIndexOf(".");
    QString domainName = domainFileName.mid(nameStartIndex, nameEndIndex - nameStartIndex);

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

    QString destinationDirectory("/tmp/");
    if (argc == 3)
    {
        destinationDirectory = argv[2];
        if (! destinationDirectory.endsWith("/"))
        {
            destinationDirectory.append("/");
        }
    }
    QDir directory(destinationDirectory);
    if (! directory.exists())
    {
        directory.mkpath(".");
    }

    // setup directories
    QString projectName = "opl_"+domain->getName().toLower()+"_modules";
    QDir projectDirectory(directory);
    projectDirectory.mkpath(projectName);
    projectDirectory.cd(projectName);
    QDir domainDirectory(projectDirectory);
    domainDirectory.mkpath("domain");
    domainDirectory.cd("domain");

    // copy opl domain file
    QString oplDomainFileName = domainDirectory.absolutePath() +"/"+ domainName + ".opl";
    QFile oplDomain(domainFileName);
    oplDomain.copy(oplDomainFileName);

    // write pddl domain file
    QString pddlDomainFileName = domainDirectory.absolutePath() +"/"+ domainName + ".pddl";
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
    QString basePath(ros::package::getPath("opl").c_str());
    QDir templateDirectory(basePath);
    templateDirectory.cd("templates");
    ROS_ASSERT(templateDirectory.exists());
    cout << "Reading templates from: " << qPrintable(templateDirectory.absolutePath()) << endl;
    cout << "Generating module interface to: " << qPrintable(projectDirectory.absolutePath()) << endl;
    opl::InterfaceGenerator generator(templateDirectory, projectDirectory, projectName);
    generator.createInterface(domain);
    cout << "Done "<< endl;

    return 0;

}
