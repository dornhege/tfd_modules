/*
 * StringReplace.cpp
 *
 *  Created on: Aug 10, 2011
 *      Author: Andreas Hertle
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <assert.h>
#include <QRegExp>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <Qt>
#include "StringReplace.h"

using namespace std;

namespace opl
{

namespace tools
{

StringReplace::StringReplace()
{
}

StringReplace::~StringReplace()
{
}

void StringReplace::loadReplacementDefinitionFile(const QString& fileName)
{
    QFile file(fileName);
    assert(file.exists());
    assert(file.open(QFile::ReadOnly));
    QTextStream buffer(&file);
    QString fileContent = buffer.readAll();
    file.close();
    QStringListIterator lineIterator(fileContent.split("\n", QString::SkipEmptyParts));
    while (lineIterator.hasNext())
    {
        QStringList replacement = lineIterator.next().split("|", QString::SkipEmptyParts);
        replacements.append(QPair<QString, QString>(replacement.first(), replacement.last()));
    }
}

void StringReplace::replaceInFile(const QRegExp& fileNamePattern, const QString& sourceDirectory, const QString& destinationDirectory)
{
    QDir sourceDir(sourceDirectory);
    assert(sourceDir.exists());
    QDir destinationDir(destinationDirectory);
    if (! destinationDir.exists())
    {
        destinationDir.mkpath(".");
    }
    QStringList content = sourceDir.entryList();
    QStringListIterator fileIterator(content);
    while (fileIterator.hasNext())
    {
        QString fileName = fileIterator.next();
//        cout << qPrintable(fileName) << endl;
//        cout << qPrintable(fileNamePattern) << endl;
        if (fileNamePattern.exactMatch(fileName))
        {
            QFile sourceFile(QDir::cleanPath(sourceDir.absoluteFilePath(fileName)));
            cout << "converting file: " << qPrintable(sourceFile.fileName()) << endl;
            assert(sourceFile.open(QFile::ReadOnly));
            QTextStream buffer(&sourceFile);
            QString fileContent = buffer.readAll();
//            cout << qPrintable(fileContent) << endl;
            sourceFile.close();
            QListIterator<QPair<QString, QString> > keyIterator(replacements);
            while (keyIterator.hasNext())
            {
                const QPair<QString, QString>& mapping = keyIterator.next();
                QString key = mapping.first;
                QString value = mapping.second;
                key.prepend("\\b").append("\\b");
                QRegExp wholeWord(key, Qt::CaseSensitive);
                fileContent.replace(wholeWord, value);
            }
            QFile destinationFile(QDir::cleanPath(destinationDir.absoluteFilePath(fileName)));
            destinationFile.remove();
            assert(destinationFile.open(QFile::WriteOnly));
            QTextStream outBuffer(&destinationFile);
            outBuffer << fileContent << flush;
            destinationFile.close();
        }
    }
}

}

}

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        printf("wrong parameter count.\n");
        printf("\n  stringReplace <replacementDefinitionFile> <fileNamePattern> <sourceDirectory> <destinationDirectory>\n");
        printf("\naborting...\n");
        return 0;
    }

    QString replacementDefinitionFileName = argv[1];
    QRegExp fileNamePattern(argv[2]);
    QString sourceDirectory = argv[3];
    QString destinationDirectory = argv[4];

    opl::tools::StringReplace replacer;
    replacer.loadReplacementDefinitionFile(replacementDefinitionFileName);
    replacer.replaceInFile(fileNamePattern, sourceDirectory, destinationDirectory);

    printf("\ndone.\n");
    return 0;
}
