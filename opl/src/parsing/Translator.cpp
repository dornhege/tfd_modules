/*
 * Translator.cpp
 *
 *  Created on: Nov 26, 2010
 *      Author: Andreas Hertle
 */

#include <iostream>
#include <assert.h>
#include <QFile>
#include <QTextStream>
#include <QRegExp>
#include <QString>
#include <QStringList>
#include "Translator.h"

namespace opl
{

Translator::Translator()
{
}

Translator::~Translator()
{
}

//Element* Translator::readDomain(const QString& domainFileName)
//{
//    QStringList tokens;
//    preprocessFile(domainFileName, tokens);
//    QStringListIterator it (tokens);
//
//    Element* rootElement = NULL;
//    assert(it.hasNext());
//    QString keyWord = it.next();
//    // TODO: Parser* parser = DomainParser();
////    rootElement = parser->parse(it);
//
//    return rootElement;
//}

void Translator::preprocessFile(const QString& domainFileName, QStringList& tokens)
{
//    std::cout << qPrintable(domainFileName) << std::endl;
    QFile domainFile(domainFileName);
    assert(domainFile.exists());
    assert(domainFile.open(QFile::ReadOnly));
//    std::cout << "reading" << std::endl;
    QTextStream in(&domainFile);
    QString data = in.readAll();
    domainFile.close();
//    std::cout << qPrintable(data) << std::endl;
    data.replace("(", " ( ");
    data.replace(")", " ) ");
    data.replace("{", " { ");
    data.replace("}", " } ");
    data.replace("[", " [ ");
    data.replace("]", " ] ");
    data.replace(";", " ; ");
    data.replace(",", " , ");
    data.replace(QRegExp("//[^\\n]*\\n"), " ");
    data.replace(QRegExp("/\\*.*\\*/"), " ");
    // FIXME: regEx for /* */ commentaries
//    data.replace(QRegExp("\\*/"), "_");
    tokens = data.split(QRegExp("\\s+"), QString::SkipEmptyParts);
    QMutableStringListIterator it(tokens);
    while (it.hasNext())
    {
        QString token = it.next();
        if (token.contains("."))
        {
            bool isNumber = false;
            token.toDouble(&isNumber);
            if (! token.contains("@") && ! isNumber)
            {
                it.remove();
                token.replace(QRegExp("\\.(?![0-9]+)"), " . ");
                QStringListIterator newTokens(token.split(QRegExp("\\s+"), QString::SkipEmptyParts));
                while (newTokens.hasNext())
                {
                    it.insert(newTokens.next());
                }
            }
        }
    }
//    data.replace(QRegExp("\\.(?![0-9]+)"), " . ");
}

}
