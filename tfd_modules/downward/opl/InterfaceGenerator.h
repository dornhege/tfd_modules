/*
 * InterfaceGenerator.h
 *
 *  Created on: May 19, 2011
 *      Author: Andreas Hertle
 */

#ifndef INTERFACEGENERATOR_H_
#define INTERFACEGENERATOR_H_

#include <QDir>
#include <QFile>
#include <QSet>
#include "Element.h"
#include "Type.h"

namespace opl
{

class InterfaceGenerator
{
    QDir baseOutputDirectory;
    QDir outputDirectory;
    QDir templateDirectory;
    QHash<QString, QString> templates;
    QSet<QString> generatedSourceFiles;

public:
    InterfaceGenerator(const QString& templatePath, const QString& outputPath);
    virtual ~InterfaceGenerator();

    void createInterface(const Element* domain);
private:
    QString readTemplateFile(const QString& templateName);
    void generateTypeFiles(const Type* type, const QString& domainName);
    QString generateIncludes(const QSet<QString> types);
    void generateModule(const Element* module, const QString& domainName);
    void generateFluent(const Element* fluent,
            QString& fluentDeclaration,
            QString& fluentImplementation,
            QString& variables,
            QString& initialization,
            QSet<QString>& includeTypes);
    QString generateObjectTablePlacement(const Type*);
    void writeToFile(const QString& content, const QString& filename, bool replaceExisting = true);
};

}

#endif /* INTERFACEGENERATOR_H_ */
