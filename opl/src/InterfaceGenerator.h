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
    QDir projectDirectory;
    QDir srcDirectory;
    QDir templateDirectory;
    QString projectName;
    QHash<QString, QString> templates;
    QSet<QString> generatedSourceFiles;

public:
    InterfaceGenerator(const QDir& templatePath, const QDir& outputPath, const QString& projectName);
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
    void writeToFile(const QString& content, QDir& directory, const QString& filename, bool replaceExisting = true);
};

}

#endif /* INTERFACEGENERATOR_H_ */
