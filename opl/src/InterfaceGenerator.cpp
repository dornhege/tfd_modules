/*
 * InterfaceGenerator.cpp
 *
 *  Created on: May 19, 2011
 *      Author: Andreas Hertle
 */

#include <QFileInfo>
#include <QSet>
#include <QTextStream>
#include "InterfaceGenerator.h"
#include <ros/ros.h>

namespace opl
{

InterfaceGenerator::InterfaceGenerator(const QDir& templateDirectory, const QDir& projectDirectory, const QString& projectName)
: projectDirectory(projectDirectory), templateDirectory(templateDirectory), projectName(projectName)
{
    // load template files
    assert(templateDirectory.exists());
    QString templateName = "TypeHeader";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "TypeBody";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "StateHeader";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "StateBody";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "StateFactoryHeader";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "StateFactoryBody";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "ModuleAdaptorHeader";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "ModuleAdaptorBody";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "ModuleHeader";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "ModuleBody";
    templates.insert(templateName, readTemplateFile(templateName));
//    templateName = "moduleQmake";
//    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "manifest";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "Makefile";
    templates.insert(templateName, readTemplateFile(templateName));
    templateName = "CMakeLists";
    templates.insert(templateName, readTemplateFile(templateName));

    QString templateSnippets = readTemplateFile("Snippets");
    QStringListIterator it(templateSnippets.split("<:\n"));
    while (it.hasNext())
    {
        QStringList keyValue = it.next().split(":>");
        templates.insert(keyValue.first().trimmed(), keyValue.last());
    }
    generatedSourceFiles.clear();
}

InterfaceGenerator::~InterfaceGenerator()
{
}

QString InterfaceGenerator::readTemplateFile(const QString& templateName)
{
    QString fileName = templateName + "Template.txt";
    QFile templateFile(templateDirectory.filePath(fileName));
    assert(templateFile.exists());
    assert(templateFile.open(QFile::ReadOnly));
    QString content = QTextStream(&templateFile).readAll();
    templateFile.close();
    return content;
}

void InterfaceGenerator::createInterface(const Element* domain)
{
    srcDirectory = QDir(projectDirectory);
    srcDirectory.mkpath("src");
    srcDirectory.cd("src");

    QSet<QString> includeTypes;
    QList<const Element*> globalFluents;
    QString headerFiles;
    QString sourceFiles;

    // generate types and global fluents
    QString fluentDeclaration;
    QString fluentImplementation;
    QString variables;
    QString initialization;
    QListIterator<Element*> bodyIterator(domain->getBody());
    QList<const Type*> types;
    while (bodyIterator.hasNext())
    {
        const Element* element = bodyIterator.next();
        if (element->getType() == "Type")
        {
            const Type* type = reinterpret_cast<const Type*>(element);
            generateTypeFiles(type, domain->getName());
            includeTypes.insert(element->getName());
            types.append(type);
        }
        else if (element->getType() == "EffectModule" || element->getType() == "CostModule" || element->getType() == "ConditionModule")
        {
            generateModule(element, domain->getName());
        }
        else
        {
            generateFluent(element, fluentDeclaration, fluentImplementation, variables, initialization, includeTypes);
        }
    }

    // generate State
    QString typeLists;
    QString typeListGetters;
    QListIterator<const Type*> typeIterator(types);
    while (typeIterator.hasNext())
    {
        const QString& typeName = typeIterator.next()->getName();
        QString listSnippet = templates.value("TypeList");
        QString listGetterSnippet = templates.value("TypeListGetter");
        listSnippet.replace("<[Type]>", typeName);
        listGetterSnippet.replace("<[Type]>", typeName);
        QString listName = typeName;
        listName.replace(0, 1, listName[0].toLower()).append("s");
        listSnippet.replace("<[Name]>", listName);
        listGetterSnippet.replace("<[Name]>", listName);
        typeLists.append(listSnippet);
        typeListGetters.append(listGetterSnippet);
    }

    QString header = templates.value("StateHeader");
    header.replace("<[Include]>", generateIncludes(includeTypes));
    header.replace("<[Variables]>", variables);
    header.replace("<[FluentDeclaration]>", fluentDeclaration);
    header.replace("<[TypeLists]>", typeLists);
    header.replace("<[TypeListGetters]>", typeListGetters);
    header.replace("<[Namespace]>", domain->getName());
    generatedSourceFiles.insert("State");
    writeToFile(header, srcDirectory, "State.h");
    QString body = templates.value("StateBody");
    body.replace("<[Initialization]>", initialization);
    body.replace("<[FluentImplementation]>", fluentImplementation);
    body.replace("<[ObjectType]>", "State");
    body.replace("<[ObjectTypePrefix]>", "");
    body.replace("<[Namespace]>", domain->getName());
    writeToFile(body, srcDirectory, "State.cpp");

    // generate StateFactory
    generatedSourceFiles.insert("StateFactory");
    QString factoryHeader = templates.value("StateFactoryHeader");
    factoryHeader.replace("<[Namespace]>", domain->getName());
    writeToFile(factoryHeader, srcDirectory, "StateFactory.h");
    QString factoryBody = templates.value("StateFactoryBody");
    typeIterator.toFront();
    QString objectFactories;
    while (typeIterator.hasNext())
    {
        const Type* type = typeIterator.next();
        QString objectFactory = templates.value("ObjectFactory");
        objectFactory.replace("<[Type]>", type->getName());
        objectFactory.replace("<[LowerCaseType]>", type->getName().toLower());
        QString objectPlacements = generateObjectTablePlacement(type);
        objectFactory.replace("<[ObjectTablePlacements]>", objectPlacements);
        objectFactories.append(objectFactory);
    }
    factoryBody.replace("<[ObjectFactories]>",objectFactories);
    factoryBody.replace("<[Namespace]>", domain->getName());
    writeToFile(factoryBody, srcDirectory, "StateFactory.cpp");

    // generate project files
    QSetIterator<QString> sourceIterator(generatedSourceFiles);
    while (sourceIterator.hasNext())
    {
        QString name = sourceIterator.next();
        QString snippet = templates.value("HeaderFile");
        snippet.replace("<[FileName]>", name);
        headerFiles.append(snippet);
        snippet = templates.value("SourceFile");
        snippet.replace("<[FileName]>", name);
        sourceFiles.append(snippet);
    }
//    QString qmake = templates.value("moduleQmake");
//    qmake.replace("<[HeaderFiles]>", headerFiles);
//    qmake.replace("<[SourceFiles]>", sourceFiles);
//    qmake.replace("<[Namespace]>", domain->getName());
//    QString qmakeFileName = domain->getName();
//    qmakeFileName.append(".pro");
//    writeToFile(qmake, qmakeFileName);
    QString cmakelist = templates.value("CMakeLists");
    cmakelist.replace("<[SourceFiles]>", sourceFiles);
    cmakelist.replace("<[Namespace]>", projectName);
    writeToFile(cmakelist, projectDirectory, "CMakeLists.txt");
    QString manifest = templates.value("manifest");
    manifest.replace("<[Namespace]>", projectName);
    writeToFile(manifest, projectDirectory, "manifest.xml");
    QString makefile = templates.value("Makefile");
    writeToFile(makefile, projectDirectory, "Makefile");
}

void InterfaceGenerator::generateTypeFiles(const Type* type, const QString& domainName)
{
    QSet<QString> includeTypes;
    QString guard = domainName;
    guard.append(type->getName());
    QString superTypeName = "opl::interface::Object";
    const Element* superType = type->getSuperType();
    if (superType != NULL)
    {
        superTypeName = superType->getName();
        includeTypes.insert(superTypeName);
    }

    QString fluentDeclaration;
    QString fluentImplementation;
    QString variables;
    QString initialization;
    QListIterator<Element*> bodyIterator(type->getBody());
    while (bodyIterator.hasNext())
    {
        const Element* element = bodyIterator.next();
        if (element->getType() == "EffectModule" || element->getType() == "CostModule" || element->getType() == "ConditionModule")
        {
            generateModule(element, domainName);
        }
        else
        {
            generateFluent(element, fluentDeclaration, fluentImplementation, variables, initialization, includeTypes);
        }
    }

    QString includes = generateIncludes(includeTypes);

    // h file
    QString header = templates.value("TypeHeader");
    header.replace("<[Include]>", includes);
    header.replace("<[Variables]>", variables);
    header.replace("<[FluentDeclaration]>", fluentDeclaration);
    header.replace("<[SuperType]>", superTypeName);
    header.replace("<[ObjectType]>", type->getName());
    header.replace("<[Namespace]>", domainName);
    QString headerFileName = type->getName();
    generatedSourceFiles.insert(headerFileName);
    headerFileName.append(".h");
    writeToFile(header, srcDirectory, headerFileName);

    // cpp file
    QString body = templates.value("TypeBody");
    body.replace("<[Initialization]>", initialization);
    body.replace("<[FluentImplementation]>", fluentImplementation);
    body.replace("<[SuperType]>", superTypeName);
    body.replace("<[ObjectTypePrefix]>", "<[ObjectType]>_");
    body.replace("<[ObjectType]>", type->getName());
    body.replace("<[Namespace]>", domainName);
    QString bodyFileName = type->getName();
    bodyFileName.append(".cpp");
    writeToFile(body, srcDirectory, bodyFileName);
}

QString InterfaceGenerator::generateIncludes(const QSet<QString> types)
{
    QString includes;
    QSetIterator<QString> includeTypeIterator(types);
    while (includeTypeIterator.hasNext())
    {
        QString includeSnippet = templates.value("Include");
        includeSnippet.replace("<[Type]>", includeTypeIterator.next());
        includes.append(includeSnippet);
    }
    return includes;
}
void InterfaceGenerator::generateModule(const Element* module, const QString& domainName)
{
    QString moduleAdaptorHeader = templates.value("ModuleAdaptorHeader");
    QString moduleAdaptorBody = templates.value("ModuleAdaptorBody");
    QString moduleHeader = templates.value("ModuleHeader");
    QString moduleBody = templates.value("ModuleBody");
    QString baseArgumentSignature = "        <[Argument]>\n";
    const Element* thisElement = module->findThis();
    QString type;
    QString scopePrefix;
    QString baseFileName;
    int index = 0;
    QString argumentLookups;
    QString argumentNames;
    QString argumentSignatures;
    if (thisElement != NULL)
    {
        type = thisElement->getName();
        scopePrefix = type + "_";
        QString argumentLookup = templates.value("ArgumentLookup");
        argumentLookup.replace("<[ConstTypePointer]>", templates.value("ConstTypePointer"));
        argumentLookup.replace("<[Type]>", type);
        QString this_ptr = "this_pointer";
        argumentLookup.replace("<[Name]>", this_ptr);
        argumentLookup.replace("<[ArgumentIndex]>", QString::number(index));
        argumentLookups.append(argumentLookup);
        index++;
        argumentNames.append(this_ptr).append(", ");
        QString argumentSignature = baseArgumentSignature;
        argumentSignature.replace("<[Argument]>", templates.value("Argument"));
        argumentSignature.replace("<[Type]>", templates.value("ConstTypePointer"));
        argumentSignature.replace("<[Type]>", type);
        argumentSignature.replace("<[Name]>", this_ptr);
        argumentSignatures.append(argumentSignature);
    }
    baseFileName = scopePrefix + module->getName();
    generatedSourceFiles.insert(baseFileName);
    generatedSourceFiles.insert(baseFileName + "_plannerCall");
    QListIterator<Element*> argumentIterator(module->getArguments());
    while (argumentIterator.hasNext())
    {
        const Element* argument = argumentIterator.next();
        QString argumentLookup = templates.value("ArgumentLookup");
        argumentLookup.replace("<[ConstTypePointer]>", templates.value("ConstTypePointer"));
        argumentLookup.replace("<[Type]>", argument->getType());
        argumentLookup.replace("<[Name]>", argument->getName());
        argumentLookup.replace("<[ArgumentIndex]>", QString::number(index));
        argumentLookups.append(argumentLookup);
        index++;
        argumentNames.append(argument->getName()).append(", ");
        QString argumentSignature = baseArgumentSignature;
        argumentSignature.replace("<[Argument]>", templates.value("Argument"));
        argumentSignature.replace("<[Type]>", templates.value("ConstTypePointer"));
        argumentSignature.replace("<[Type]>", argument->getType());
        argumentSignature.replace("<[Name]>", argument->getName());
        argumentSignatures.append(argumentSignature);
    }
    if (module->getType() == "ConditionModule")
    {
        moduleAdaptorBody.replace("<[ReturnType]>", "bool");
        moduleAdaptorBody.replace("<[ReturnValueConversion]>", templates.value("ConditionModuleReturnConversion"));
        moduleHeader.replace("<[ReturnType]>", "bool");
        moduleBody.replace("<[ReturnType]>", "bool");
        moduleBody.replace("<[DefaultReturn]>", templates.value("DefaultConditionReturn"));
    }
    else if (module->getType() == "CostModule")
    {
        moduleAdaptorBody.replace("<[ReturnType]>", "double");
        moduleAdaptorBody.replace("<[ReturnValueConversion]>", templates.value("CostModuleReturnConversion"));
        moduleHeader.replace("<[ReturnType]>", "double");
        moduleBody.replace("<[ReturnType]>", "double");
        moduleBody.replace("<[DefaultReturn]>", templates.value("DefaultCostReturn"));
    }

    // Module Adaptor Header
    moduleAdaptorHeader.replace("<[Namespace]>", domainName);
    moduleAdaptorHeader.replace("<[Type]>", scopePrefix);
    moduleAdaptorHeader.replace("<[Name]>", module->getName());
    writeToFile(moduleAdaptorHeader, srcDirectory, baseFileName + "_plannerCall.h");

    // Module Adaptor Body
    moduleAdaptorBody.replace("<[ArgumentCount]>", QString::number(index));
    moduleAdaptorBody.replace("<[ArgumentLookups]>", argumentLookups);
    moduleAdaptorBody.replace("<[ArgumentNames]>", argumentNames);
    moduleAdaptorBody.replace("<[Namespace]>", domainName);
    moduleAdaptorBody.replace("<[Type]>", scopePrefix);
    moduleAdaptorBody.replace("<[Name]>", module->getName());
    writeToFile(moduleAdaptorBody, srcDirectory, baseFileName + "_plannerCall.cpp");

    // Module Header
    moduleHeader.replace("<[Arguments]>", argumentSignatures);
    moduleHeader.replace("<[Namespace]>", domainName);
    moduleHeader.replace("<[Type]>", scopePrefix);
    moduleHeader.replace("<[Name]>", module->getName());
    writeToFile(moduleHeader, srcDirectory, baseFileName + ".h");

    // Module Body
    moduleBody.replace("<[Arguments]>", argumentSignatures);
    moduleBody.replace("<[Namespace]>", domainName);
    moduleBody.replace("<[Type]>", scopePrefix);
    moduleBody.replace("<[Name]>", module->getName());
    writeToFile(moduleBody, srcDirectory, baseFileName + ".cpp", false);
}

void InterfaceGenerator::generateFluent(const Element* fluent,
        QString& fluentDeclaration,
        QString& fluentImplementation,
        QString& variables,
        QString& initializations,
        QSet<QString>& includeTypes)
{
    if (fluent->getType() != "DurativeAction" && fluent->getType() != "CostModule" && fluent->getType() != "ConditionModule" && fluent->getType() != "EffectModule" && fluent->getType() != "Type")
    {
        QString fluentSnippet = templates.value("FluentDeclaration");
        QString fluentBodySnippet = templates.value("FluentImplementation");
        bool directAccessFluent = false;
        if (fluent->getArguments().empty())
        {
            directAccessFluent = true;
        }

        QString arguments;
        QString key;
        QListIterator<Element*> argumentIterator(fluent->getArguments());
        while (argumentIterator.hasNext())
        {
            const Element* argument = argumentIterator.next();
            QString argumentSnippet = templates.value("Argument");
            QString type = templates.value("ConstTypePointer");
            type.replace("<[Type]>", argument->getType());
            argumentSnippet.replace("<[Type]>", type);
            argumentSnippet.replace("<[Name]>", argument->getName());
            arguments.append(argumentSnippet);
            includeTypes.insert(argument->getType());
            QString keySnippet = templates.value("KeyFragment");
            keySnippet.replace("<[ArgumentName]>", argument->getName());
            key.append(keySnippet);
        }
        if (arguments.endsWith(", "))
        {
            arguments = arguments.remove(arguments.length() - 2, 2);
        }
        fluentSnippet.replace("<[Arguments]>", arguments);
        fluentBodySnippet.replace("<[Arguments]>", arguments);

        if (directAccessFluent)
        {
            fluentBodySnippet.replace("<[FluentLookup]>", "");

            QString variable = templates.value("FluentVariable");
            variable.replace("<[FluentName]>", fluent->getName());
            variables.append(variable);

            QString initialization = templates.value("FluentLookup");
            initialization.replace("<[FluentName]>", fluent->getName());
            initialization.replace("<[Key]>", key);
            initializations.append(initialization);
        }
        else
        {
            QString lookup = templates.value("FluentVariable");
            lookup.append(templates.value("FluentLookup"));
            fluentBodySnippet.replace("<[FluentLookup]>", lookup);
            fluentBodySnippet.replace("<[Key]>", key);
        }

        if (fluent->getType() == "float")
        {
            fluentSnippet.replace("<[Type]>", "double");
            fluentBodySnippet.replace("<[Type]>", "double");
            fluentBodySnippet.replace("<[Function]>", templates.value("DoubleFunction"));
        }
        else if (fluent->getType() == "boolean")
        {
            fluentSnippet.replace("<[Type]>", "bool");
            fluentBodySnippet.replace("<[Type]>", "bool");
            fluentBodySnippet.replace("<[Function]>", templates.value("BooleanFunction"));
        }
        else
        {
            QString constTypePointer = templates.value("ConstTypePointer");
            constTypePointer.replace("<[Type]>", fluent->getType());
            fluentSnippet.replace("<[Type]>", constTypePointer);
            fluentBodySnippet.replace("<[Function]>", templates.value("ObjectFunction"));
            fluentBodySnippet.replace("<[Type]>", constTypePointer);
            includeTypes.insert(fluent->getType());
        }
        fluentSnippet.replace("<[FluentName]>", fluent->getName());
        fluentBodySnippet.replace("<[FluentName]>", fluent->getName());
//            std::cout << qPrintable(fluentSnippet) << std::endl;
        fluentDeclaration.append(fluentSnippet);
        fluentImplementation.append(fluentBodySnippet);
    }
}

QString InterfaceGenerator::generateObjectTablePlacement(const Type* type)
{
    QString placement = templates.value("ObjectTablePlacement");
    if (type == NULL)
    {
        placement.replace("<[lowerCaseType]>", "object");
    }
    else
    {
        QString typeName = type->getName();
        typeName.replace(0, 1, typeName.at(0).toLower());
        placement.replace("<[lowerCaseType]>", typeName);
        placement.append(generateObjectTablePlacement(type->getSuperType()));
    }
    return placement;
}

void InterfaceGenerator::writeToFile(const QString& content, QDir& directory, const QString& filename, bool replaceExisting)
{
    QFile generatedFile;
    generatedFile.setFileName(directory.filePath(filename));
    if (replaceExisting)
    {
        if (generatedFile.exists())
        {
            generatedFile.remove();
        }
        generatedFile.open(QFile::WriteOnly);
        QTextStream(&generatedFile) << content;
        generatedFile.close();
    }
    else
    {
        if (! generatedFile.exists())
        {
            generatedFile.open(QFile::WriteOnly);
            QTextStream(&generatedFile) << content;
            generatedFile.close();
        }
    }
}

}

