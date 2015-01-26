/*
 * OPLSemantic.cpp
 *
 *  Created on: Aug 5, 2011
 *      Author: Andreas Hertle
 */

#include "OPLSemantic.h"

namespace opl
{

OPLSemantic::OPLSemantic()
{
    // build semantic structure of domain
    domainDetector.defineStructure(new opl::DomainStructure());
    domainDetector.defineStructure(new opl::TypeStructure());
    domainDetector.defineStructure(new opl::FluentStructure());
    domainDetector.defineStructure(new opl::ModuleStructure());
    domainDetector.defineStructure(new opl::SymbolDefinitionStructure());
    domainDetector.defineStructure(new opl::StringStructure());
    domainDetector.defineStructure(new opl::NumberStructure());
    domainDetector.defineStructure(new opl::SymbolAccessStructure());
    domainDetector.defineStructure(new opl::DurativeActionStructure());
    domainDetector.defineStructure(new opl::FunctionStructure());
    opl::NamedListStructure* list = new opl::NamedListStructure();
    list->setName("Effects");
    list->setKeyWord("EffectList");
    list->addListContentStructure("SymbolAccess");
    domainDetector.defineStructure(list);
    list = new opl::NamedListStructure();
    list->setName("Duration");
    list->setKeyWord("Duration");
    list->addListContentStructure("Number");
    list->addListContentStructure("SymbolAccess");
    domainDetector.defineStructure(list);
    list = new opl::NamedListStructure();
    list->setName("Condition");
    list->setKeyWord("Condition");
    list->addListContentStructure("Function");
    domainDetector.defineStructure(list);
    list = new opl::NamedListStructure();
    list->setName("Effect");
    list->setKeyWord("Effect");
    list->addListContentStructure("Function");
    domainDetector.defineStructure(list);

    // build semantic structure of problem
    problemDetector.defineStructure(new opl::ProblemStructure());
    problemDetector.defineStructure(new opl::ObjectStructure());
    problemDetector.defineStructure(new opl::InitializationStructure());
    problemDetector.defineStructure(new opl::FunctionStructure());
    problemDetector.defineStructure(new opl::SymbolDefinitionStructure());
    problemDetector.defineStructure(new opl::SymbolAccessStructure());
    problemDetector.defineStructure(new opl::NumberStructure());
    list = new opl::NamedListStructure();
    list->setName("Goal");
    list->setKeyWord("Goal");
    list->addListContentStructure("Function");
    problemDetector.defineStructure(list);
}

OPLSemantic::~OPLSemantic()
{
}

}
