/*
 * ParseUnit.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: Andreas Hertle
 */

#include "ParseUnit.h"

namespace opl
{

ParseUnit::ParseUnit()
{
    scopedAcceess = NULL;
}

ParseUnit::~ParseUnit()
{
}

std::ostream& operator<<(std::ostream& output, const ParseUnit& element)
{
    QStringListIterator tokenIterator(element.tokens);
    while (tokenIterator.hasNext())
    {
        output << qPrintable(tokenIterator.next()) << " ";
    }
    if (! element.arguments.empty())
    {
        output << "(";
        QListIterator<ParseUnit* > it(element.arguments);
        while (it.hasNext())
        {
            output << *(it.next());
            if (it.hasNext())
            {
                output << ", ";
            }
        }
        output << ")";
    }
    if (! element.body.empty())
    {
        output << "\n{\n";
        QListIterator<ParseUnit* > it(element.body);
        while (it.hasNext())
        {
            output << "\t" << *(it.next()) << ";\n";
        }
        output << "\n}\n";
    }
    if (element.scopedAcceess != NULL)
    {
        output << "." << *element.scopedAcceess;
    }
    return output;
}

}
