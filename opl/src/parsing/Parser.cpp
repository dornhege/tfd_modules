/*
 * Parser.cpp
 *
 *  Created on: Apr 20, 2011
 *      Author: Andreas Hertle
 */

#include <iostream>
#include <assert.h>
#include "parsing/Parser.h"
#include "KeyWordList.h"

namespace opl
{

Parser::Parser()
{
}

Parser::~Parser()
{
}

ParseUnit* Parser::parse(QStringListIterator& tokenIterator)
{
    ParseUnit* element = new ParseUnit();

    // first token
    assert (tokenIterator.hasNext());
    QString token = tokenIterator.next();
//    std::cout << "Parser2::parse: " << qPrintable(token) << std::endl << std::flush;
    //assert(isWord(token));
    element->addToken(token);

    // more tokens
    while (tokenIterator.hasNext())
    {
        token = tokenIterator.peekNext();
        if (! KeyWordList::isWord(token) && token != ":")
        {
            break;
        }
        element->addToken(tokenIterator.next());
    }

    // bracket list
    if (tokenIterator.hasNext())
    {
        token = tokenIterator.peekNext();
        if (token == "(")
        {
            tokenIterator.next(); // (
//            std::cout << "Parser2::parse add bracket list: " << qPrintable(tokenIterator.peekNext()) << std::endl << std::flush;
            while (tokenIterator.hasNext() && tokenIterator.peekNext() != ")")
            {
                element->addArgument(parse(tokenIterator));
                if (tokenIterator.peekNext() != ")")
                {
                    if (tokenIterator.peekNext() != ",")
                    {
                        std::cout << "ERROR: Parser::parse bracket list: expected: ',' instead got: " << qPrintable(tokenIterator.peekNext()) << std::endl << std::flush;
                        dumpContenxt(tokenIterator);
                    }
                    assert(tokenIterator.next() == ",");
                }
            }
            tokenIterator.next(); // )
        }
    }

    // braces list
    if (tokenIterator.hasNext())
    {
        token = tokenIterator.peekNext();
        if (token == "{")
        {
            tokenIterator.next(); // {
//            std::cout << "Parser2::parse add braces list: " << qPrintable(tokenIterator.peekNext()) << std::endl << std::flush;
            // read semicolon separated list
            while (tokenIterator.hasNext() && tokenIterator.peekNext() != "}")
            {
                element->addToBody(parse(tokenIterator));
                if (tokenIterator.peekPrevious() != "}")
                {
                    if (tokenIterator.peekNext() != ";")
                    {
                        std::cout << "ERROR: Parser::parse braces list: expected: ';' instead got: " << qPrintable(tokenIterator.peekNext()) << std::endl << std::flush;
                        dumpContenxt(tokenIterator);
                    }
                    assert(tokenIterator.next() == ";");
                }
            }
            tokenIterator.next(); // }
        }
        // TODO: improve operator parsing
        else if (token == ".")
        {
            tokenIterator.next(); // .
            element->setScopedAccess(parse(tokenIterator));
        }
        else if (token == "=")
        {
            tokenIterator.next(); // =
            element->addToBody(parse(tokenIterator));
        }
    }

    return element;
}

void Parser::dumpContenxt(QStringListIterator tokenIterator, int tokenCount) const
{
    std::cout << "current Parser context:" << std::endl;
    for (int i = 0; i < tokenCount; i++)
    {
        if (! tokenIterator.hasPrevious())
        {
            break;
        }
        tokenIterator.previous();
    }
    std::cout << "previous tokens:" << std::endl;
    for (int i = 0; i < tokenCount; i++)
    {
        if (! tokenIterator.hasNext())
        {
            break;
        }
        std::cout << " " << qPrintable(tokenIterator.next());
    }
    std::cout << std::endl << "current token: " << qPrintable(tokenIterator.next()) << std::endl;
    std::cout << "next tokens:" << std::endl;
    for (int i = 0; i < tokenCount; i++)
    {
        if (! tokenIterator.hasNext())
        {
            break;
        }
        std::cout << " " << qPrintable(tokenIterator.next());
    }
    std::cout << std::endl;
}


}
