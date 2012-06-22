/*
 * KeyWordList.cpp
 *
 *  Created on: Jun 14, 2011
 *      Author: Andreas Hertle
 */

#include "KeyWordList.h"

namespace opl
{

KeyWordList* KeyWordList::instance = NULL;

KeyWordList::KeyWordList()
{
    wordPattern = QRegExp("\\d[\\.\\d]*|(\\w[\\w|\\d|_|@]*)");
    keyWords.insert("Domain");
    keyWords.insert("Type");
    keyWords.insert("Problem");
    keyWords.insert("Goal");
    keyWords.insert("predicate");
    keyWords.insert("number");
    keyWords.insert("ConditionModule");
    keyWords.insert("CostModule");
    keyWords.insert("EffectModule");
    functions.insert("atStart");
    functions.insert("atEnd");
    functions.insert("overall");
    functions.insert("and");
    functions.insert("or");
    functions.insert("not");
    functions.insert("implies");
    functions.insert("forAll");
    functions.insert("exists");
    functions.insert("when");
    functions.insert("assign");
    functions.insert("equals");
    functions.insert("greaterEqual");
    functions.insert("decrease");
    functions.insert("increase");
    operators.insert(".");
    operators.insert("=");
    operators.insert(">");
    operators.insert("<");
}

KeyWordList::~KeyWordList()
{
}

void KeyWordList::instantiate()
{
    if (instance == NULL)
    {
        instance = new KeyWordList();
    }
}

bool KeyWordList::isValidName(const QString& word)
{
    instantiate();
    if (instance->isWord(word)
            && ! instance->functions.contains(word)
            && ! instance->keyWords.contains(word))
    {
        return true;
    }
    return false;
}

}
