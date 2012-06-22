/*
 * KeyWordList.h
 *
 *  Created on: Jun 14, 2011
 *      Author: Andreas Hertle
 */

#ifndef KEYWORDLIST_H_
#define KEYWORDLIST_H_

#include <QSet>
#include <QRegExp>
#include <QString>

namespace opl
{

class KeyWordList
{
private:
    QSet<QString> keyWords;
    QSet<QString> functions;
    QSet<QString> operators;
    QRegExp wordPattern;

    static KeyWordList* instance;

    KeyWordList();
    static void instantiate();

public:
    virtual ~KeyWordList();

    static bool isKeyWord(const QString& word) {instantiate(); return instance->keyWords.contains(word);}
    static bool isFunction(const QString& word) {instantiate(); return instance->functions.contains(word);}
    static bool isOperator(const QString& word) {instantiate(); return instance->functions.contains(word);}
    static bool isWord(const QString& token) {instantiate(); return instance->wordPattern.exactMatch(token);}
    static bool isValidName(const QString& word);
};

}

#endif /* KEYWORDLIST_H_ */
