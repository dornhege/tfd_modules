#ifndef DOMAIN_PARSER_H
#define DOMAIN_PARSER_H

#include <QStringList>
#include <string>
#include <deque>
using std::deque;

class DomainParser
{
    public:
        DomainParser();
        ~DomainParser();

        bool parse(const std::string & filename);

        /// Returns the domain name as defined in the file.
        std::string getName() const { return _name; }

        void dumpTree() const;

    protected:
        /// An item in the parse tree.
        /**
         * items contains the child items. 
         * Only if items is empty, the content should be filled with a string.
         */
        struct TreeItem {
            deque<TreeItem> items;
            std::string content;
        };

        /// Recursively parse a tree item.
        TreeItem parseTreeItem(QStringList & tokens);

        void parseContent();

        void dumpItem(const TreeItem & it, unsigned int indent) const;

    protected:
        std::string _name;

        TreeItem _parseTreeRoot;
};

#endif

