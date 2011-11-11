#include "domainParser.h"
#include <fstream>
#include <QRegExp>
#include <ros/ros.h>

DomainParser::DomainParser()
{
}

DomainParser::~DomainParser()
{
}

bool DomainParser::parse(const std::string & filename)
{
    std::ifstream f(filename.c_str());
    if(!f.good())
        return false;

    QString content;
    while(f.good()) {
        std::string l;
        getline(f, l);
        QString line(l.c_str());
        // strip comments
        int idx = line.indexOf(";");
        line = line.mid(0, idx);
        content += " " + line;
    }

    content = content.replace("(", " ( ");
    content = content.replace(")", " ) ");

    QStringList tokens = content.split(" ", QString::SkipEmptyParts);

    _parseTreeRoot = parseTreeItem(tokens);

    f.close();

    parseContent();

    return true;
}

/**
 * A tree item should be '( stuff )'
 */ 
DomainParser::TreeItem DomainParser::parseTreeItem(QStringList & tokens)
{
    ROS_ASSERT(tokens.size() >= 2);
    ROS_ASSERT(tokens[0] == "(");
    tokens.removeFirst();

    TreeItem ret;
    // fill ret.items
    while(true) {
        ROS_ASSERT(!tokens.isEmpty());
        if(tokens[0] == ")") {  // done.
            tokens.removeFirst();
            return ret;
        }
        if(tokens[0] == "(") {
            ret.items.push_back(parseTreeItem(tokens)); 
        } else {
            TreeItem si;
            si.content = qPrintable(tokens[0]);
            ret.items.push_back(si);
            tokens.removeFirst();
        }
    }

    return ret;
}

void DomainParser::parseContent()
{
    ROS_ASSERT(_parseTreeRoot.items.size() >= 2);
    ROS_ASSERT(_parseTreeRoot.items[0].content == "define");
    TreeItem & domainIt = _parseTreeRoot.items[1];
    ROS_ASSERT(domainIt.items.size() == 2);
    ROS_ASSERT(domainIt.items[0].content == "domain");
    _name = domainIt.items[1].content;
}

void DomainParser::dumpTree() const
{
    dumpItem(_parseTreeRoot, 0);
}

void DomainParser::dumpItem(const TreeItem & item, unsigned int indent) const
{
    if(item.items.empty()) {        // only string
        for(unsigned int i = 0; i < indent; i++) printf(" ");
        printf("%s\n", item.content.c_str());
        return;
    }
    for(deque<TreeItem>::const_iterator it = item.items.begin(); it != item.items.end(); it++) {
        dumpItem(*it, indent + 2);
    }
    printf("\n");
}


