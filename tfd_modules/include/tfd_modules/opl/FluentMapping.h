/*
 * FluentMapping.h
 *
 *  Created on: Jul 13, 2011
 *      Author: Andreas Hertle
 */

#ifndef FLUENTMAPPING_H_
#define FLUENTMAPPING_H_

#include <string>

namespace opl
{

namespace interface
{

class FluentMapping
{
    int index;
    double value;
    bool constant;

public:
    FluentMapping(int index);
    FluentMapping(int index, double value, bool constant = false);
    virtual ~FluentMapping();

    int getIndex() const {return index;}
    double getValue() const {return value;}
    bool isConstant() const {return constant;}

    std::string dump() const;
};

}

}

#endif /* FluentMapping_H_ */
